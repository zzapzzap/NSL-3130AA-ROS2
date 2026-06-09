#!/usr/bin/env python3
"""
Extrinsic calibration: LiDAR point cloud ↔ Camera image.
Refactored from sierrabase_driver/src/extrinsic/extrinsic_calib_node.py.

Result is x_cam = R · x_lidar + t  (LiDAR frame → Camera frame).

Intrinsic source priority:
  a. --camera-info-yaml (explicit path)
  b. {output_dir}/{camera_id}/intrinsic.yml  (OpenCV FileStorage)
  c. ~/.ros/camera_info/camera_{camera_id}.yaml (ROS format)
  d. Live camera_info topic
Output: {output_dir}/{camera_id}/extrinsic.yml (OpenCV FileStorage, R 3x3 + t 3x1).
Dataset/corner files in {output_dir}/{camera_id}/extrinsic/.

Workflow:
  1. Launch alongside camera + lidar drivers.
  2. Terminal prompts: s=store frame, c=calibrate, r=reset, q=quit.
  3. [s] opens two pickers in sequence (you pick manually with Ctrl+Click):
       - AMP(LiDAR): Ctrl+Click N board points on the amplitude image. Enter fits a
         RANSAC plane over the picked region and projects each pick's ray onto the
         plane (so the stored 3D uses the plane, not the raw/noisy depth). Enter
         again accepts. The title reports inliers / plane RMS / correction size.
       - RGB(camera): Ctrl+Click the SAME N points in the SAME order.
       Both sides must yield exactly N points or the frame is discarded (nothing
       is written unless the LiDAR/RGB pair is complete and equal).
  4. Repeat [s] for 5+ frames from different viewpoints.
  5. [c] Runs solvePnPRansac → saves R/t YML + camera intrinsics YAML.
       A side-by-side amplitude|RGB debug image is saved per frame for review.
"""

import argparse
import multiprocessing
import os
import queue
import sys
import threading
from pathlib import Path

import cv2
import matplotlib
import numpy as np
import yaml

import rclpy
import message_filters
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2_util

matplotlib.use('TkAgg')

# Real-time terminal output even when stdout is a pipe (ros2 launch ExecuteProcess)
# or a spawned picker child. Runs at module import → also applies to spawn children.
try:
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
except (AttributeError, ValueError):
    pass

# ── ROI for fallback 3D point selection (metres) ─────────────────────────────
ROI = dict(x_min=-2.0, x_max=2.0, y_min=-1.0, y_max=1.0, z_min=0.0, z_max=3.0)

CV_BRIDGE = CvBridge()
_g_pause  = False
_g_store  = False
_g_calib  = False
_g_reset  = False
_g_lock   = threading.Lock()


# ── Camera intrinsics helpers ─────────────────────────────────────────────────

def _make_cam_params(K, D, P, dist_model, W, H):
    model = 'fisheye' if dist_model in ('equidistant', 'fisheye') else 'pinhole'
    return dict(camera_model=model, distortion_model=dist_model,
                camera_matrix=K, dist_coeffs=D, projection_matrix=P,
                image_width=W, image_height=H)


def _load_intrinsics_cv_yml(path: str) -> dict:
    """Load intrinsics from OpenCV FileStorage .yml (written by intrinsic_calibration_node.py)."""
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ValueError(f'Cannot open OpenCV FileStorage: {path}')
    K = fs.getNode('camera_matrix').mat()
    D = fs.getNode('distortion_coefficients').mat()
    dm_node = fs.getNode('distortion_model')
    dm = dm_node.string() if not dm_node.empty() else 'plumb_bob'
    W_node = fs.getNode('image_width')
    H_node = fs.getNode('image_height')
    W = int(W_node.real()) if not W_node.empty() else 0
    H = int(H_node.real()) if not H_node.empty() else 0
    fs.release()
    if K is None or K.size == 0:
        raise ValueError(f'camera_matrix empty in {path}')
    if D is None or D.size == 0:
        D = np.zeros((1, 5), np.float64)
    P = np.zeros((3, 4), np.float64)
    P[:3, :3] = K
    return _make_cam_params(K, D.reshape(-1, 1), P, dm or 'plumb_bob', W, H)


def _load_intrinsics_yaml(path: str) -> dict:
    """Load intrinsics from ROS camera_info YAML or custom flat YAML."""
    with open(path) as f:
        d = yaml.safe_load(f)

    if 'camera_matrix' in d:                              # ROS cameracalibrator format
        K = np.array(d['camera_matrix']['data'],          np.float64).reshape(3, 3)
        D = np.array(d['distortion_coefficients']['data'], np.float64).reshape(-1, 1)
        dist_model = d.get('distortion_model', 'plumb_bob')
        W, H = int(d.get('image_width', 0)), int(d.get('image_height', 0))
        P_data = d.get('projection_matrix', {}).get('data', [K[0, 0], 0, K[0, 2], 0,
                                                               0, K[1, 1], K[1, 2], 0,
                                                               0, 0, 1, 0])
        P = np.array(P_data, np.float64).reshape(3, 4)
    elif 'K' in d:                                        # custom flat format
        K = np.array(d['K'], np.float64).reshape(3, 3)
        D = np.array(d['D'], np.float64).reshape(-1, 1)
        dist_model = d.get('distortion_model', 'plumb_bob')
        W, H = int(d.get('image_width', 0)), int(d.get('image_height', 0))
        P = np.zeros((3, 4), np.float64)
        P[:3, :3] = K
    else:
        raise ValueError(f'Unrecognised camera YAML format: {path}')

    return _make_cam_params(K, D, P, dist_model, W, H)


def _cam_params_from_info_msg(msg: CameraInfo) -> dict:
    K = np.array(msg.k, np.float64).reshape(3, 3)
    D = np.array(msg.d, np.float64).reshape(-1, 1)
    if D.size == 0:
        D = np.zeros((5, 1), np.float64)
    P = np.array(msg.p, np.float64).reshape(3, 4)
    model = (msg.distortion_model or '').strip().lower()
    return _make_cam_params(K, D, P, model, int(msg.width), int(msg.height))


# ── PnP helpers ───────────────────────────────────────────────────────────────

def _solve_pnp(cam_params, corners_3d, corners_2d):
    obj = np.ascontiguousarray(corners_3d.reshape(-1, 1, 3), np.float64)
    K   = cam_params['camera_matrix']
    D   = cam_params['dist_coeffs']

    if cam_params['camera_model'] == 'fisheye':
        img_pts = cv2.fisheye.undistortPoints(
            np.ascontiguousarray(corners_2d.reshape(-1, 1, 2), np.float64),
            K, D, P=np.eye(3, dtype=np.float64))
        solve_K, solve_D = np.eye(3, dtype=np.float64), None
    else:
        img_pts   = np.ascontiguousarray(corners_2d.reshape(-1, 1, 2), np.float64)
        solve_K, solve_D = K, D

    ok, r, t, inliers = cv2.solvePnPRansac(
        obj, img_pts, solve_K, solve_D, flags=cv2.SOLVEPNP_ITERATIVE)
    if ok and inliers is not None and len(inliers) >= 3:
        idx = inliers.reshape(-1)
        r, t = cv2.solvePnPRefineLM(obj[idx], img_pts[idx], solve_K, solve_D, r, t)
    return ok, r, t, inliers


def _project_points(cam_params, corners_3d, r, t):
    K, D = cam_params['camera_matrix'], cam_params['dist_coeffs']
    if cam_params['camera_model'] == 'fisheye':
        pts, _ = cv2.fisheye.projectPoints(
            corners_3d.reshape(-1, 1, 3).astype(np.float64), r, t, K, D)
    else:
        pts, _ = cv2.projectPoints(corners_3d.astype(np.float64), r, t, K, D)
    return pts.reshape(-1, 2)


def _rmse(proj, ref, inliers=None):
    proj = np.asarray(proj, np.float64).reshape(-1, 2)
    ref  = np.asarray(ref,  np.float64).reshape(-1, 2)
    if inliers is not None:
        idx = np.asarray(inliers).reshape(-1)
        proj, ref = proj[idx], ref[idx]
    return float(np.sqrt(np.mean(np.sum((proj - ref) ** 2, axis=1))))


# ── PCD save ──────────────────────────────────────────────────────────────────

def _save_pcd_rgb(pts_xyz, rgb_u8, path: str):
    """Write an ASCII XYZRGB PCD (PCL packed-rgb float field).

    rgb_u8: Nx3 uint8 (R, G, B). Read by RViz/CloudCompare/Open3D/pcl_viewer."""
    pts = np.asarray(pts_xyz, np.float64).reshape(-1, 3)
    rgb = np.asarray(rgb_u8, np.uint32).reshape(-1, 3)
    n = len(pts)
    packed = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
    rgb_f = np.ascontiguousarray(packed.astype(np.uint32)).view(np.float32)
    with open(path, 'w') as f:
        f.write('VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\n'
                'COUNT 1 1 1 1\n'
                f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n')
        for (x, y, z), c in zip(pts, rgb_f):
            f.write(f'{x:.6f} {y:.6f} {z:.6f} {float(c)!r}\n')


# ── LiDAR plane + amplitude marker helpers ───────────────────────────────────

def _imgmsg_to_u16(msg: Image) -> np.ndarray:
    arr = CV_BRIDGE.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    arr = np.asarray(arr)
    if arr.ndim == 3:
        arr = arr[:, :, 0]
    return arr.astype(np.uint16, copy=False)


def _pc2_to_arrays(pc_msg: PointCloud2):
    pts = pc2_util.read_points(
        pc_msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=False)
    arr = np.array(list(pts))

    if arr.size == 0:
        flat = np.empty((0, 4), dtype=np.float32)
        organized = None
        return flat, organized

    if arr.dtype.names:
        flat = np.column_stack([arr['x'], arr['y'], arr['z'], arr['intensity']]).astype(np.float32)
    else:
        flat = np.asarray(arr, dtype=np.float32).reshape(-1, 4)

    organized = None
    if pc_msg.height > 1 and flat.shape[0] == pc_msg.height * pc_msg.width:
        organized = flat.reshape(pc_msg.height, pc_msg.width, 4)

    valid = np.isfinite(flat[:, :3]).all(axis=1)
    return flat[valid], organized


def _fit_plane_svd(points_xyz: np.ndarray):
    centroid = points_xyz.mean(axis=0)
    _, _, vh = np.linalg.svd(points_xyz - centroid, full_matrices=False)
    normal = vh[-1]
    norm = np.linalg.norm(normal)
    if norm < 1e-9:
        return None
    normal = normal / norm
    d = -float(np.dot(normal, centroid))
    return normal.astype(np.float64), d


def _ransac_plane(points_xyz: np.ndarray, threshold=0.025, iterations=300):
    points_xyz = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
    if len(points_xyz) < 30:
        return None

    rng = np.random.default_rng(7)
    best_idx = None
    best_count = 0

    for _ in range(iterations):
        ids = rng.choice(len(points_xyz), 3, replace=False)
        p0, p1, p2 = points_xyz[ids]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        d = -float(np.dot(normal, p0))
        dist = np.abs(points_xyz @ normal + d)
        idx = np.flatnonzero(dist < threshold)
        if len(idx) > best_count:
            best_idx = idx
            best_count = len(idx)

    if best_idx is None or best_count < 20:
        return None

    refined = _fit_plane_svd(points_xyz[best_idx])
    if refined is None:
        return None
    normal, d = refined
    dist = np.abs(points_xyz @ normal + d)
    inliers = np.flatnonzero(dist < threshold)
    return normal, d, inliers


def _project_ray_to_plane(point_xyz, normal, d):
    ray = np.asarray(point_xyz, dtype=np.float64).reshape(3)
    ray_norm = np.linalg.norm(ray)
    if not np.isfinite(ray_norm) or ray_norm < 1e-6:
        return None
    direction = ray / ray_norm
    denom = float(np.dot(normal, direction))
    if abs(denom) < 1e-8:
        return None
    scale = -d / denom
    if scale <= 0.0:
        return None
    return direction * scale


def _nearest_valid_point(organized_xyzi, u, v, radius=3):
    h, w, _ = organized_xyzi.shape
    u = int(round(u))
    v = int(round(v))
    best = None
    best_dist2 = None
    for rr in range(radius + 1):
        x0, x1 = max(0, u - rr), min(w - 1, u + rr)
        y0, y1 = max(0, v - rr), min(h - 1, v + rr)
        patch = organized_xyzi[y0:y1 + 1, x0:x1 + 1, :3]
        valid = np.isfinite(patch).all(axis=2)
        if not valid.any():
            continue
        yy, xx = np.nonzero(valid)
        xx = xx + x0
        yy = yy + y0
        dist2 = (xx - u) ** 2 + (yy - v) ** 2
        idx = int(np.argmin(dist2))
        best = organized_xyzi[yy[idx], xx[idx], :3].astype(np.float64)
        best_dist2 = dist2[idx]
        break
    return best if best_dist2 is not None else None


def _append_points_csv(csv_path: Path, pts, width: int):
    existing = []
    if csv_path.exists() and csv_path.stat().st_size > 0:
        existing = np.loadtxt(str(csv_path), delimiter=',').reshape(-1, width).tolist()
    existing.extend(np.asarray(pts, dtype=np.float64).reshape(-1, width).tolist())
    np.savetxt(str(csv_path), np.array(existing).reshape(-1, width), delimiter=',')
    return len(existing)


def _amp_to_display(amplitude_u16):
    """Robust contrast stretch so the board is clearly visible.

    NSL amplitude is extremely high-dynamic-range: typically a few % of pixels are
    saturated near the 16-bit ceiling (bright reflectors / interference columns)
    while the board surface sits in the low hundreds. Scaling by a high percentile
    (or the max) crushes the board to black, so we drop the saturated cluster first,
    then stretch the remaining range and lift it with a mild gamma."""
    amp = np.asarray(amplitude_u16, dtype=np.float32)
    nz = amp[amp > 0]
    if nz.size == 0:
        return np.zeros(amp.shape, np.float32)
    peak = float(nz.max())
    unsat = nz[nz < 0.98 * peak]
    ref = unsat if unsat.size > 100 else nz
    lo = float(np.percentile(ref, 5.0))
    hi = float(np.percentile(ref, 98.0))
    if hi <= lo:
        hi = lo + 1.0
    vis = np.clip((amp - lo) / (hi - lo), 0.0, 1.0)
    return np.power(vis, 0.7)   # gamma < 1 lifts the board surface


# ── Frame store orchestration (parent process) ────────────────────────────────

def _drain_queue(q, proc):
    """Get one result from a picker subprocess without dead-locking if it crashes."""
    while proc.is_alive():
        try:
            return q.get(timeout=0.2)
        except queue.Empty:
            continue
    try:
        return q.get_nowait()
    except queue.Empty:
        return None


def _save_pair_debug(dataset_path, camera_id, idx, amplitude, img_bgr, amp_pixels, rgb_pts):
    """Side-by-side amplitude | RGB image with matching numbered markers for review."""
    ddir = Path(dataset_path) / 'debug'
    ddir.mkdir(parents=True, exist_ok=True)

    amp_c = cv2.applyColorMap((_amp_to_display(amplitude) * 255).astype(np.uint8),
                              cv2.COLORMAP_TURBO)
    rgb = img_bgr.copy()

    # Upscale the (low-res) amplitude to RGB height FIRST, then draw markers at the
    # same size as RGB. Drawing before the resize magnified the amp stars by `scale`
    # (~4.5×), burying the board under giant markers.
    scale = rgb.shape[0] / amp_c.shape[0]
    amp_r = cv2.resize(amp_c, (int(round(amp_c.shape[1] * scale)), rgb.shape[0]),
                       interpolation=cv2.INTER_NEAREST)

    for i, (u, v) in enumerate(amp_pixels, 1):
        p = (int(round(u * scale)), int(round(v * scale)))
        cv2.drawMarker(amp_r, p, (255, 255, 255), cv2.MARKER_STAR, 22, 2)
        cv2.putText(amp_r, str(i), (p[0] + 6, p[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)

    for i, (x, y) in enumerate(rgb_pts, 1):
        p = (int(round(x)), int(round(y)))
        cv2.drawMarker(rgb, p, (0, 0, 255), cv2.MARKER_STAR, 22, 2)
        cv2.putText(rgb, str(i), (p[0] + 6, p[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2, cv2.LINE_AA)

    out = ddir / f'{idx:02d}.png'
    cv2.imwrite(str(out), np.hstack([amp_r, rgb]))
    return out


def _store_one_frame(img_bgr, organized, amplitude, now_sec,
                     dataset_path, camera_id, expected_n):
    """Run the LiDAR(amplitude) picker, then the RGB picker, and commit BOTH
    atomically only when each yields exactly expected_n points. The pickers run in
    spawned subprocesses and return their points via a queue, so nothing is written
    unless the LiDAR/RGB pair is complete and equal in count.
    Returns True if a matched set was saved."""
    dp = Path(dataset_path)

    # ── 1) LiDAR side: amplitude Ctrl+Click + RANSAC plane correction ─────────
    q3 = multiprocessing.Queue()
    p3 = multiprocessing.Process(
        target=_pick_amplitude_points,
        args=(organized, amplitude, now_sec, expected_n, q3))
    p3.start()
    res3 = _drain_queue(q3, p3)
    p3.join()
    if not res3 or len(res3.get('points_3d', [])) != expected_n:
        print('[store] LiDAR side cancelled / wrong count — frame discarded.', flush=True)
        return False

    print(f'[store] LiDAR OK ({expected_n} pts). Now Ctrl+Click the SAME points on RGB.',
          flush=True)

    # ── 2) Camera side: RGB Ctrl+Click ────────────────────────────────────────
    q2 = multiprocessing.Queue()
    p2 = multiprocessing.Process(
        target=_pick_rgb_points,
        args=(img_bgr, now_sec, expected_n, q2))
    p2.start()
    res2 = _drain_queue(q2, p2)
    p2.join()
    if not res2 or len(res2.get('points_2d', [])) != expected_n:
        print('[store] RGB side cancelled / wrong count — frame discarded '
              '(LiDAR points were NOT saved).', flush=True)
        return False

    pts3 = np.asarray(res3['points_3d'], np.float64).reshape(-1, 3)
    pts2 = np.asarray(res2['points_2d'], np.float64).reshape(-1, 2)
    if len(pts3) != len(pts2):
        print(f'[store] pair count mismatch {len(pts3)} vs {len(pts2)} — frame discarded.',
              flush=True)
        return False

    # ── 3) Atomic commit (both CSVs together) ─────────────────────────────────
    dp.mkdir(parents=True, exist_ok=True)
    (dp / 'pointclouds').mkdir(parents=True, exist_ok=True)
    (dp / 'images').mkdir(parents=True, exist_ok=True)
    total3 = _append_points_csv(dp / '3D_corners.csv', pts3, 3)
    _append_points_csv(dp / '2D_corners.csv', pts2, 2)
    set_idx = total3 // expected_n - 1

    roi = res3.get('roi_points')
    if roi is not None and len(roi) > 0:
        roi_xyz = np.asarray(roi)[:, :3]
        cloud = np.vstack([roi_xyz, pts3])                          # region + the picked points
        colors = np.vstack([
            np.tile([255, 0, 0], (len(roi_xyz), 1)),                # region → red
            np.tile([0, 0, 255], (len(pts3), 1)),                   # each pick → one blue point
        ]).astype(np.uint32)
        _save_pcd_rgb(cloud, colors,
                      str(dp / 'pointclouds' / f'{set_idx:02d}.pcd'))
    cv2.imwrite(str(dp / 'images' / f'{set_idx:02d}.jpg'), img_bgr)
    dbg = _save_pair_debug(dataset_path, camera_id, set_idx,
                           amplitude, img_bgr, res3['pixels'], res2['points_2d'])

    rs = res3['ransac']
    print(f'[store] SAVED set #{total3 // expected_n}  ({expected_n} pairs)  | '
          f'RANSAC inliers={rs["n_inliers"]}/{rs["n_total"]} ({rs["ratio"]*100:.0f}%)  '
          f'plane_rms={rs["rms_mm"]:.1f}mm  | review → {dbg}', flush=True)
    return True


# ── Interactive pickers (Ctrl+Click, run inside spawned subprocesses) ─────────

def _pick_amplitude_points(organized, amplitude, now_sec, expected_n, out_q):
    """LiDAR side. Ctrl+Click board points on the amplitude image, fit a RANSAC
    plane over the picked region, and project each pick's ray onto that plane so the
    stored 3D point follows the plane geometry instead of the raw (noisy) depth.

    The amplitude image and the organized cloud share the same HxW grid, so a picked
    pixel (u, v) maps directly to cloud index [v, u].

    Returns via out_q: dict(points_3d, pixels, roi_points, ransac, corr_mm) or None."""
    import matplotlib.pyplot as plt
    from matplotlib import use as mpl_use
    mpl_use('TkAgg')
    plt.rcParams['keymap.home'] = ['h', 'home']   # free the 'r' key for reset

    if organized is None or amplitude is None:
        out_q.put(None)
        return
    h, w, _ = organized.shape
    if amplitude.shape[:2] != (h, w):
        print(f'[AMP] shape mismatch: cloud={w}x{h}, amp={amplitude.shape}. Frame skipped.',
              flush=True)
        out_q.put(None)
        return

    fig, ax = plt.subplots(figsize=(12, 9))
    ax.imshow(_amp_to_display(amplitude), cmap='turbo', origin='upper')
    ax.set_axis_off()

    state = {'pix': [], 'artists': [], 'accepted': False, 'result': None,
             'ctrl': False, 'alt': False, 'roi': None, 'roi_dragging': False,
             'roi_start': None, 'roi_artist': None}

    def set_title(msg=None):
        roi_txt = 'ROI set' if state['roi'] else 'Alt+drag = RANSAC region (optional)'
        ax.set_title(msg or (
            f't={now_sec}  AMP(LiDAR): Ctrl+Click {expected_n} board points  '
            f'[{len(state["pix"])}/{expected_n}]   {roi_txt}   '
            f'r=reset  Enter=fit, Enter again=accept'))

    def redraw():
        for a in state['artists']:
            try:
                a.remove()
            except Exception:
                pass
        state['artists'].clear()
        if state['pix']:
            arr = np.array(state['pix'])
            line, = ax.plot(arr[:, 0], arr[:, 1], '-', color='white', linewidth=1.0)
            state['artists'].append(line)
            for i, (u, v) in enumerate(state['pix'], 1):
                sc = ax.scatter([u], [v], c='red', s=110, marker='*',
                                edgecolors='white', linewidths=0.8, zorder=5)
                tx = ax.text(u + 3, v + 3, str(i), color='white', fontsize=12, zorder=6,
                             bbox=dict(facecolor='black', alpha=0.6, edgecolor='none'))
                state['artists'].extend([sc, tx])
        set_title()
        fig.canvas.draw_idle()

    def fit_and_show():
        if len(state['pix']) != expected_n:
            print(f'[AMP] need exactly {expected_n} points, have {len(state["pix"])}.', flush=True)
            return
        if state['roi'] is not None:
            x0, y0, x1, y1 = state['roi']           # user-drawn Alt+drag region
        else:
            pix = np.array(state['pix'])            # fallback: bbox of picks + margin
            mx = max(8.0, 0.25 * (pix[:, 0].max() - pix[:, 0].min()))
            my = max(8.0, 0.25 * (pix[:, 1].max() - pix[:, 1].min()))
            x0 = int(max(0, np.floor(pix[:, 0].min() - mx)))
            x1 = int(min(w - 1, np.ceil(pix[:, 0].max() + mx)))
            y0 = int(max(0, np.floor(pix[:, 1].min() - my)))
            y1 = int(min(h - 1, np.ceil(pix[:, 1].max() + my)))

        roi_cloud = organized[y0:y1 + 1, x0:x1 + 1, :3]
        roi_pts = roi_cloud[np.isfinite(roi_cloud).all(axis=2)].reshape(-1, 3)
        fit = _ransac_plane(roi_pts, threshold=0.02, iterations=500)
        if fit is None:
            set_title('[AMP] RANSAC plane fit FAILED — pick points on a flat board, then Enter.')
            print('[AMP] RANSAC plane fit failed (need a planar region with enough returns).',
                  flush=True)
            fig.canvas.draw_idle()
            return

        normal, d, inliers = fit
        n_in, n_tot = len(inliers), len(roi_pts)
        resid = np.abs(roi_pts[inliers] @ normal + d)
        rms_mm = float(np.sqrt(np.mean(resid ** 2))) * 1000.0

        corrected, corr_mm = [], []
        for (u, v) in state['pix']:
            raw = _nearest_valid_point(organized, u, v, radius=8)
            if raw is None:
                set_title('[AMP] a pick has no LiDAR return nearby — pick on the board '
                          'surface (not a hole/edge), reset (r) and re-pick.')
                print('[AMP] no valid LiDAR point near a pick. Reset and retry.', flush=True)
                fig.canvas.draw_idle()
                return
            proj = _project_ray_to_plane(raw, normal, d)
            if proj is None:
                set_title('[AMP] ray/plane projection failed — reset (r) and re-pick.')
                fig.canvas.draw_idle()
                return
            corrected.append([float(proj[0]), float(proj[1]), float(proj[2])])
            corr_mm.append(float(np.linalg.norm(proj - raw)) * 1000.0)

        rect, = ax.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0],
                        color='cyan', linewidth=1.4, zorder=4)   # plane-fit region
        state['artists'].append(rect)

        ratio = n_in / max(1, n_tot)
        msg = (f'[AMP] plane OK  inliers={n_in}/{n_tot} ({ratio*100:.0f}%)  '
               f'rms={rms_mm:.1f}mm  max_corr={max(corr_mm):.1f}mm  →  Enter=ACCEPT, r=redo')
        set_title(msg)
        print(msg, flush=True)
        print(f'[AMP] plane n=({normal[0]:.3f},{normal[1]:.3f},{normal[2]:.3f}) d={d:.3f}',
              flush=True)
        for i, (c, m) in enumerate(zip(corrected, corr_mm), 1):
            print(f'[AMP]   pt{i}: ({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f})  moved {m:.1f}mm',
                  flush=True)

        state['result'] = {
            'points_3d': corrected,
            'pixels': [[float(u), float(v)] for (u, v) in state['pix']],
            'roi_points': roi_pts,
            'ransac': {'n_inliers': int(n_in), 'n_total': int(n_tot),
                       'ratio': float(ratio), 'rms_mm': rms_mm,
                       'normal': [float(x) for x in normal], 'd': float(d)},
            'corr_mm': corr_mm,
        }
        fig.canvas.draw_idle()

    def _draw_roi_rect():
        if state['roi_artist'] is not None:
            try:
                state['roi_artist'].remove()
            except Exception:
                pass
            state['roi_artist'] = None
        if state['roi'] is not None:
            x0, y0, x1, y1 = state['roi']
            state['roi_artist'], = ax.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0],
                                           color='orange', linewidth=1.6,
                                           linestyle='--', zorder=3)

    def on_press(ev):
        if ev.inaxes != ax or ev.xdata is None:
            return
        mods = (ev.key or '').lower()
        alt = ('alt' in mods) or state['alt']
        ctrl = ('control' in mods) or ('ctrl' in mods) or state['ctrl']
        # Alt+left-drag or right-drag → define the RANSAC region
        if ev.button == 3 or (alt and ev.button == 1):
            state['roi_start'] = (float(np.clip(ev.xdata, 0, w - 1)),
                                  float(np.clip(ev.ydata, 0, h - 1)))
            state['roi_dragging'] = True
            return
        if ev.button != 1:
            return
        # Ctrl+left-click → pick a marker point
        if not ctrl:
            print('[AMP] click ignored — Ctrl+Click to pick a point, '
                  'Alt+drag (or right-drag) to set the RANSAC region.', flush=True)
            return
        if len(state['pix']) >= expected_n:
            print(f'[AMP] already have {expected_n}. Press Enter to fit, or r to reset.',
                  flush=True)
            return
        u = float(np.clip(ev.xdata, 0, w - 1))
        v = float(np.clip(ev.ydata, 0, h - 1))
        state['pix'].append((u, v))
        state['result'] = None   # picks changed → previous fit invalid
        print(f'[AMP] picked #{len(state["pix"])}: pixel ({u:.1f}, {v:.1f})', flush=True)
        redraw()
        _draw_roi_rect()

    def on_motion(ev):
        if not state['roi_dragging'] or ev.inaxes != ax or ev.xdata is None:
            return
        x0, y0 = state['roi_start']
        x1 = float(np.clip(ev.xdata, 0, w - 1))
        y1 = float(np.clip(ev.ydata, 0, h - 1))
        state['roi'] = (int(min(x0, x1)), int(min(y0, y1)),
                        int(max(x0, x1)), int(max(y0, y1)))
        _draw_roi_rect()
        fig.canvas.draw_idle()

    def on_btn_release(ev):
        if not state['roi_dragging']:
            return
        state['roi_dragging'] = False
        if state['roi'] is not None:
            x0, y0, x1, y1 = state['roi']
            if (x1 - x0) < 3 or (y1 - y0) < 3:          # too small → treat as cancel
                state['roi'] = None
                _draw_roi_rect()
            else:
                state['result'] = None
                print(f'[AMP] RANSAC region set: ({x0},{y0})-({x1},{y1}). '
                      'Ctrl+Click points, then Enter.', flush=True)
            set_title()
            fig.canvas.draw_idle()

    def onkey(ev):
        k = (ev.key or '').lower()
        if 'control' in k or 'ctrl' in k:
            state['ctrl'] = True
        if 'alt' in k:
            state['alt'] = True
        if ev.key == 'r':
            state['pix'].clear()
            state['result'] = None
            state['roi'] = None
            _draw_roi_rect()
            redraw()
            print('[AMP] reset (points + region).', flush=True)
        elif ev.key in ('enter', 'return'):
            if state['result'] is None:
                fit_and_show()
            else:
                state['accepted'] = True
                plt.close(fig)

    def on_key_release(ev):
        k = (ev.key or '').lower()
        if 'control' in k or 'ctrl' in k:
            state['ctrl'] = False
        if 'alt' in k:
            state['alt'] = False

    fig.canvas.mpl_connect('button_press_event', on_press)
    fig.canvas.mpl_connect('motion_notify_event', on_motion)
    fig.canvas.mpl_connect('button_release_event', on_btn_release)
    fig.canvas.mpl_connect('key_press_event', onkey)
    fig.canvas.mpl_connect('key_release_event', on_key_release)
    redraw()
    plt.tight_layout()
    plt.show()

    out_q.put(state['result'] if (state['accepted'] and state['result'] is not None) else None)


def _pick_rgb_points(img_bgr, now_sec, expected_n, out_q):
    """Camera side. Ctrl+Click the same points (same order) on the RGB image.
    Returns via out_q: dict(points_2d) or None."""
    import matplotlib.pyplot as plt
    from matplotlib import use as mpl_use
    mpl_use('TkAgg')
    plt.rcParams['keymap.home'] = ['h', 'home']

    fig, ax = plt.subplots(figsize=(14, 8))
    ax.imshow(img_bgr[:, :, ::-1])
    ax.set_axis_off()

    state = {'pts': [], 'artists': [], 'accepted': False, 'ctrl': False}

    def set_title():
        ax.set_title(
            f't={now_sec}  RGB(camera): Ctrl+Click the same {expected_n} points in order  '
            f'[{len(state["pts"])}/{expected_n}]   r=reset  Enter=accept')

    def redraw():
        for a in state['artists']:
            try:
                a.remove()
            except Exception:
                pass
        state['artists'].clear()
        if state['pts']:
            arr = np.array(state['pts'])
            line, = ax.plot(arr[:, 0], arr[:, 1], '-', color='lime', linewidth=1.2)
            state['artists'].append(line)
            for i, (x, y) in enumerate(state['pts'], 1):
                sc = ax.scatter([x], [y], c='red', s=120, marker='*',
                                edgecolors='white', linewidths=0.9, zorder=5)
                tx = ax.text(x + 6, y + 6, str(i), color='yellow', fontsize=13, zorder=6,
                             bbox=dict(facecolor='black', alpha=0.6, edgecolor='none'))
                state['artists'].extend([sc, tx])
        set_title()
        fig.canvas.draw_idle()

    def onclick(ev):
        if ev.inaxes != ax or ev.xdata is None:
            return
        mods = (ev.key or '').lower()
        if 'control' not in mods and 'ctrl' not in mods and not state['ctrl']:
            print('[RGB] click ignored — hold Ctrl and click to actually pick a point.',
                  flush=True)
            return
        if len(state['pts']) >= expected_n:
            print(f'[RGB] already have {expected_n}. Press Enter, or r to reset.', flush=True)
            return
        state['pts'].append((float(ev.xdata), float(ev.ydata)))
        print(f'[RGB] picked #{len(state["pts"])}: ({ev.xdata:.1f}, {ev.ydata:.1f})', flush=True)
        redraw()

    def onkey(ev):
        k = (ev.key or '').lower()
        if 'control' in k or 'ctrl' in k:
            state['ctrl'] = True
        if ev.key == 'r':
            state['pts'].clear()
            redraw()
            print('[RGB] reset.', flush=True)
        elif ev.key in ('enter', 'return'):
            if len(state['pts']) != expected_n:
                print(f'[RGB] need exactly {expected_n}, have {len(state["pts"])}.', flush=True)
                return
            state['accepted'] = True
            plt.close(fig)

    def onrelease(ev):
        k = (ev.key or '').lower()
        if 'control' in k or 'ctrl' in k:
            state['ctrl'] = False

    fig.canvas.mpl_connect('button_press_event', onclick)
    fig.canvas.mpl_connect('key_press_event', onkey)
    fig.canvas.mpl_connect('key_release_event', onrelease)
    redraw()
    plt.tight_layout()
    plt.show()

    if state['accepted'] and len(state['pts']) == expected_n:
        out_q.put({'points_2d': [list(p) for p in state['pts']]})
    else:
        out_q.put(None)


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class ExtrinsicCalibNode(Node):

    def __init__(self, params: dict):
        super().__init__('extrinsic_calib_node')
        self._p          = params
        self._lock       = threading.Lock()
        self._frame      = None
        self._cam_params = None
        self._subs       = []
        self._sync       = None

        name       = params['camera_id']
        output_dir = Path(os.path.expanduser(params['output_dir']))

        # ── Intrinsics source (priority order) ───────────────────────────────
        yaml_path    = params.get('camera_info_yaml', '')
        cv_yml_path  = output_dir / name / 'intrinsic.yml'
        ros_yml_path = Path.home() / '.ros' / 'camera_info' / f'camera_{name}.yaml'

        if yaml_path and Path(yaml_path).exists():
            self._cam_params = _load_intrinsics_yaml(yaml_path)
            self.get_logger().info(f'Intrinsics loaded from (explicit): {yaml_path}')
        elif cv_yml_path.exists():
            try:
                self._cam_params = _load_intrinsics_cv_yml(str(cv_yml_path))
                self.get_logger().info(f'Intrinsics loaded from (OpenCV YML): {cv_yml_path}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load {cv_yml_path}: {e}')
        elif ros_yml_path.exists():
            try:
                self._cam_params = _load_intrinsics_yaml(str(ros_yml_path))
                self.get_logger().info(f'Intrinsics auto-loaded from (ROS): {ros_yml_path}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load {ros_yml_path}: {e}')

        if self._cam_params is None:
            self._info_sub = self.create_subscription(
                CameraInfo, params['camera_info_topic'], self._cam_info_cb, 1)
            self.get_logger().info(
                f'Waiting for CameraInfo on {params["camera_info_topic"]} ...')

        # ── Synced image + point cloud (+ optional amplitude) ────────────────
        img_sub = message_filters.Subscriber(self, Image,       params['image_topic'])
        pc_sub  = message_filters.Subscriber(self, PointCloud2, params['lidar_topic'])
        amp_topic = params.get('amplitude_topic', '')
        if amp_topic:
            amp_sub = message_filters.Subscriber(self, Image, amp_topic)
            self._subs = [img_sub, pc_sub, amp_sub]
            self._sync = message_filters.ApproximateTimeSynchronizer(
                [img_sub, pc_sub, amp_sub], queue_size=8, slop=0.15)
            self._sync.registerCallback(self._frame_cb_amp)
            self.get_logger().info(
                f'Subscribed to image + point_cloud + amplitude ({amp_topic}). Ready.')
        else:
            self._subs = [img_sub, pc_sub]
            self._sync = message_filters.ApproximateTimeSynchronizer(
                [img_sub, pc_sub], queue_size=5, slop=0.1)
            self._sync.registerCallback(self._frame_cb)
            self.get_logger().info('Subscribed to image + point_cloud. Ready.')

    def _cam_info_cb(self, msg: CameraInfo):
        if self._cam_params is None:
            self._cam_params = _cam_params_from_info_msg(msg)
            self.get_logger().info('CameraInfo received — intrinsics ready.')

    def _frame_cb(self, img_msg: Image, pc_msg: PointCloud2):
        img_bgr = CV_BRIDGE.imgmsg_to_cv2(img_msg, 'bgr8')
        pts_np, organized = _pc2_to_arrays(pc_msg)
        now_sec = img_msg.header.stamp.sec
        with self._lock:
            self._frame = (img_bgr, pts_np, organized, None, now_sec)

    def _frame_cb_amp(self, img_msg: Image, pc_msg: PointCloud2, amp_msg: Image):
        img_bgr = CV_BRIDGE.imgmsg_to_cv2(img_msg, 'bgr8')
        pts_np, organized = _pc2_to_arrays(pc_msg)
        amplitude = _imgmsg_to_u16(amp_msg)
        now_sec = img_msg.header.stamp.sec
        with self._lock:
            self._frame = (img_bgr, pts_np, organized, amplitude, now_sec)

    def get_frame(self):
        with self._lock:
            return self._frame

    # ── Calibrate ─────────────────────────────────────────────────────────────

    def calibrate(self) -> bool:
        p    = self._p
        name = p['camera_id']
        dp   = Path(p['dataset_path'])

        c2d = dp / '2D_corners.csv'
        c3d = dp / '3D_corners.csv'
        if not c2d.exists() or not c3d.exists():
            self.get_logger().error('No stored corners. Press [s] first.')
            return False

        corners_2d = np.loadtxt(str(c2d), delimiter=',').reshape(-1, 2)
        corners_3d = np.loadtxt(str(c3d), delimiter=',').reshape(-1, 3)
        if len(corners_2d) != len(corners_3d):
            self.get_logger().error(
                f'2D/3D point count mismatch: {len(corners_2d)} vs {len(corners_3d)}. '
                'Reset or remove the last incomplete store.')
            return False
        if len(corners_2d) < 4:
            self.get_logger().error('Need at least 4 point pairs for solvePnP.')
            return False

        if self._cam_params is None:
            self.get_logger().error(
                'No camera intrinsics. Check CameraInfo topic or --camera-info-yaml.')
            return False

        cam = self._cam_params
        n   = len(corners_2d)
        self.get_logger().info(
            f'Calibrating with {n} point pairs '
            f'({cam["camera_model"]}, {cam["distortion_model"]})')

        ok, r, t, inliers = _solve_pnp(cam, corners_3d, corners_2d)
        if not ok:
            self.get_logger().error('solvePnPRansac failed.')
            return False

        reproj = _project_points(cam, corners_3d, r, t)
        rmse   = _rmse(reproj, corners_2d, inliers)
        n_in   = 0 if inliers is None else len(np.asarray(inliers).reshape(-1))
        self.get_logger().info(f'Inliers: {n_in}/{n}   RMSE: {rmse:.2f} px')

        R_mat = cv2.Rodrigues(r)[0]            # 3×3
        t_col = t.reshape(3, 1)               # 3×1

        with np.printoptions(precision=5, suppress=True):
            print('\n[c] ───── Extrinsic result  (x_cam = R · x_lidar + t) ─────', flush=True)
            print(f'[c] inliers={n_in}/{n}   RMSE={rmse:.2f} px', flush=True)
            print(f'[c] R =\n{R_mat}', flush=True)
            print(f'[c] t = {[round(float(x), 4) for x in t_col.reshape(3)]}  (metres)',
                  flush=True)
            cam_in_lidar = (-R_mat.T @ t_col).reshape(3)   # camera origin in LiDAR frame
            print(f'[c] camera origin in LiDAR frame = '
                  f'{[round(float(x), 4) for x in cam_in_lidar]}  (m)', flush=True)
            print('[c] ───────────────────────────────────────────────────────\n', flush=True)

        out = Path(os.path.expanduser(p['output_dir'])) / name
        out.mkdir(parents=True, exist_ok=True)

        # x_cam = R · x_lidar + t  (LiDAR frame → Camera frame)
        extr_path = out / 'extrinsic.yml'
        fs = cv2.FileStorage(str(extr_path), cv2.FILE_STORAGE_WRITE)
        fs.write('camera_id', name)
        fs.write('R', R_mat)
        fs.write('t', t_col)
        fs.release()
        self.get_logger().info(f'[Extrinsic YML saved] {extr_path}')

        # ── Save camera intrinsics alongside ─────────────────────────────────
        K  = cam['camera_matrix']
        D  = cam['dist_coeffs'].flatten()
        P  = cam['projection_matrix']
        cy = {
            'image_width':      cam['image_width'],
            'image_height':     cam['image_height'],
            'camera_model':     cam['camera_model'],
            'distortion_model': cam['distortion_model'],
            'K': K.flatten().tolist(),
            'D': D.tolist(),
            'fx': float(K[0, 0]),  'fy': float(K[1, 1]),
            'cx': float(K[0, 2]),  'cy': float(K[1, 2]),
            'fx_rectified': float(P[0, 0]), 'fy_rectified': float(P[1, 1]),
            'cx_rectified': float(P[0, 2]), 'cy_rectified': float(P[1, 2]),
            'rmse_px': round(float(rmse), 4),
        }
        cam_path = out / 'camera_info.yaml'
        cam_path.write_text(yaml.dump(cy, default_flow_style=False))
        self.get_logger().info(f'[Cam info saved]      {cam_path}')
        return True

    # ── Reset ─────────────────────────────────────────────────────────────────

    def reset(self):
        import shutil
        dp = Path(self._p['dataset_path'])
        if dp.exists():
            shutil.rmtree(str(dp))
        self.get_logger().info(f'Dataset reset: {dp}')


# ── Keyboard handler ──────────────────────────────────────────────────────────

def _keyboard_thread(stop_event: threading.Event):
    global _g_pause, _g_store, _g_calib, _g_reset

    # ros2 launch replaces stdin with a pipe — open /dev/tty to reach the terminal
    try:
        tty_file = open('/dev/tty', 'r')
    except OSError:
        tty_file = sys.stdin

    while not stop_event.is_set():
        sys.stdout.write('[s]=store frame  [c]=calibrate  [r]=reset  [q]=quit\n> ')
        sys.stdout.flush()
        try:
            line = tty_file.readline()
            if not line:
                break
            mode = line.strip().lower()
        except (EOFError, OSError):
            break
        with _g_lock:
            if   mode == 's':
                _g_store = True
                _g_pause = True
            elif mode == 'c':
                _g_calib = True
                _g_pause = True
            elif mode == 'r':
                _g_reset = True
                _g_pause = True
            elif mode == 'q':
                stop_event.set()

    if tty_file is not sys.stdin:
        tty_file.close()


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    global _g_pause, _g_store, _g_calib, _g_reset

    ap = argparse.ArgumentParser(
        description='NSL-3130AA extrinsic calibration (ROS2 Humble)')
    ap.add_argument('--camera-id',         default='nsl',
                    help='Camera ID (used in output filenames)')
    ap.add_argument('--image-topic',        default='/camera/rgb/image_raw')
    ap.add_argument('--camera-info-topic',  default='/camera/rgb/camera_info')
    ap.add_argument('--camera-info-yaml',   default='',
                    help='Explicit path to camera intrinsics YAML (skips other sources)')
    ap.add_argument('--lidar-topic',        default='/camera/point_cloud')
    ap.add_argument('--amplitude-topic',    default='/roboscanAmpl',
                    help='LiDAR amplitude image topic. Empty string disables amplitude-assisted 3D picking.')
    ap.add_argument('--points-per-frame',   type=int, default=5,
                    help='Number of corresponding RGB/LiDAR marker points stored per frame.')
    ap.add_argument('--output-dir',
                    default=os.path.join(os.getcwd(), 'calib_output'),
                    help='Directory for output files')
    args, ros_args = ap.parse_known_args()
    if args.points_per_frame < 4:
        print('[ERROR] --points-per-frame must be >= 4 for solvePnP.')
        sys.exit(1)

    camera_id   = args.camera_id
    output_dir  = os.path.expanduser(args.output_dir)
    dataset_path = os.path.join(output_dir, camera_id, 'extrinsic')

    # Guard: intrinsic file must exist before extrinsic calibration
    intr_file = Path(output_dir) / camera_id / 'intrinsic.yml'
    if not intr_file.exists():
        candidates = sorted(Path(output_dir).glob('*/intrinsic.yml')) if Path(output_dir).exists() else []
        print(f'\n[ERROR] Intrinsic file not found: {intr_file}')
        if candidates:
            detected_id = candidates[0].parent.name
            print(f'        Found: {candidates[0]}')
            print(f'        Re-run with the correct camera_id:')
            print(f'          ./setup/extrinsic_calib.sh {detected_id}')
        else:
            print(f'        No intrinsic files found in: {output_dir}')
            print(f'        Run intrinsic calibration first:')
            print(f'          ./setup/intrinsic_calib.sh')
        print()
        sys.exit(1)

    params = {
        'camera_id':          camera_id,
        'image_topic':        args.image_topic,
        'camera_info_topic':  args.camera_info_topic,
        'camera_info_yaml':   args.camera_info_yaml,
        'lidar_topic':        args.lidar_topic,
        'amplitude_topic':    args.amplitude_topic,
        'points_per_frame':   args.points_per_frame,
        'dataset_path':       dataset_path,
        'output_dir':         output_dir,
    }

    rclpy.init(args=ros_args or None)
    node = ExtrinsicCalibNode(params)

    stop_ev = threading.Event()
    spin_th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_th.start()
    kb_th   = threading.Thread(target=_keyboard_thread, args=(stop_ev,), daemon=True)
    kb_th.start()

    import time
    try:
        while rclpy.ok() and not stop_ev.is_set():
            with _g_lock:
                act = _g_pause

            if not act:
                time.sleep(0.05)
                continue

            with _g_lock:
                do_store = _g_store
                do_calib = _g_calib
                do_reset = _g_reset
                _g_store = _g_calib = _g_reset = _g_pause = False

            if do_store:
                print('[s] Capturing synced frame ...', flush=True)
                # Tolerate the startup race: the first synced frame can take ~1s.
                frame = node.get_frame()
                waited = 0.0
                while frame is None and waited < 3.0 and rclpy.ok():
                    time.sleep(0.1)
                    waited += 0.1
                    frame = node.get_frame()

                if frame is None:
                    print('[WARN] No synced image+point_cloud+amplitude frame within 3s.\n'
                          '       Check camera.launch.py is running and all topics publish '
                          '(ros2 topic hz ...).', flush=True)
                else:
                    img_bgr, _pts_np, organized, amplitude, now_sec = frame
                    expected_n = params['points_per_frame']
                    if organized is None or amplitude is None:
                        print('[WARN] Amplitude or organized cloud missing — the '
                              'amplitude+RANSAC picker needs both. Check --amplitude-topic '
                              'and that /camera/point_cloud is organized (height>1).',
                              flush=True)
                    else:
                        print('[s] LiDAR picker opening — Ctrl+Click the board points on the '
                              'amplitude window (a GUI window should appear) ...', flush=True)
                        if _store_one_frame(img_bgr, organized, amplitude, now_sec,
                                            params['dataset_path'], camera_id, expected_n):
                            print('[s] Frame stored. [s]=next pose  [c]=calibrate.', flush=True)
                        else:
                            print('[s] Nothing saved for this frame. Press [s] to retry.',
                                  flush=True)

            if do_calib:
                node.calibrate()

            if do_reset:
                node.reset()

    except KeyboardInterrupt:
        pass
    finally:
        stop_ev.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    multiprocessing.set_start_method('spawn')
    main()
