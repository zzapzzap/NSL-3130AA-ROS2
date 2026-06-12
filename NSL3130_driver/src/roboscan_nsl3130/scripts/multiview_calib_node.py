#!/usr/bin/env python3
"""
Multiview calibration node — STag-based shared reference frame.

Subscribes to one camera's RGB image, detects a STag marker, estimates the
marker's 6-DoF pose in the RGB camera optical frame, and saves the
marker <-> rgb relationship so several cameras can be tied to ONE common
reference frame (the marker), thereby sharing their extrinsics.

The saved R, t express the MARKER pose in the RGB CAMERA optical frame:

        x_cam = R · x_marker + t          (marker frame → camera frame)

so the camera's pose in the marker (reference) frame is its inverse,
(Rᵀ, −Rᵀ·t).  multiview.launch.py uses exactly this to anchor every camera
under the shared `stag_marker` frame and rejects any camera whose R|t is
missing/unreadable.

Outputs (serial = camera USB serial, the same key intrinsic/extrinsic use):

    calib_output/{serial}/multiview.yml          ← canonical R|t + metadata
    calib_output/{serial}/multiview/             ← debug images + summary

The camera intrinsics are read from calib_output/{serial}/intrinsic.yml.  The
NSL-3130 RGB lens is fisheye/equidistant, so corner pixels are undistorted with
the fisheye model before solvePnP (SOLVEPNP_IPPE_SQUARE), which is what gives a
correct metric pose.

Usage (normally via multiview_calib.launch.py):
    python3 multiview_calib_node.py \
        --camera-id N00A5060D --calib-dir /path/to/calib_output \
        --image-topic /cam_59/camera/rgb/image_raw \
        --library-hd 21 --marker-size 0.32
"""

import argparse
import datetime
import os
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Empty

try:
    import stag
except ImportError:
    print('[multiview_calib] ERROR: stag-python not installed.\n'
          '  Install with:  python3 -m pip install --user stag-python', file=sys.stderr)
    raise

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

try:
    from scipy.spatial.transform import Rotation as _Rot
except ImportError:
    _Rot = None

try:
    from sensor_msgs_py import point_cloud2 as _pc2
except ImportError:
    _pc2 = None


# ─────────────────────────── geometry helpers ───────────────────────────────

def _object_points(marker_size: float) -> np.ndarray:
    """Square-marker object points in the marker frame, ordered to match STag's
    corner output (TL, TR, BR, BL).  Marker lies in the Z=0 plane, +Z points out
    of the marker toward the camera, side = marker_size."""
    h = marker_size / 2.0
    return np.array([[-h,  h, 0.0],
                     [ h,  h, 0.0],
                     [ h, -h, 0.0],
                     [-h, -h, 0.0]], dtype=np.float64)


def _rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix → quaternion (x, y, z, w)."""
    m = np.asarray(R, dtype=np.float64).reshape(3, 3)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w, x = 0.25 * s, (m[2, 1] - m[1, 2]) / s
        y, z = (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x = (m[2, 1] - m[1, 2]) / s, 0.25 * s
        y, z = (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s
        y, z = 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s
        y, z = (m[1, 2] + m[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    return q / np.linalg.norm(q)


def _mean_rotation(rvecs: np.ndarray) -> np.ndarray:
    """Chordal-L2 mean of a stack of rotation vectors → 3x3 rotation matrix."""
    if _Rot is not None:
        return _Rot.from_rotvec(rvecs).mean().as_matrix()
    # Fallback: quaternion average with sign alignment.
    quats = []
    ref = None
    for rv in rvecs:
        R, _ = cv2.Rodrigues(rv)
        q = _rotmat_to_quat(R)
        if ref is None:
            ref = q
        if np.dot(q, ref) < 0:
            q = -q
        quats.append(q)
    q = np.mean(np.asarray(quats), axis=0)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _namespace_from_topic(topic: str) -> str:
    """'/cam_59/camera/rgb/image_raw' → 'cam_59'; '/camera/rgb/...' → ''."""
    parts = topic.strip('/').split('/')
    if len(parts) >= 2 and parts[0] != 'camera':
        return parts[0]
    return ''


def _cloud_to_xyz(msg):
    """PointCloud2 → (N,3) float64 array in the cloud frame, or None."""
    if _pc2 is None:
        return None
    arr = _pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    try:
        pts = np.stack([arr['x'], arr['y'], arr['z']], axis=-1).astype(np.float64)
    except (IndexError, ValueError, TypeError):
        pts = np.array([[p[0], p[1], p[2]] for p in arr], dtype=np.float64)
    return pts if pts.ndim == 2 and pts.shape[0] else None


def _ransac_offset(s, tol=0.10, iters=200):
    """1-D plane-offset RANSAC along a fixed (trusted) normal: find the largest
    set of projected depths `s` agreeing within `tol`. Returns (offset, inliers)."""
    s = np.asarray(s, np.float64)
    best_inl = np.abs(s - np.median(s)) < tol
    best_c = int(best_inl.sum())
    rng = np.random.default_rng(0)
    pivots = s if s.shape[0] <= iters else s[rng.integers(0, s.shape[0], iters)]
    for p in pivots:
        inl = np.abs(s - p) < tol
        c = int(inl.sum())
        if c > best_c:
            best_c, best_inl = c, inl
    return float(s[best_inl].mean()), best_c


# Per-tag physical size — hardcoded on purpose (the user finds a marker_size param
# confusing): tag id 7 is the big 0.32 m REFERENCE/origin marker; every other id is a
# 0.19 m auxiliary that just helps share/stabilise the TF tree under tag 7.
REF_ID = 7
SIZE_BY_ID = {REF_ID: 0.32}
AUX_SIZE = 0.19


def _size_of(tag_id):
    return SIZE_BY_ID.get(int(tag_id), AUX_SIZE)


# ─────────────────────────────── the node ───────────────────────────────────

class MultiviewCalibNode(Node):

    def __init__(self, args):
        # Distinct name in trigger/idle mode so it never clashes with a direct
        # `calibration:=true` instance on the same machine.
        if args.wait_trigger:
            node_name = f'multiview_calib_listener_{args.camera_id}' if args.camera_id else 'multiview_calib_listener'
        else:
            node_name = 'multiview_calib_node'
        super().__init__(node_name)
        self.a = args
        self.bridge = CvBridge() if CvBridge is not None else None

        self.serial = args.camera_id
        self.dev_dir = Path(args.calib_dir) / self.serial
        self.out_dir = self.dev_dir / 'multiview'
        self.out_dir.mkdir(parents=True, exist_ok=True)

        self.K, self.D, self.fisheye, (self.w, self.h) = self._load_intrinsics()

        self.ns = _namespace_from_topic(args.image_topic)
        self.camera_frame = f'{self.ns}_camera_frame' if self.ns else 'camera_frame'
        self.lidar_frame = f'{self.ns}_lidar_frame' if self.ns else 'lidar_frame'

        # Authoritative frame: read the live driver's point-cloud frame_id so the
        # saved frames always match what camera.launch.py actually publishes, even
        # when the launcher's IP-derived namespace guess is wrong (e.g. running the
        # calibration on a different machine than the camera). Best-effort: if no
        # cloud arrives we fall back to the image-topic-derived names above.
        self.live_lidar_frame = None
        cloud_topic = self._cloud_topic(args.image_topic)
        self.create_subscription(PointCloud2, cloud_topic, self._cloud_cb, 1)

        # Depth refinement: snap the marker depth to the LiDAR-measured plane.
        # Monocular STag gives a good rotation + viewing ray but a weak metric depth
        # (especially far / with marker_size error); the LiDAR fixes the depth.
        self._clouds = deque(maxlen=8)
        self._ext = self._load_extrinsic() if args.depth_refine else None
        if args.depth_refine and self._ext is None:
            self.get_logger().warn(
                'depth_refine on but extrinsic.yml missing → depth NOT refined '
                '(run extrinsic_calib first, or pass depth_refine:=false).')
        if args.depth_refine and _pc2 is None:
            self.get_logger().warn('sensor_msgs_py unavailable → depth refine disabled.')

        # accumulated good detections, per tag id: {id: {'rv':[], 'tv':[], 'err':[]}}
        self.tags = {}
        self.saved = False
        self.frame_count = 0
        self.last_save_dbg = -1
        self._collect_start = None   # monotonic time the reference tag was first seen (duration mode)
        self._last_save_time = None  # monotonic time of the last save (fleet trigger cooldown)

        # Fleet trigger: in wait-trigger mode the node idles (armed=False) and only
        # collects after an Empty message lands on trigger_topic (host broadcast),
        # re-arming after each save. In direct mode it is armed from the start.
        self.armed = not args.wait_trigger
        if args.wait_trigger:
            self.create_subscription(Empty, args.trigger_topic, self._on_trigger, 10)

        self.sub = self.create_subscription(Image, args.image_topic, self._cb, 1)
        if args.wait_trigger:
            mode = f'{args.duration:.0f}s window' if args.duration > 0 else f'{args.num_frames}-view'
            self.get_logger().info(
                f'[fleet] idle — waiting for {args.trigger_topic} '
                f'(then headless {mode} median calib for {self.serial}).')
        self.get_logger().info(
            f'[multiview_calib] serial={self.serial}  topic={args.image_topic}\n'
            f'  HD{args.library_hd}  multi-tag (id {REF_ID}=0.32 m REFERENCE, others=0.19 m aux)\n'
            f'  intrinsics: {"fisheye/equidistant" if self.fisheye else "pinhole"}  '
            f'{self.w}x{self.h}\n'
            f'  {f"collecting {args.duration:.0f}s then median-average" if args.duration > 0 else f"collecting {args.num_frames} good views"} '
            f'→ {self.dev_dir/"multiview.yml"}\n'
            f'  display={"on — viewer + [s]save [r]reset [q]quit" if args.display else "off (headless one-touch)"}')

    # ---- setup ----------------------------------------------------------------

    def _load_intrinsics(self):
        yml = self.dev_dir / 'intrinsic.yml'
        if not yml.exists():
            self.get_logger().error(
                f'{yml} not found. Run intrinsic_calib.launch.py for {self.serial} first.')
            raise SystemExit(2)
        fs = cv2.FileStorage(str(yml), cv2.FILE_STORAGE_READ)
        K = fs.getNode('camera_matrix').mat()
        D = fs.getNode('distortion_coefficients').mat()
        model = fs.getNode('distortion_model').string()
        w = int(fs.getNode('image_width').real() or 0)
        h = int(fs.getNode('image_height').real() or 0)
        fs.release()
        if K is None or D is None:
            self.get_logger().error(f'camera_matrix/distortion missing in {yml}')
            raise SystemExit(2)
        fisheye = (model or '').lower() in ('equidistant', 'fisheye')
        return (np.asarray(K, np.float64).reshape(3, 3),
                np.asarray(D, np.float64).reshape(1, -1),
                fisheye, (w, h))

    def _load_extrinsic(self):
        """LiDAR→RGB extrinsic (x_cam = R_e·x_lidar + t_e) or None if absent."""
        yml = self.dev_dir / 'extrinsic.yml'
        if not yml.exists():
            return None
        fs = cv2.FileStorage(str(yml), cv2.FILE_STORAGE_READ)
        R = fs.getNode('R').mat()
        t = fs.getNode('t').mat()
        fs.release()
        if R is None or t is None or R.size != 9 or t.size != 3:
            return None
        return (np.asarray(R, np.float64).reshape(3, 3),
                np.asarray(t, np.float64).reshape(3))

    # ---- per-frame ------------------------------------------------------------

    def _undistort(self, pts_px: np.ndarray) -> np.ndarray:
        """Fisheye/pinhole undistort 2D pixel corners → pinhole pixel coords (P=K)."""
        pts = pts_px.reshape(-1, 1, 2).astype(np.float64)
        if self.fisheye:
            D4 = self.D.reshape(-1)[:4].reshape(1, 4)
            und = cv2.fisheye.undistortPoints(pts, self.K, D4, P=self.K)
        else:
            und = cv2.undistortPoints(pts, self.K, self.D, P=self.K)
        return und.reshape(-1, 2)

    def _pose(self, corners_px, objp):
        """corners_px: (4,2) STag corners, objp: marker object points → (rvec, tvec, rmse_px)."""
        und = self._undistort(corners_px)
        ok, rvec, tvec = cv2.solvePnP(
            objp, und.astype(np.float64), self.K, None,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        proj, _ = cv2.projectPoints(objp, rvec, tvec, self.K, None)
        rmse = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - und) ** 2, axis=1))))
        return rvec, tvec, rmse

    def _on_trigger(self, _msg: Empty):
        # A host broadcasts a short burst (a few Empty msgs) for DDS-discovery
        # robustness; collapse the burst into ONE calibration: ignore while armed,
        # and for a cooldown after the last save.
        if self.armed:
            return
        now = time.monotonic()
        if self._last_save_time is not None and (now - self._last_save_time) < self.a.trigger_cooldown:
            return
        self._rearm(idle=False)
        mode = f'{self.a.duration:.0f}s' if self.a.duration > 0 else f'{self.a.num_frames} views'
        self.get_logger().info(f'[fleet] trigger received → collecting {mode} for {self.serial} ...')

    def _rearm(self, idle: bool):
        """Reset collection state. idle=True → back to waiting for the next trigger."""
        self.tags.clear()
        self._clouds.clear()
        self._collect_start = None
        self.last_save_dbg = -1
        self.saved = False
        self.armed = not idle

    def _cb(self, msg: Image):
        if self.saved or not self.armed:
            return
        self.frame_count += 1
        try:
            if self.bridge is not None:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                img = self._manual_decode(msg)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'image convert failed: {e}')
            return

        corners, ids, _ = stag.detectMarkers(img, self.a.library_hd)
        dbg = img.copy()

        if ids is not None and len(ids) > 0:
            stag.drawDetectedMarkers(dbg, corners, ids)
            for i, mid in enumerate(np.asarray(ids).reshape(-1)):
                mid = int(mid)
                size = _size_of(mid)
                cpx = np.asarray(corners[i], np.float64).reshape(4, 2)
                res = self._pose(cpx, _object_points(size))
                if res is None:
                    continue
                rvec, tvec, rmse = res
                if rmse <= self.a.reproj_thresh:
                    tag = self.tags.setdefault(mid, {'rv': [], 'tv': [], 'err': []})
                    # count-mode caps each tag at num_frames; duration-mode keeps every
                    # view in the time window so the median averages over all of them.
                    if self.a.duration > 0 or len(tag['rv']) < self.a.num_frames:
                        tag['rv'].append(rvec.reshape(3))
                        tag['tv'].append(tvec.reshape(3))
                        tag['err'].append(rmse)
                cv2.drawFrameAxes(dbg, self.K, None, rvec, tvec, size * 0.5, 3)
            self._annotate(dbg)

        self._hud(dbg)
        self._maybe_dump_debug(dbg)

        if self.a.display:
            self._show(dbg)          # viewer + [s]save [r]reset [q]quit (early/abort)

        # Auto-finish works in both modes (display just adds the viewer):
        #   duration>0 → median-average once the time window elapses (one-touch);
        #   duration<=0 → legacy count-based stop at --num-frames.
        if self.saved:
            return
        if self.a.duration > 0:
            if self._ref_count() >= 1 and self._collect_start is None:
                self._collect_start = time.monotonic()
                self.get_logger().info(
                    f'[collect] reference tag acquired — averaging over {self.a.duration:.0f}s ...')
            if (self._collect_start is not None
                    and (time.monotonic() - self._collect_start) >= self.a.duration
                    and self._ref_count() >= self.a.min_frames):
                self._finish(f'{self.a.duration:.0f}s window elapsed '
                             f'(median of {self._ref_count()} reference views)')
        elif self._ref_count() >= self.a.num_frames:
            self._finish('collected enough views (headless auto)')

    def _ref_id(self):
        """Reference tag id: tag 7 if seen, else the most-collected tag (or None)."""
        if REF_ID in self.tags:
            return REF_ID
        if not self.tags:
            return None
        return max(self.tags, key=lambda k: len(self.tags[k]['rv']))

    def _ref_count(self):
        rid = self._ref_id()
        return len(self.tags[rid]['rv']) if rid is not None else 0

    @staticmethod
    def _cloud_topic(image_topic: str) -> str:
        """Sibling point-cloud topic for an rgb image topic."""
        suffix = 'rgb/image_raw'
        if image_topic.endswith(suffix):
            return image_topic[:-len(suffix)] + 'point_cloud'
        ns = _namespace_from_topic(image_topic)
        return f'/{ns}/camera/point_cloud' if ns else '/camera/point_cloud'

    def _cloud_cb(self, msg: PointCloud2):
        if self.live_lidar_frame is None and msg.header.frame_id:
            self.live_lidar_frame = msg.header.frame_id
            if not self.live_lidar_frame.startswith(self.ns):
                self.get_logger().warn(
                    f'live point-cloud frame "{self.live_lidar_frame}" differs from the '
                    f'topic-derived namespace "{self.ns}"; the live frame will be saved.')
        if self.a.depth_refine and self._ext is not None and not self.saved:
            pts = _cloud_to_xyz(msg)
            if pts is not None:
                self._clouds.append(pts)

    def _resolve_frames(self):
        """Frames to persist: prefer the live driver's frame_id over the IP guess."""
        lf = self.live_lidar_frame
        if lf and lf.endswith('_lidar_frame'):
            ns = lf[:-len('_lidar_frame')]
            return ns, f'{ns}_camera_frame', lf
        return self.ns, self.camera_frame, self.lidar_frame

    def _refine_depth(self, R, t, marker_size):
        """Snap the marker depth to the LiDAR plane, keeping the STag rotation.

        Trust the STag marker normal n=R[:,2]; gather LiDAR points inside the
        marker footprint within ±depth_band of the 1st-pass marker plane (brought
        into the camera frame via the extrinsic); RANSAC the consensus plane
        offset along n; then move the marker centre along the camera viewing ray
        onto that plane (so the image projection — rotation & lateral — is kept,
        only depth changes). Returns (t_corrected, info_dict)."""
        if self._ext is None:
            return t, {'status': 'no extrinsic.yml'}
        if not self._clouds:
            return t, {'status': 'no LiDAR cloud received'}
        R_e, t_e = self._ext                      # x_cam = R_e·x_lidar + t_e
        pts_l = np.concatenate(list(self._clouds), axis=0)
        pts_c = (R_e @ pts_l.T).T + t_e           # LiDAR points in camera frame

        n = R[:, 2].astype(np.float64)
        n = n / np.linalg.norm(n)                 # marker normal (camera frame)
        c = t.astype(np.float64)                  # marker centre (camera frame)
        s = pts_c @ n                             # signed depth along the normal
        s0 = float(c @ n)
        band = float(self.a.depth_band)

        rel = pts_c - c
        lateral = rel - np.outer(rel @ n, n)      # in-plane offset from centre
        m = (np.abs(s - s0) < band) & \
            (np.einsum('ij,ij->i', lateral, lateral) < (0.75 * marker_size) ** 2)
        k = int(m.sum())
        if k < 30:
            return t, {'status': f'only {k} LiDAR pts in marker region'}

        d_pl, inl = _ransac_offset(s[m], tol=min(self.a.ransac_tol, band))
        if inl < max(15, int(0.3 * k)):
            return t, {'status': f'weak plane ({inl}/{k} inliers)'}

        u = c / np.linalg.norm(c)                 # camera viewing ray (preserves projection)
        denom = float(n @ u)
        if abs(denom) < 1e-6:
            return t, {'status': 'marker edge-on (degenerate)'}
        t_corr = (d_pl / denom) * u
        delta_m = float(np.linalg.norm(t_corr) - np.linalg.norm(c))
        max_delta = float(getattr(self.a, 'max_depth_delta', 0.0))
        if max_delta > 0.0 and abs(delta_m) > max_delta:
            return t, {
                'status': f'rejected large depth delta ({delta_m:+.3f} m > {max_delta:.3f} m)',
                'delta_m': delta_m,
                'plane_delta_m': float(d_pl - s0),
                'depth_before_m': float(np.linalg.norm(c)),
                'depth_after_m': float(np.linalg.norm(t_corr)),
                'inliers': int(inl), 'used': k}
        return t_corr, {
            'status': 'ok',
            'delta_m': delta_m,
            'plane_delta_m': float(d_pl - s0),
            'depth_before_m': float(np.linalg.norm(c)),
            'depth_after_m': float(np.linalg.norm(t_corr)),
            'inliers': int(inl), 'used': k}

    def _select_marker(self, ids):
        ids = np.asarray(ids).reshape(-1)
        if self.a.marker_id >= 0:
            hit = np.where(ids == self.a.marker_id)[0]
            return int(hit[0]) if len(hit) else None
        # default: the lowest id (deterministic reference choice)
        return int(np.argmin(ids))

    def _manual_decode(self, msg: Image) -> np.ndarray:
        """Minimal decode for rgb8/bgr8/mono8 when cv_bridge is unavailable."""
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding in ('rgb8', 'bgr8'):
            img = buf.reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img
        if msg.encoding == 'mono8':
            return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
        raise ValueError(f'unsupported encoding {msg.encoding}')

    # ---- visualization / persistence -----------------------------------------

    def _annotate(self, dbg):
        # one short line per tag: id (REF/aux), views collected, size
        y = 30
        for mid in sorted(self.tags):
            n = len(self.tags[mid]['rv'])
            tag = 'REF' if mid == REF_ID else 'aux'
            col = (0, 255, 0) if mid == REF_ID else (0, 200, 255)
            count = f'{n}' if self.a.duration > 0 else f'{n}/{self.a.num_frames}'
            cv2.putText(dbg, f'id={mid}[{tag}] {count}  {_size_of(mid):.2f}m',
                        (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
            y += 28

    def _hud(self, dbg):
        rid = self._ref_id()
        n = self._ref_count()
        if self.a.duration > 0:
            elapsed = (time.monotonic() - self._collect_start) if self._collect_start is not None else 0.0
            line = f'ref=id{rid} {n} views  [{elapsed:.0f}/{self.a.duration:.0f}s]  tags={sorted(self.tags)}'
        else:
            full = ' (full)' if n >= self.a.num_frames else ''
            line = f'ref=id{rid} {n}/{self.a.num_frames}{full}  tags={sorted(self.tags)}'
        if self.a.display:
            line += ('   [s]save [r]reset [q]quit' if n >= self.a.min_frames
                     else f'   (need >={self.a.min_frames} on ref)')
        cv2.putText(dbg, line, (10, dbg.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    def _maybe_dump_debug(self, dbg):
        # save a debug frame roughly every `num_frames`/6 reference captures, capped
        every = max(1, self.a.num_frames // 6)
        n = self._ref_count()
        if n > 0 and n != self.last_save_dbg and n % every == 0:
            self.last_save_dbg = n
            cv2.imwrite(str(self.out_dir / f'detect_{n:03d}.png'), dbg)

    def _show(self, dbg):
        try:
            disp = dbg
            if dbg.shape[1] > 1280:
                s = 1280.0 / dbg.shape[1]
                disp = cv2.resize(dbg, None, fx=s, fy=s)
            cv2.imshow('multiview_calib  [s]save  [r]reset  [q]quit', disp)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('s'):
                self._finish('user pressed save')
            elif k == ord('r'):
                self._reset_collection()
            elif k == ord('q'):
                self.get_logger().warn('user quit before saving')
                rclpy.shutdown()
        except cv2.error:
            self.a.display = False  # headless → silently switch to auto mode

    def _reset_collection(self):
        self.tags.clear()
        self._clouds.clear()
        self.last_save_dbg = -1
        self.get_logger().info('[reset] cleared all tags — re-aim the markers, then [s]')

    def _finish(self, reason: str):
        if self.saved:
            return
        rid = self._ref_id()
        if rid is None or len(self.tags[rid]['rv']) < self.a.min_frames:
            self.get_logger().warn(
                f'reference tag (id {REF_ID}) needs >= {self.a.min_frames} views; not saving yet')
            return

        # average + LiDAR-RANSAC depth-refine for every sufficiently-seen tag
        results = {}
        for mid, tag in self.tags.items():
            if len(tag['rv']) < self.a.min_frames:
                continue
            R = _mean_rotation(np.asarray(tag['rv']))
            t_mono = np.median(np.asarray(tag['tv']), axis=0)
            t = t_mono.copy()
            size = _size_of(mid)
            refine = None
            if self.a.depth_refine:
                t_ref, refine = self._refine_depth(R, t_mono, size)
                if refine.get('status') == 'ok':
                    t = t_ref
            results[mid] = dict(R=R, t=t, t_mono=t_mono, size=size,
                                rmse=float(np.mean(tag['err'])), n=len(tag['rv']), refine=refine)
            ref = refine
            ok = ref and ref.get('status') == 'ok'
            self.get_logger().info(
                f'[tag {mid}{" REF" if mid == rid else ""}] {results[mid]["n"]} views  '
                f'reproj {results[mid]["rmse"]:.2f}px  ' +
                (f'depth {ref["depth_before_m"]:.3f}→{ref["depth_after_m"]:.3f} m (Δ{ref["delta_m"]:+.3f})'
                 if ok else f'depth-refine: {ref["status"] if ref else "off"}'))

        self._save(rid, results)
        self.saved = True
        if self.a.display:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        if self.a.wait_trigger:
            self._last_save_time = time.monotonic()
            self.get_logger().info(
                f'[multiview_calib] saved ({reason}); re-armed — waiting for next {self.a.trigger_topic}.')
            self._rearm(idle=True)
        else:
            self.get_logger().info(f'[multiview_calib] saved ({reason}); shutting down.')
            rclpy.shutdown()

    def _save(self, rid, results):
        ns, camera_frame, lidar_frame = self._resolve_frames()
        ref = results[rid]
        depth_ok = bool(ref['refine'] and ref['refine'].get('status') == 'ok')
        yml = self.dev_dir / 'multiview.yml'
        fs = cv2.FileStorage(str(yml), cv2.FILE_STORAGE_WRITE)
        fs.write('camera_id', self.serial)
        fs.write('image_topic', self.a.image_topic)
        fs.write('namespace', ns)
        fs.write('camera_frame', camera_frame)
        fs.write('lidar_frame', lidar_frame)
        fs.write('reference_frame', 'stag_marker')
        fs.write('library_hd', int(self.a.library_hd))
        fs.write('marker_id', int(rid))
        fs.write('marker_size', float(ref['size']))
        fs.write('num_frames', int(ref['n']))
        fs.write('reproj_rmse_px', float(ref['rmse']))
        fs.write('depth_refined', 1 if depth_ok else 0)
        if depth_ok:
            fs.write('depth_delta_m', float(ref['refine']['delta_m']))
        # canonical: reference tag pose in rgb cam = stag_marker ↔ rgb (x_cam = R·x_marker + t)
        fs.write('R', np.ascontiguousarray(ref['R'], np.float64))
        fs.write('t', np.ascontiguousarray(ref['t'].reshape(3, 1), np.float64))
        # every tag (incl. reference), pose in the rgb cam frame, for TF sharing
        ids = sorted(results)
        fs.write('tag_count', int(len(ids)))
        for k, mid in enumerate(ids):
            r = results[mid]
            ddv = r['refine']['delta_m'] if (r['refine'] and r['refine'].get('status') == 'ok') else 0.0
            fs.write(f'tag_{k}_id', int(mid))
            fs.write(f'tag_{k}_size', float(r['size']))
            fs.write(f'tag_{k}_R', np.ascontiguousarray(r['R'], np.float64))
            fs.write(f'tag_{k}_t', np.ascontiguousarray(r['t'].reshape(3, 1), np.float64))
            fs.write(f'tag_{k}_depth_delta_m', float(ddv))
        fs.release()

        # human-readable summary — the per-tag monocular→RANSAC depth comparison
        summary = self.out_dir / 'summary.txt'
        with open(summary, 'w') as f:
            f.write(f'multiview calibration — {self.serial}\n')
            f.write(f'timestamp  : {datetime.datetime.now().isoformat(timespec="seconds")}\n')
            f.write(f'reference  : stag_marker = tag id {rid} ({ref["size"]:.2f} m)\n')
            f.write(f'tags       : {ids}\n\n')
            f.write('per-tag depth-refine (monocular range → LiDAR-RANSAC range):\n')
            for mid in ids:
                r = results[mid]
                rf = r['refine']
                tag = 'REF' if mid == rid else 'aux'
                if rf and rf.get('status') == 'ok':
                    f.write(f'  id {mid:>3} [{tag}] {r["size"]:.2f}m :  '
                            f'{rf["depth_before_m"]:.3f} → {rf["depth_after_m"]:.3f} m  '
                            f'(Δ{rf["delta_m"]:+.3f} m, {rf["inliers"]}/{rf["used"]} pts)\n')
                else:
                    f.write(f'  id {mid:>3} [{tag}] {r["size"]:.2f}m :  '
                            f'{np.linalg.norm(r["t_mono"]):.3f} m  '
                            f'(refine: {rf["status"] if rf else "off"})\n')
        t_mc = (-ref['R'].T @ ref['t']).reshape(3)
        self.get_logger().info(
            f'  → {yml}  ({len(ids)} tags, ref id {rid})\n  → {summary}\n'
            f'  cam@marker=({t_mc[0]:+.3f},{t_mc[1]:+.3f},{t_mc[2]:+.3f}) m')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera-id', required=True, help='Camera USB serial (output folder key)')
    ap.add_argument('--calib-dir', required=True, help='calib_output directory')
    ap.add_argument('--image-topic', required=True, help='RGB image topic to subscribe')
    ap.add_argument('--library-hd', type=int, default=21,
                    help='STag HD library [11,13,15,17,19,21,23]')
    ap.add_argument('--marker-size', type=float, default=0.32,
                    help='Marker side length in metres (outer black border)')
    ap.add_argument('--marker-id', type=int, default=-1,
                    help='Reference marker id; -1 = lowest visible id')
    ap.add_argument('--num-frames', type=int, default=10,
                    help='Default (count-based): collect this many good reference views, then '
                         'median-average + LiDAR depth-refine + save. ~10 is plenty (STag pose is stable).')
    ap.add_argument('--duration', type=float, default=0.0,
                    help='Optional time-based mode: if >0, collect for this many seconds from the first '
                         'reference view instead of a fixed count. Default 0 = count-based (--num-frames).')
    ap.add_argument('--min-frames', type=int, default=5,
                    help='Minimum good views on the reference tag required to save')
    ap.add_argument('--trigger-cooldown', type=float, default=5.0,
                    help='Fleet mode: ignore further triggers for this many seconds after a save, so one '
                         'host broadcast burst (a few Empty msgs) counts as a single calibration.')
    ap.add_argument('--reproj-thresh', type=float, default=3.0,
                    help='Max per-view reprojection RMSE (px) to accept a view')
    ap.add_argument('--display', default='false',
                    help='Show live detection window (true/false). Default false (headless one-touch); '
                         'true just adds a viewer — it still auto-saves after --duration.')
    ap.add_argument('--wait-trigger', default='false',
                    help='Fleet mode: idle until an std_msgs/Empty lands on --trigger-topic, then run one '
                         'headless calib window and re-arm. Lets a host calibrate every camera with one '
                         'broadcast — no SSH/accounts. Default false (calibrate immediately).')
    ap.add_argument('--trigger-topic', default='/fleet/calibrate',
                    help='Topic the host broadcasts std_msgs/Empty on to start fleet calibration.')
    ap.add_argument('--depth-refine', default='true',
                    help='Snap marker depth onto the LiDAR plane via RANSAC (true/false)')
    ap.add_argument('--depth-band', type=float, default=0.50,
                    help='± depth band (m) around the 1st-pass marker plane for the LiDAR RANSAC')
    ap.add_argument('--ransac-tol', type=float, default=0.08,
                    help='RANSAC inlier tolerance (m): LiDAR points within this distance of the '
                         'consensus marker plane are inliers (capped at --depth-band)')
    ap.add_argument('--max-depth-delta', type=float, default=0.40,
                    help='Reject LiDAR depth refinement if the final camera-ray range correction exceeds '
                         'this many meters. Set <=0 to disable.')
    args, ros_args = ap.parse_known_args()
    args.display = str(args.display).strip().lower() in ('true', '1', 'yes')
    args.depth_refine = str(args.depth_refine).strip().lower() in ('true', '1', 'yes')
    args.wait_trigger = str(args.wait_trigger).strip().lower() in ('true', '1', 'yes')

    rclpy.init(args=ros_args or None)
    try:
        node = MultiviewCalibNode(args)
    except SystemExit:
        if rclpy.ok():
            rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Headless direct mode auto-saves on exit if enough views; with a display,
        # saving is explicit ([s]); trigger/idle mode never saves on Ctrl-C.
        if (not node.saved and not node.a.display and not node.a.wait_trigger
                and node._ref_count() >= args.min_frames):
            node._finish('shutdown auto-save (headless)')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
