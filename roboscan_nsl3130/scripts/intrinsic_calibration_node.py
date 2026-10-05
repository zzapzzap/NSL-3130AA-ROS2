#!/usr/bin/env python3
"""
Intrinsic calibration node for NSL-3130AA RGB camera.

Launches cameracalibrator for image collection and GUI feedback.
When [SAVE] is pressed, extracts images from the archive and runs
cv2.fisheye.calibrate directly — cameracalibrator's own D=[0,0,0,0]
result is intentionally ignored.

Workflow:
  1. Collect samples — cover all corners and tilt the board (skew bar)
  2. [CALIBRATE] → unlocks the SAVE button (cameracalibrator's result ignored)
  3. [SAVE]  →  this script calibrates from the images and writes YML
"""

import argparse
import os
import subprocess
import tarfile
import time
from pathlib import Path

import cv2
import numpy as np


# ── image extraction ─────────────────────────────────────────────────────────

def _extract_images(archive_path: str) -> list:
    images = []
    with tarfile.open(archive_path, 'r:gz') as tar:
        for m in sorted(tar.getmembers(), key=lambda x: x.name):
            if not m.name.lower().endswith(('.png', '.jpg', '.jpeg')):
                continue
            f = tar.extractfile(m)
            if f is None:
                continue
            data = np.frombuffer(f.read(), dtype=np.uint8)
            img = cv2.imdecode(data, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                images.append(img)
    return images


# ── corner detection ──────────────────────────────────────────────────────────

def _detect_corners(images: list, board_wh: tuple, square_m: float):
    """Return (objpoints, imgpoints, (W, H))."""
    W_b, H_b = board_wh

    # fisheye requires Nx1x3
    objp = np.zeros((W_b * H_b, 1, 3), np.float64)
    objp[:, 0, :2] = np.array(
        [[c * square_m, r * square_m] for r in range(H_b) for c in range(W_b)],
        np.float64)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    cb_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH |
                cv2.CALIB_CB_NORMALIZE_IMAGE |
                cv2.CALIB_CB_FAST_CHECK)

    objpoints, imgpoints = [], []
    img_size = None
    found = 0

    for img in images:
        ret, corners = cv2.findChessboardCorners(img, (W_b, H_b), cb_flags)
        if not ret:
            continue
        corners = cv2.cornerSubPix(img, corners, (7, 7), (-1, -1), criteria)
        objpoints.append(objp.copy())
        imgpoints.append(corners.reshape(-1, 1, 2).astype(np.float64))
        if img_size is None:
            img_size = (img.shape[1], img.shape[0])
        found += 1

    print(f'[intrinsic] Corners detected: {found}/{len(images)} frames', flush=True)
    return objpoints, imgpoints, img_size


# ── fisheye calibration ───────────────────────────────────────────────────────

def _fisheye_calibrate(objpoints, imgpoints, img_size):
    """
    FOV sweep: cameracalibrator starts from f=W/π (full-hemisphere) which traps
    the optimizer at D=[0,0,0,0].  Sweep realistic FOV values and pick the best.
    Stops early once a very good result (RMS<1.5) is found.
    """
    W, H = img_size
    flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
             cv2.fisheye.CALIB_FIX_SKEW |
             cv2.fisheye.CALIB_USE_INTRINSIC_GUESS)

    best_K, best_D, best_rms = None, None, float('inf')

    for fov_deg in [120, 140, 100, 160, 90, 110, 130, 80, 150, 170]:
        f0 = (W / 2.0) / np.deg2rad(fov_deg / 2.0)
        K0 = np.array([[f0, 0.0, W / 2.0],
                       [0.0, f0, H / 2.0],
                       [0.0, 0.0, 1.0]], np.float64)
        D0 = np.zeros((4, 1), np.float64)

        try:
            rms, K_out, D_out, _, _ = cv2.fisheye.calibrate(
                objpoints, imgpoints, img_size, K0, D0, flags=flags)
        except cv2.error as e:
            print(f'[intrinsic]   FOV {fov_deg:3d}° (f={f0:.0f}) → {e}', flush=True)
            continue

        d_mag = float(np.linalg.norm(D_out))
        print(f'[intrinsic]   FOV {fov_deg:3d}° (f={f0:.0f}) → RMS={rms:.4f}  D={D_out.T}',
              flush=True)

        if d_mag < 1e-6 or rms > 5.0:
            continue
        if rms < best_rms:
            best_rms, best_K, best_D = rms, K_out.copy(), D_out.copy()

        if best_rms < 1.5:
            break  # good enough — stop early to avoid GUI freeze

    return best_K, best_D, best_rms


def _refine_with_outlier_rejection(objpoints, imgpoints, img_size, K, D):
    """Remove high-reprojection-error frames and re-calibrate once."""
    flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
             cv2.fisheye.CALIB_FIX_SKEW |
             cv2.fisheye.CALIB_USE_INTRINSIC_GUESS)

    per_frame = []
    for obj, img_pts in zip(objpoints, imgpoints):
        proj, _ = cv2.fisheye.projectPoints(
            obj, np.zeros(3), np.zeros(3), K, D)
        err = float(np.sqrt(np.mean((proj.reshape(-1, 2) - img_pts.reshape(-1, 2)) ** 2)))
        per_frame.append(err)

    mean_err = np.mean(per_frame)
    std_err  = np.std(per_frame)
    threshold = mean_err + 2.0 * std_err

    kept_obj = [o for o, e in zip(objpoints, per_frame) if e <= threshold]
    kept_img = [i for i, e in zip(imgpoints, per_frame) if e <= threshold]
    removed  = len(objpoints) - len(kept_obj)

    if removed == 0 or len(kept_obj) < 8:
        return K, D, None

    K2, D2 = K.copy(), D.copy()
    try:
        rms2, K2, D2, _, _ = cv2.fisheye.calibrate(
            kept_obj, kept_img, img_size, K2, D2, flags=flags)
        print(f'[intrinsic] Refined: removed {removed} outlier frames → RMS={rms2:.4f}',
              flush=True)
        return K2, D2, rms2
    except cv2.error:
        return K, D, None


# ── save ─────────────────────────────────────────────────────────────────────

def _save_intrinsic_yml(path: Path, camera_id: str, W: int, H: int,
                        K: np.ndarray, D: np.ndarray):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    fs.write('camera_id',               camera_id)
    fs.write('image_width',             W)
    fs.write('image_height',            H)
    fs.write('distortion_model',        'equidistant')
    fs.write('camera_matrix',           K)
    fs.write('distortion_coefficients', D.reshape(1, -1))
    fs.release()
    print(f'[intrinsic] Saved → {path}', flush=True)


# ── archive processing ────────────────────────────────────────────────────────

def _wait_for_stable_archive(path: str) -> bool:
    for _ in range(30):
        try:
            s1 = os.path.getsize(path)
        except OSError:
            time.sleep(0.3)
            continue
        time.sleep(0.3)
        try:
            s2 = os.path.getsize(path)
        except OSError:
            continue
        if s1 > 0 and s1 == s2:
            return True
    return False


def _process_archive(archive_path: str, camera_id: str, output_path: Path,
                     board_wh: tuple, square_m: float):
    print(f'\n[intrinsic] Processing archive: {archive_path}', flush=True)

    try:
        images = _extract_images(archive_path)
    except Exception as e:
        print(f'[intrinsic] Failed to read archive: {e}', flush=True)
        return

    if not images:
        print('[intrinsic] No images in archive. Press [CALIBRATE] then [SAVE].', flush=True)
        return

    print(f'[intrinsic] {len(images)} images extracted', flush=True)

    objpoints, imgpoints, img_size = _detect_corners(images, board_wh, square_m)

    if len(objpoints) < 8:
        print(f'[intrinsic] Only {len(objpoints)} usable frames — need ≥ 8.', flush=True)
        return

    K, D, rms = _fisheye_calibrate(objpoints, imgpoints, img_size)

    if K is None:
        print('[intrinsic] Calibration failed for all initial focal length guesses.', flush=True)
        print('           Try collecting more tilted/corner samples and SAVE again.', flush=True)
        return

    K2, D2, rms2 = _refine_with_outlier_rejection(objpoints, imgpoints, img_size, K, D)
    if rms2 is not None and rms2 < rms:
        K, D, rms = K2, D2, rms2

    print(f'[intrinsic] Final RMS={rms:.4f}  K={K[0,0]:.1f},{K[1,1]:.1f}  D={D.T}', flush=True)
    _save_intrinsic_yml(output_path, camera_id, img_size[0], img_size[1], K, D)


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera-id',   default='nsl')
    ap.add_argument('--size',        default='8x13',
                    help='Chessboard inner corners WxH')
    ap.add_argument('--square',      type=float, default=0.04,
                    help='Square side length in metres')
    ap.add_argument('--image-topic', default='/camera/rgb/image_raw')
    ap.add_argument('--output-dir',  default=os.path.join(os.getcwd(), 'calib_output'))
    args = ap.parse_args()

    w_str, h_str = args.size.lower().split('x')
    board_wh = (int(w_str), int(h_str))

    output_dir  = Path(os.path.expanduser(args.output_dir)) / args.camera_id
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / 'intrinsic.yml'

    archive_path = '/tmp/calibrationdata.tar.gz'

    if os.path.exists(archive_path):
        os.remove(archive_path)

    cmd = [
        'ros2', 'run', 'camera_calibration', 'cameracalibrator',
        '--size', args.size,
        '--square', str(args.square),
        '--ros-args', '-r', f'image:={args.image_topic}',
    ]
    print()
    print('  ┌─ GUI 사용법 ──────────────────────────────────────────────┐')
    print('  │  1. 보드를 FOV 전체 + 코너 + 틸트(skew 바 채우기)         │')
    print('  │  2. [CALIBRATE]  (슬라이더 위치 무관, GUI 결과 무시됨)     │')
    print('  │  3. [SAVE]  →  이 스크립트가 이미지로 직접 calibration     │')
    print('  └──────────────────────────────────────────────────────────┘', flush=True)
    print()
    print(f'[intrinsic] Launching: {" ".join(cmd)}', flush=True)
    print(f'[intrinsic] Monitoring {archive_path} ...', flush=True)

    # PYTHONUNBUFFERED=1 ensures cameracalibrator prints lines in real-time
    env = {**os.environ, 'PYTHONUNBUFFERED': '1'}
    proc = subprocess.Popen(cmd, env=env)
    last_mtime = None

    try:
        while True:
            time.sleep(1.0)
            if not os.path.exists(archive_path):
                continue
            mtime = os.path.getmtime(archive_path)
            if mtime == last_mtime:
                continue
            last_mtime = mtime
            print('[intrinsic] Archive detected — waiting for write to finish ...', flush=True)
            if not _wait_for_stable_archive(archive_path):
                print('[intrinsic] Archive did not stabilise, skipping.', flush=True)
                continue
            _process_archive(archive_path, args.camera_id, output_path,
                             board_wh, args.square)
            print('[intrinsic] SAVE again to refine, or Ctrl+C to quit.', flush=True)
    except KeyboardInterrupt:
        print('\n[intrinsic] Interrupted.')
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    print('[intrinsic] Done.')


if __name__ == '__main__':
    main()
