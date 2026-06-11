#!/usr/bin/env python3
"""
Publish the shared STag reference frame and anchor every calibrated camera
under it as static TF, so RViz / tf2 can show all cameras in one common frame.

For each camera serial that has  calib_output/{serial}/multiview.yml  we read

        x_cam = R · x_marker + t          (marker → rgb camera optical frame)

and broadcast the camera under the reference frame `stag_marker`.

  • camera pose in the marker frame:   R_mc = Rᵀ,  t_mc = −Rᵀ·t
  • if extrinsic.yml is also present, we compose it so the whole driver subtree
    (the live `{ns}_lidar_frame` published by the point cloud) hangs under the
    marker without conflicting with the lidar→camera TF that camera.launch.py
    already publishes:

        stag_marker → {ns}_lidar_frame      (this node)
        {ns}_lidar_frame → {ns}_camera_frame (extrinsic_tf_node, on the camera)

    extrinsic.yml stores  x_cam = R_e · x_lidar + t_e (lidar pose in camera), so
        R_ml = R_mc · R_e ,  t_ml = R_mc · t_e + t_mc

  • if extrinsic.yml is missing, we anchor stag_marker → {ns}_camera_frame
    directly (the point cloud then can't be placed — a warning is logged).

REJECTION (step 5): a camera whose multiview.yml is missing, unreadable, or has
a malformed R|t is skipped with a warning and simply does not appear under the
reference frame.

Usage (via multiview.launch.py, automatic) or standalone:
    python3 multiview_tf_node.py --calib-dir /path/to/calib_output
    python3 multiview_tf_node.py --calib-dir ... --camera-id N00A5060D   # one camera
    python3 multiview_tf_node.py --calib-dir ... --camera-id N00A5060D --frame-prefix cam_52
"""

import argparse
import os
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

REFERENCE_FRAME = 'stag_marker'


def _rotmat_to_quat(R):
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


def _read_Rt(yml_path: Path):
    """Return (R 3x3, t 3) from an OpenCV .yml, or None if missing/malformed."""
    fs = cv2.FileStorage(str(yml_path), cv2.FILE_STORAGE_READ)
    R = fs.getNode('R').mat()
    t = fs.getNode('t').mat()
    fs.release()
    if R is None or t is None or R.size != 9 or t.size != 3:
        return None
    return np.asarray(R, np.float64).reshape(3, 3), np.asarray(t, np.float64).reshape(3)


class MultiviewTfNode(Node):

    def __init__(self, calib_dir: str, camera_filter: str, frame_prefix: str, reload_sec: float = 1.0):
        super().__init__('multiview_tf_node')
        self.calib_dir = Path(os.path.expanduser(calib_dir))
        self.frame_prefix = frame_prefix.strip().strip('/')
        self.camera_filter = camera_filter
        self.bc = StaticTransformBroadcaster(self)
        self._mtimes = {}
        self._publish_all(camera_filter)
        # Re-publish when a calibration file changes so a fresh multiview_calib
        # (online, e.g. a fleet /fleet/calibrate) updates /tf_static without a
        # bringup restart. Static TF re-sends replace the latched value.
        if reload_sec > 0:
            self.create_timer(reload_sec, self._poll)

    def _device_dirs(self, camera_filter: str):
        if camera_filter:
            d = self.calib_dir / camera_filter
            return [d] if d.is_dir() else []
        return sorted(p for p in self.calib_dir.iterdir() if p.is_dir())

    def _watch_mtimes(self):
        """mtime of every calib file that feeds the published TF (multiview.yml +
        extrinsic.yml, per watched device). New/removed files change the dict too."""
        out = {}
        for dev in self._device_dirs(self.camera_filter):
            for name in ('multiview.yml', 'extrinsic.yml'):
                f = dev / name
                try:
                    out[str(f)] = f.stat().st_mtime
                except OSError:
                    pass
        return out

    def _poll(self):
        cur = self._watch_mtimes()
        if cur != self._mtimes:
            self.get_logger().info('[multiview_tf] calibration changed → re-publishing TF.')
            self._publish_all(self.camera_filter)

    def _publish_all(self, camera_filter: str):
        transforms = []
        accepted, rejected = [], []
        for dev in self._device_dirs(camera_filter):
            serial = dev.name
            mv = dev / 'multiview.yml'
            if not mv.exists():
                continue  # not a multiview-calibrated camera; silently skip
            rt = _read_Rt(mv)
            if rt is None:
                rejected.append((serial, 'R|t missing/malformed'))
                continue

            R, t = rt
            R_mc = R.T                  # camera axes in marker frame
            t_mc = -R.T @ t             # camera origin in marker frame

            fs = cv2.FileStorage(str(mv), cv2.FILE_STORAGE_READ)
            camera_frame = fs.getNode('camera_frame').string() or f'{serial}_camera_frame'
            lidar_frame = fs.getNode('lidar_frame').string() or f'{serial}_lidar_frame'
            fs.release()
            if self.frame_prefix:
                camera_frame = f'{self.frame_prefix}_camera_frame'
                lidar_frame = f'{self.frame_prefix}_lidar_frame'

            ext = _read_Rt(dev / 'extrinsic.yml') if (dev / 'extrinsic.yml').exists() else None
            if ext is not None:
                # extrinsic.yml stores x_cam = R_e·x_lidar + t_e (lidar pose in camera),
                # so lidar pose in marker = (camera in marker) ∘ (lidar in camera):
                R_e, t_e = ext
                R_ml = R_mc @ R_e
                t_ml = R_mc @ t_e + t_mc
                transforms.append(self._tf(lidar_frame, R_ml, t_ml))
                accepted.append((serial, lidar_frame, t_ml, True))
            else:
                transforms.append(self._tf(camera_frame, R_mc, t_mc))
                accepted.append((serial, camera_frame, t_mc, False))

            # Auxiliary tags — publish each as stag_marker → tag_{id}_frame:
            #   tag in marker = (cam in marker) ∘ (tag in cam) = (R_mc·Ri, R_mc·ti + t_mc)
            fs = cv2.FileStorage(str(mv), cv2.FILE_STORAGE_READ)
            ref_id = int(fs.getNode('marker_id').real() or -1)
            ntag = int(fs.getNode('tag_count').real() or 0)
            for k in range(ntag):
                tid = int(fs.getNode(f'tag_{k}_id').real())
                if tid == ref_id:
                    continue   # the reference tag IS the origin
                Ri = fs.getNode(f'tag_{k}_R').mat()
                ti = fs.getNode(f'tag_{k}_t').mat()
                if Ri is None or ti is None or Ri.size != 9 or ti.size != 3:
                    continue
                Ri = np.asarray(Ri, np.float64).reshape(3, 3)
                ti = np.asarray(ti, np.float64).reshape(3)
                transforms.append(self._tf(f'tag_{tid}_frame', R_mc @ Ri, R_mc @ ti + t_mc))
                accepted.append((serial, f'tag_{tid}_frame', R_mc @ ti + t_mc, True))
            fs.release()

        if transforms:
            self.bc.sendTransform(transforms)
        self._mtimes = self._watch_mtimes()   # snapshot for change detection

        for serial, child, t_, via_lidar in accepted:
            note = '' if via_lidar else '  (no extrinsic.yml → point cloud not anchored)'
            self.get_logger().info(
                f'[multiview_tf] {REFERENCE_FRAME} → {child}  '
                f'pos=({t_[0]:+.3f},{t_[1]:+.3f},{t_[2]:+.3f}) m{note}')
        for serial, why in rejected:
            self.get_logger().warn(f'[multiview_tf] REJECTED {serial}: {why}')
        if not accepted:
            self.get_logger().warn(
                f'[multiview_tf] no cameras anchored under {REFERENCE_FRAME}. '
                f'Run multiview_calib.launch.py on each camera first.')
        else:
            self.get_logger().info(
                f'[multiview_tf] reference frame "{REFERENCE_FRAME}" '
                f'with {len(accepted)} camera(s) anchored.')

    def _tf(self, child_frame, R, t):
        q = _rotmat_to_quat(R)
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = REFERENCE_FRAME
        ts.child_frame_id = child_frame
        ts.transform.translation.x = float(t[0])
        ts.transform.translation.y = float(t[1])
        ts.transform.translation.z = float(t[2])
        ts.transform.rotation.x = float(q[0])
        ts.transform.rotation.y = float(q[1])
        ts.transform.rotation.z = float(q[2])
        ts.transform.rotation.w = float(q[3])
        return ts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--calib-dir', required=True, help='calib_output directory')
    ap.add_argument('--camera-id', default='',
                    help='Only anchor this serial; empty = every multiview-calibrated camera')
    ap.add_argument('--frame-prefix', default='',
                    help='Override saved frame names, e.g. cam_52 -> cam_52_lidar_frame')
    ap.add_argument('--reload-sec', type=float, default=1.0,
                    help='Poll the calib files every N s and re-publish TF when they change '
                         '(online update after a recalibration). <=0 disables.')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args or None)
    node = MultiviewTfNode(args.calib_dir, args.camera_id, args.frame_prefix, args.reload_sec)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
