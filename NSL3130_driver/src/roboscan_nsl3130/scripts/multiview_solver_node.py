#!/usr/bin/env python3
"""Host-side deterministic multi-tag chain solver.

Each edge, after a /fleet/calibrate, publishes its per-tag observations as JSON on a
latched `/cam_NN/tag_observations` topic (see multiview_calib_node._publish_observations).
This node collects every camera's snapshot and builds one shared `stag_marker` frame by a
priority greedy chain: tag id 0 is preferred as the origin, cameras with more visible tags are
placed first, and each new camera is attached through the lowest-id already-placed shared tag.
There is no global bundle adjustment, triangulation, or depth gate in the solve path.

Output: per camera a multiview.yml in the EXACT format multiview_calib_node._save writes
(x_cam = R·x_marker + t, marker = the global anchor tag), so multiview_tf_node and the pose
stack consume it unchanged. Writeback to the edges is a separate step (multiview_put_*).

    ros2 run ros_humanpose multiview_solver_node.py --out-dir /tmp/mv_bundle
    ros2 run ros_humanpose multiview_solver_node.py --trigger-topic /fleet/calibrate
    python3 multiview_solver_node.py --selftest        # offline math check, no ROS traffic
"""

import argparse
import hashlib
import json
import os
import re
import sys
import time
from pathlib import Path

import numpy as np

try:
    from scipy.spatial.transform import Rotation as Rot
except ImportError:  # pragma: no cover
    print('[mv_solver] ERROR: scipy required (scipy.spatial.transform).', file=sys.stderr)
    raise

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


# ───────────────────────────── SE3 helpers ──────────────────────────────────
# A pose is a (R 3x3, t 3) tuple meaning  x_parent = R · x_child + t.

def se3_compose(A, B):
    Ra, ta = A
    Rb, tb = B
    return Ra @ Rb, Ra @ tb + ta


def se3_inv(A):
    R, t = A
    Rt = R.T
    return Rt, -Rt @ t


def se3_identity():
    return np.eye(3), np.zeros(3)


UP = np.array([0.0, 0.0, 1.0])


def _unit(v, fallback=None):
    v = np.asarray(v, dtype=np.float64).reshape(3)
    n = float(np.linalg.norm(v))
    if n > 1e-12:
        return v / n
    return np.asarray(fallback if fallback is not None else UP, dtype=np.float64).reshape(3)


def _rot_between(a, b):
    """Smallest rotation matrix that maps unit vector a onto unit vector b."""
    a = _unit(a)
    b = _unit(b)
    v = np.cross(a, b)
    c = float(np.clip(a @ b, -1.0, 1.0))
    s = float(np.linalg.norm(v))
    if s < 1e-12:
        if c > 0.0:
            return np.eye(3)
        axis = np.cross(a, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, np.array([0.0, 1.0, 0.0]))
        return Rot.from_rotvec(np.pi * _unit(axis)).as_matrix()
    return Rot.from_rotvec(_unit(v) * np.arctan2(s, c)).as_matrix()


def _angle_deg_between(a, b):
    a = _unit(a)
    b = _unit(b)
    return float(np.degrees(np.arccos(np.clip(float(a @ b), -1.0, 1.0))))


def _snap_up(R):
    """Force a tag's normal (R[:,2]) to world +Z (tags lie flat), keeping the in-plane heading."""
    x = R[:, 0].copy(); x[2] = 0.0
    x = x / np.linalg.norm(x) if np.linalg.norm(x) > 1e-6 else np.array([1.0, 0.0, 0.0])
    z = UP.copy(); y = np.cross(z, x)
    return np.column_stack([x, y, z])


# ─────────────────────────── observation parsing ────────────────────────────

def _poses_from_corners(corners8, size, K, D, fisheye):
    """Both IPPE_SQUARE solutions (tag-in-camera R, t) from the 4 mean corner pixels. The HOST
    re-derives the pose from the corners so it owns the planar-flip disambiguation: solvePnPGeneric
    returns the two planar poses (primary + the tilt-flipped alternate). [] if cv2/PnP unavailable."""
    if cv2 is None or corners8 is None or len(corners8) != 8 or K is None:
        return []
    K = np.asarray(K, float).reshape(3, 3)
    D = np.asarray(D if D is not None else [0, 0, 0, 0], float).reshape(1, -1)
    pts = np.asarray(corners8, float).reshape(4, 1, 2)
    try:
        if fisheye:
            und = cv2.fisheye.undistortPoints(pts, K, D[:, :4].reshape(1, 4), P=K)
        else:
            und = cv2.undistortPoints(pts, K, D, P=K)
        h = float(size) / 2.0
        objp = np.array([[-h, h, 0.], [h, h, 0.], [h, -h, 0.], [-h, -h, 0.]], float)
        nsol, rvecs, tvecs, _err = cv2.solvePnPGeneric(
            objp, und.reshape(-1, 2).astype(float), K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error:
        return []
    return [(cv2.Rodrigues(rvecs[k])[0], np.asarray(tvecs[k], float).reshape(3))
            for k in range(int(nsol))]


def parse_observation(text):
    """JSON string from /cam_NN/tag_observations → normalized dict, or None if bad.

    The edge-published pose is kept as the primary pose because it may already contain the
    edge-side LiDAR depth refinement. The host only re-derives the IPPE alternate from corners so
    the greedy chain can choose a deterministic flip without throwing away that depth."""
    try:
        d = json.loads(text)
    except (ValueError, TypeError):
        return None
    if d.get('schema') != 'nsl_tag_obs_v1' or not d.get('tags'):
        return None
    K, D, fish = d.get('K'), d.get('D'), bool(d.get('fisheye', False))
    tags = {}
    for tg in d['tags']:
        tid = int(tg['id'])
        size = float(tg.get('size', 0.0))
        depth = tg.get('depth') or {}
        R = np.asarray(tg['R'], float).reshape(3, 3)
        t = np.asarray(tg['t'], float).reshape(3)
        t_mono = np.asarray(tg.get('t_mono', t), float).reshape(3)
        poses = _poses_from_corners(tg.get('corners_px'), size, K, D, fish)
        R_alt, t_alt = poses[1] if len(poses) > 1 else (None, None)
        range_lidar, range_conf = None, 1.0
        if depth.get('status') == 'ok':
            range_lidar = float(depth['after_m'])    # LiDAR sliding+RANSAC refined range (the fine-correction)
            used = float(depth.get('used') or 0.0)
            if 'confidence' in depth:
                range_conf = max(0.05, min(1.0, float(depth.get('confidence') or 0.0)))
            else:
                range_conf = (max(0.05, min(1.0, float(depth.get('inliers') or 0.0) / used))
                              if used > 0 else 0.5)    # RANSAC plane inlier ratio = depth confidence
            # Keep alternate flip candidates on the same LiDAR-refined range scale as the primary.
            if t_alt is not None:
                n_alt = float(np.linalg.norm(t_alt))
                if n_alt > 1e-9:
                    t_alt = t_alt / n_alt * range_lidar
        tags[tid] = {
            'R': R, 't': t, 'R_alt': R_alt, 't_alt': t_alt,
            't_mono': t_mono,
            'size': size,
            'rmse_px': float(tg.get('rmse_px', 0.0)),
            'views': int(tg.get('views', 0)),
            'range_lidar': range_lidar, 'range_conf': range_conf,
            'corners_px': tg.get('corners_px'),    # kept only to recover the IPPE alternate
        }
    d['tags'] = tags
    return d


# ───────────────────────────── the solver ───────────────────────────────────

class BundleSolver:
    """Deterministic chain solver over camera + tag SE3 poses."""

    def __init__(self, ref_id=0, w_up=2.0, depth_vote_range=0.6,
                 depth_vote_step=0.01, depth_vote_perp=0.25, depth_vote_half=0.05,
                 fit_z_up=True, max_z_up_correction=35.0):
        self.ref_id = int(ref_id)
        self.w_up = float(w_up)
        self.depth_vote_range = float(depth_vote_range)
        self.depth_vote_step = float(depth_vote_step)
        self.depth_vote_perp = float(depth_vote_perp)
        self.depth_vote_half = float(depth_vote_half)
        self.fit_z_up = bool(fit_z_up)
        self.max_z_up_correction = float(max_z_up_correction)

    def _priority(self, tid):
        """Tag priority for anchor + link order: id 0 first, then ascending id."""
        return int(tid)

    def _anchor_id(self, observations, forced):
        seen = set()
        for ob in observations:
            seen.update(ob['tags'])
        if not seen:
            return None
        if forced is not None and forced >= 0 and int(forced) in seen:
            return int(forced)
        if self.ref_id in seen:
            return self.ref_id
        return min(seen, key=self._priority)

    @staticmethod
    def _pose_options(tg):
        opts = [(False, (np.asarray(tg['R'], float), np.asarray(tg['t'], float)))]
        if tg.get('R_alt') is not None and tg.get('t_alt') is not None:
            opts.append((True, (np.asarray(tg['R_alt'], float), np.asarray(tg['t_alt'], float))))
        return opts

    def _depth_vote_rigid(self, cloud_cam, tag_ts_cam, d_link,
                          perp=None, half_depth=None, srange=None, step=None):
        """RIGID depth refine in the camera frame. The cloud is the metric truth (fixed); the RGB
        tag cluster has good bearing but weak depth, so slide the WHOLE cluster along the linking
        tag's bearing d_link by s and pick the s that maximises cloud inliers summed over a
        50×50×10 cm box (perp half-width `perp`, half-thickness `half_depth`) at each tag. A
        second pass refines s by the inlier centroid (plane fit). Returns (s, inliers)."""
        if cloud_cam is None or len(cloud_cam) < 50 or not tag_ts_cam:
            return 0.0, 0
        perp = self.depth_vote_perp if perp is None else float(perp)
        half_depth = self.depth_vote_half if half_depth is None else float(half_depth)
        srange = self.depth_vote_range if srange is None else float(srange)
        step = self.depth_vote_step if step is None else float(step)
        d = np.asarray(d_link, float); d = d / (np.linalg.norm(d) + 1e-12)
        seed = np.array([1.0, 0.0, 0.0]) if abs(d[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        e1 = seed - d * float(seed @ d)
        e1 = e1 / (np.linalg.norm(e1) + 1e-12)
        e2 = np.cross(d, e1)
        C = np.asarray(cloud_cam, float)
        def inliers_at(s):
            mask = np.zeros(len(C), bool)
            for c in tag_ts_cam:
                q = C - (np.asarray(c, float) + s * d)
                along = q @ d
                side1 = q @ e1
                side2 = q @ e2
                mask |= (np.abs(along) < half_depth) & (np.abs(side1) < perp) & (np.abs(side2) < perp)
            return mask
        best = (-1, 0.0)
        for s in np.arange(-srange, srange + 0.5 * step, step):
            n = int(inliers_at(s).sum())
            if n > best[0]:
                best = (n, float(s))
        s = best[1]
        m = inliers_at(s)                              # plane-fit refine: align tag centroid to the
        if m.sum() >= 10:                              # cloud-surface median along the bearing
            s = float(np.median(C[m] @ d)
                      - np.median([np.asarray(c, float) @ d for c in tag_ts_cam]))
        return s, best[0]

    def _fit_camera_z_up(self, R_w_cam, chosen):
        """Roll/pitch fit after translation is fixed.

        Main/link marker depth chooses the epipolar translation first. Then this
        finds the smallest camera-rotation correction that makes the average
        observed marker normal face world +Z. Yaw is preserved as much as possible.
        """
        if self.w_up <= 0.0 or not self.fit_z_up or not chosen:
            return R_w_cam, 0.0, 0.0, False
        normals = []
        for R_c_tag, _t in chosen.values():
            n = _unit(R_w_cam @ np.asarray(R_c_tag, dtype=np.float64).reshape(3, 3)[:, 2])
            if n[2] < 0.0:
                n = -n
            normals.append(n)
        if not normals:
            return R_w_cam, 0.0, 0.0, False
        n_avg = _unit(np.mean(np.asarray(normals), axis=0))
        max_before = max(_angle_deg_between(n, UP) for n in normals)
        corr = _rot_between(n_avg, UP)
        corr_deg = float(np.degrees(np.linalg.norm(Rot.from_matrix(corr).as_rotvec())))
        if corr_deg > self.max_z_up_correction:
            return R_w_cam, corr_deg, max_before, True
        return corr @ R_w_cam, corr_deg, max_before, False

    def _greedy_chain(self, observations, anchor, clouds=None):
        """Priority greedy chain (NO global BA). anchor tag = origin. Process cameras by tag count
        (most first); place each via its lowest-id ALREADY-PLACED (linking) tag by composing the
        per-tag corner poses. The linking tag's epipolar line fixes translation first via the
        edge sliding-window depth; then camera rotation is minimally fit so the observed tag
        normals face world +Z. Add new tags snapped horizontal. Returns (cam_pose, tag_pose, info)."""
        clouds = clouds or {}
        tagset = [set(ob['tags']) for ob in observations]
        tp = {anchor: se3_identity()}
        cp, info = {}, {}
        remaining = set(range(len(observations)))

        while remaining:
            cand = [(ci, sorted(tagset[ci] & set(tp))) for ci in remaining if tagset[ci] & set(tp)]
            if not cand:
                break                                       # rest share no placed tag (isolated)
            ci, shared = sorted(
                cand,
                key=lambda x: (-len(tagset[x[0]]), self._priority(min(x[1], key=self._priority)),
                               observations[x[0]].get('namespace', ''),
                               observations[x[0]].get('camera_id', ''))
            )[0]
            remaining.discard(ci)
            link = min(shared, key=self._priority)          # lowest-priority-id shared tag = the link
            ob = observations[ci]

            # 1) LINK flip — deterministic up-normal disambiguation when tags are horizontal.
            def cam_from(link_pose):
                return se3_compose(tp[link], se3_inv(link_pose))
            if self.w_up > 0.0:
                flip_l, link_pose = max(
                    self._pose_options(ob['tags'][link]),
                    key=lambda opt: sum(se3_compose(cam_from(opt[1]),
                                                    self._pose_options(ob['tags'][t])[0][1])[0][2, 2]
                                        for t in ob['tags'])
                )
            else:
                flip_l, link_pose = self._pose_options(ob['tags'][link])[0]
            cam0 = cam_from(link_pose)

            # 2) per-tag flip — choose each tag's planar solution with the most up-pointing world
            #    normal (the link keeps its step-1 choice). A wrong flip would mis-place the tag.
            chosen, flips = {}, []
            for tid in ob['tags']:
                if tid == link:
                    fl, pose = flip_l, link_pose
                elif self.w_up > 0.0:
                    fl, pose = max(self._pose_options(ob['tags'][tid]),
                                   key=lambda opt: se3_compose(cam0, opt[1])[0][2, 2])
                else:
                    fl, pose = self._pose_options(ob['tags'][tid])[0]
                chosen[tid] = pose
                if fl:
                    flips.append(tid)

            # 3) rigid depth shift. If no host cloud is supplied, use the link tag's edge-side
            # LiDAR-refined range as the single shift and apply it to every tag in this camera.
            base_ts = {}
            for tid, tg in ob['tags'].items():
                base_ts[tid] = np.asarray(tg.get('t_mono', chosen[tid][1]), float).reshape(3).copy()
            d_link = base_ts[link] / (np.linalg.norm(base_ts[link]) + 1e-12)
            s, ninl, depth_source = 0.0, 0, 'rgb'
            link_rgb_m = float(np.linalg.norm(base_ts[link]))
            link_lidar_m = ob['tags'][link].get('range_lidar')

            if ci in clouds:
                s, ninl = self._depth_vote_rigid(clouds[ci], list(base_ts.values()), d_link)
                depth_source = 'cloud_vote'
            elif ob['tags'][link].get('range_lidar') is not None:
                s = float(ob['tags'][link]['range_lidar']) - float(np.linalg.norm(base_ts[link]))
                depth_source = 'link_lidar'
            ts = {tid: t + s * d_link for tid, t in base_ts.items()}

            # 4) Place the camera via the depth-refined link translation, then fit roll/pitch
            #    so all visible marker normals face +Z. Recompute camera translation after the
            #    rotation fit so the link marker position stays exactly fixed.
            cam_raw = se3_compose(tp[link], se3_inv((chosen[link][0], ts[link])))
            R_cam, normal_fit_deg, normal_max_deg, normal_fit_skipped = self._fit_camera_z_up(
                cam_raw[0], chosen)
            if normal_fit_skipped:
                cam = cam_raw
            else:
                cam = (R_cam, tp[link][1] - R_cam @ ts[link])
            cp[ci] = cam
            for tid in ob['tags']:
                if tid in tp:
                    continue
                Rw, tw = se3_compose(cam, (chosen[tid][0], ts[tid]))
                if self.w_up > 0.0:
                    Rw = _snap_up(Rw)
                tp[tid] = (Rw, tw)
            info[ob['camera_id']] = {
                'namespace': ob.get('namespace', ''),
                'link': link,
                'shared': shared,
                'added': sorted(t for t in ob['tags'] if t not in shared),
                'flips': flips,
                'depth_source': depth_source,
                'depth_shift_m': round(float(s), 3),
                'link_rgb_m': round(link_rgb_m, 3),
                'link_lidar_m': None if link_lidar_m is None else round(float(link_lidar_m), 3),
                'cloud_inliers': int(ninl),
                'normal_fit_deg': round(float(normal_fit_deg), 3),
                'normal_max_before_deg': round(float(normal_max_deg), 3),
                'normal_fit_skipped': bool(normal_fit_skipped),
            }
        return cp, tp, info, sorted(set(range(len(observations))) - remaining), set(tp)

    def solve(self, observations, anchor_id=None, clouds=None):
        """Priority greedy CHAIN (no global BA / no triangulation / no corner-BA). The tag tree is
        built by composing per-tag corner poses in priority order, with a rigid cloud depth-vote per
        camera (clouds in the CAMERA frame, optional). See _greedy_chain / _depth_vote_rigid."""
        anchor = self._anchor_id(observations, anchor_id)
        if anchor is None:
            return {'ok': False, 'error': 'no tags in any observation'}
        observations = [{**ob, 'tags': {tid: dict(tg, _tid=tid) for tid, tg in ob['tags'].items()}}
                        for ob in observations]
        cp, tp, chain_info, free_cams, free_tags = self._greedy_chain(observations, anchor, clouds)
        if not cp:
            return {'ok': False, 'error': 'no camera connected to the anchor tag'}
        isolated = [observations[i]['camera_id'] for i in range(len(observations)) if i not in free_cams]
        bridges = {}
        for ob in observations:
            for tid in ob['tags']:
                bridges.setdefault(tid, []).append(ob['camera_id'])
        return {
            'ok': True, 'anchor': anchor, 'cam_pose': cp, 'tag_pose': tp,
            'free_cams': free_cams, 'free_tags': sorted(set(free_tags) - {anchor}),
            'isolated_cams': isolated, 'isolated_tags': [],
            'edges': 0, 'rms_before': 0.0, 'rms_after': 0.0,
            'flips': [], 'edge_resid': [], 'bridges': bridges,
            'lidar_rejected': [], 'lidar_softened': [],
            'n_alt': 0, 'pnp': {}, 'dropped': [], 'tri': {}, 'chain': chain_info,
        }


# ─────────────────────────── yml writer (same format) ───────────────────────

def write_multiview_yml(path, ob, anchor, T_w_cam, tag_pose):
    """Write ONE camera's multiview.yml in multiview_calib_node._save's exact format.

    x_cam = R·x_marker + t with marker = the global anchor tag. Per-tag poses are the
    chain-derived tag-in-this-camera transforms from the shared tag tree."""
    if cv2 is None:
        raise RuntimeError('cv2 required to write OpenCV FileStorage yml')
    T_ci_w = se3_inv(T_w_cam)
    R_anchor, t_anchor = se3_compose(T_ci_w, tag_pose[anchor])   # anchor (marker) in this camera
    anchor_obs = ob['tags'].get(anchor, {})

    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    fs.write('camera_id', ob['camera_id'])
    fs.write('namespace', ob.get('namespace', ''))
    fs.write('camera_frame', ob.get('camera_frame', ''))
    fs.write('lidar_frame', ob.get('lidar_frame', ''))
    fs.write('reference_frame', 'stag_marker')
    fs.write('library_hd', int(ob.get('library_hd', 21)))
    fs.write('marker_id', int(anchor))
    fs.write('marker_size', float(anchor_obs.get('size', ob.get('marker_size', 0.0))))
    fs.write('num_frames', int(anchor_obs.get('views', 0)))
    fs.write('reproj_rmse_px', float(anchor_obs.get('rmse_px', 0.0)))
    fs.write('depth_refined', 1 if anchor_obs.get('range_lidar') is not None else 0)
    fs.write('bundle_solved', 1)
    fs.write('chain_solved', 1)
    fs.write('R', np.ascontiguousarray(R_anchor, np.float64))
    fs.write('t', np.ascontiguousarray(t_anchor.reshape(3, 1), np.float64))
    # every tag this camera saw, in this camera's frame, from the chain solution
    ids = sorted(tid for tid in ob['tags'] if tid in tag_pose)
    fs.write('tag_count', int(len(ids)))
    for k, tid in enumerate(ids):
        R_k, t_k = se3_compose(T_ci_w, tag_pose[tid])
        fs.write(f'tag_{k}_id', int(tid))
        fs.write(f'tag_{k}_size', float(ob['tags'][tid].get('size', 0.0)))
        fs.write(f'tag_{k}_R', np.ascontiguousarray(R_k, np.float64))
        fs.write(f'tag_{k}_t', np.ascontiguousarray(t_k.reshape(3, 1), np.float64))
        fs.write(f'tag_{k}_depth_delta_m', 0.0)
    fs.release()


# ───────────────────────────── ROS node ─────────────────────────────────────

def run_node(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy)
    from std_msgs.msg import Empty, String

    latched = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    topic_re = re.compile(r'^/(cam_\d+)/tag_observations$')

    class BundleNode(Node):
        def __init__(self):
            super().__init__('multiview_solver_node')
            self.obs = {}                       # camera_id -> observation dict
            self._subs = {}
            self._put_clients = {}              # service name -> PutWeight client (writeback)
            self.solver = BundleSolver(
                ref_id=args.ref_id,
                w_up=args.w_up,
                depth_vote_range=args.depth_vote_range,
                depth_vote_step=args.depth_vote_step,
                depth_vote_perp=args.depth_vote_perp,
                depth_vote_half=args.depth_vote_half,
                fit_z_up=args.fit_z_up,
                max_z_up_correction=args.max_z_up_correction,
            )
            self.roi_flush_pub = None
            if args.roi_debug_flush_topic:
                self.roi_flush_pub = self.create_publisher(Empty, args.roi_debug_flush_topic, 10)
            self.create_timer(1.0, self._discover)
            if args.trigger_topic:
                self.create_subscription(Empty, args.trigger_topic, self._on_trigger, 10)
                self.get_logger().info(
                    f'[mv_solver] listening; will solve {args.settle:.0f}s after '
                    f'{args.trigger_topic} (or every {args.period:.0f}s if --period>0).')
            if args.period > 0:
                self.create_timer(args.period, lambda: self._arm(0.0))
            self._solve_at = None        # monotonic deadline; the MAIN LOOP solves (not a callback)

        def _discover(self):
            names = dict(self.get_topic_names_and_types())
            for name in names:
                m = topic_re.match(name)
                if m and name not in self._subs:
                    ns = m.group(1)
                    self._subs[name] = self.create_subscription(
                        String, name, (lambda ns: lambda msg: self._on_obs(ns, msg))(ns), latched)
                    self.get_logger().info(f'[mv_solver] subscribed {name}')

        def _on_obs(self, ns, msg):
            ob = parse_observation(msg.data)
            if ob is None:
                self.get_logger().warn(f'[mv_solver] {ns}: unparsable/empty observation')
                return
            self.obs[ob['camera_id']] = ob
            self.get_logger().info(
                f'[mv_solver] {ns} ({ob["camera_id"]}): {len(ob["tags"])} tags '
                f'{sorted(ob["tags"])}')

        def _arm(self, delay):
            # Arm a solve `delay` s from now. The MAIN LOOP performs it — NOT a timer/sub callback —
            # because the writeback calls rclpy.spin_until_future_complete, which deadlocks if run
            # inside a single-threaded-executor callback (the service response never gets processed).
            self._solve_at = time.monotonic() + max(0.0, delay)

        def _on_trigger(self, _msg):
            # solve a settle window after the broadcast so every edge's fresh snapshot lands
            self._arm(args.settle)
            self.get_logger().info(f'[mv_solver] trigger → solving in {args.settle:.0f}s ...')

        def solve_now(self, why):
            obs = list(self.obs.values())
            if len(obs) < 1:
                self.get_logger().warn('[mv_solver] no observations collected yet.')
                return
            res = self.solver.solve(obs, args.anchor_id)
            if not res['ok']:
                self.get_logger().error(f'[mv_solver] solve failed: {res["error"]}')
                return
            self._report(obs, res, why)

        def _report(self, obs, res, why):
            self.get_logger().info(
                f'[mv_solver] ({why}) deterministic-chain anchor=id{res["anchor"]} (origin)  '
                f'cams={len(res["free_cams"])}  tags={len(res["free_tags"]) + 1}')
            # bridge graph — which tag id is seen by which cameras (★ = the anchor/origin). A tag
            # seen by ≥2 cameras BRIDGES them; this is the correspondence that ties the views.
            for tid in sorted(res.get('bridges', {})):
                cams = sorted(set(res['bridges'][tid]))
                star = ' ★anchor' if tid == res['anchor'] else (' (bridge)' if len(cams) >= 2 else '')
                self.get_logger().info(f'    id{tid}: {", ".join(cams)}{star}')
            for step, (cam_id, inf) in enumerate(res.get('chain', {}).items(), start=1):
                lidar = 'none' if inf["link_lidar_m"] is None else f'{inf["link_lidar_m"]:.3f}'
                src = inf.get('namespace') or cam_id
                self.get_logger().info(
                    f'    chain#{step} src={src} serial={cam_id}: '
                    f'link=id{inf["link"]} shared={inf["shared"]} '
                    f'added={inf["added"]} depth={inf["depth_source"]} '
                    f'link_range={inf["link_rgb_m"]:.3f}->{lidar}m '
                    f'shift={inf["depth_shift_m"]:+.3f}m inliers={inf["cloud_inliers"]} '
                    f'normal_fit={inf["normal_fit_deg"]:.3f}deg'
                    f'{"/SKIP" if inf["normal_fit_skipped"] else ""} '
                    f'flips={inf["flips"]}')
            if res['isolated_cams']:
                self.get_logger().warn(
                    f'[mv_solver] ISOLATED cameras (no shared tag → NOT anchored): '
                    f'{res["isolated_cams"]}')
            if res['isolated_tags']:
                self.get_logger().warn(f'[mv_solver] isolated tags: {res["isolated_tags"]}')
            pushed = 0
            for ci in res['free_cams']:
                ob = obs[ci]
                T_w_cam = res['cam_pose'][ci]
                pos = T_w_cam[1]
                out = os.path.join(args.out_dir, ob['camera_id'], 'multiview.yml')
                write_multiview_yml(out, ob, res['anchor'], T_w_cam, res['tag_pose'])
                self.get_logger().info(
                    f'  {ob["namespace"]} ({ob["camera_id"]}) @ '
                    f'({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) m → {out}')
                if args.writeback and ob.get('namespace'):
                    if self._push(ob['namespace'], out):
                        pushed += 1
            if args.writeback:
                self.get_logger().info(
                    f'[mv_solver] pushed {pushed}/{len(res["free_cams"])} multiview.yml to edges '
                    f'→ each multiview_tf_node re-publishes /tf_static (live).')
            else:
                self.get_logger().info(
                    f'[mv_solver] wrote {len(res["free_cams"])} multiview.yml under {args.out_dir} '
                    f'(dry-run; re-run with --writeback to push to the edges).')
            self._flush_roi_debug()

        def _flush_roi_debug(self):
            if self.roi_flush_pub is None:
                return
            delay = max(0.0, float(args.roi_debug_flush_delay))
            if delay > 0.0:
                time.sleep(delay)
            count = max(1, int(args.roi_debug_flush_count))
            for _ in range(count):
                self.roi_flush_pub.publish(Empty())
                time.sleep(0.05)
            self.get_logger().info(
                f'[mv_solver] ROI debug flush → {args.roi_debug_flush_topic} '
                f'({count} pulse{"s" if count != 1 else ""})')

        def _push(self, ns, yml_path):
            """Chunked + sha256 push of one solved multiview.yml to /<ns>/multiview/put (WRITEBACK).
            Reuses the PutWeight contract; the edge installs it atomically and its multiview_tf_node
            re-publishes /tf_static on the next mtime poll. PutWeight is imported lazily so dry-run
            and --selftest never need ros_humanpose."""
            try:
                from ros_humanpose.srv import PutWeight
            except ImportError as exc:
                self.get_logger().error(f'[mv_solver] writeback needs ros_humanpose PutWeight: {exc}')
                return False
            svc = f'/{ns}/multiview/put'
            cli = self._put_clients.get(svc)
            if cli is None:
                cli = self.create_client(PutWeight, svc)
                self._put_clients[svc] = cli
            if not cli.wait_for_service(timeout_sec=args.writeback_wait):
                self.get_logger().error(
                    f'[mv_solver] writeback: {svc} unavailable (waited {args.writeback_wait:.0f}s)')
                return False
            total = os.path.getsize(yml_path)
            h = hashlib.sha256()
            with open(yml_path, 'rb') as f:
                for b in iter(lambda: f.read(1024 * 1024), b''):
                    h.update(b)
            sha = h.hexdigest()
            offset = 0
            with open(yml_path, 'rb') as f:
                while True:
                    data = f.read(2 * 1024 * 1024)
                    req = PutWeight.Request()
                    req.name = 'multiview.yml'
                    req.offset = offset
                    req.total_size = total
                    req.sha256 = sha
                    req.data = list(data)
                    fut = cli.call_async(req)
                    rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
                    resp = fut.result()
                    if resp is None or not resp.success:
                        self.get_logger().error(
                            f'[mv_solver] writeback {svc}: '
                            f'{resp.message if resp else "service call timed out"}')
                        return False
                    offset += len(data)
                    if offset >= total:
                        if not resp.complete:
                            self.get_logger().error(
                                f'[mv_solver] writeback {svc}: server did not confirm completion')
                            return False
                        break
            self.get_logger().info(f'[mv_solver] writeback → {svc} ({total} B, sha verified)')
            return True

    rclpy.init()
    node = BundleNode()
    if args.once:
        node._arm(args.settle)           # solve once after the settle window, then exit
    try:
        # Manual spin loop: spin_once services subs/timers; when the armed deadline elapses we run
        # the solve+writeback HERE (main thread, outside any callback) so spin_until_future_complete
        # in the writeback can actually process the service response.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node._solve_at is not None and time.monotonic() >= node._solve_at:
                node._solve_at = None
                node.solve_now('once' if args.once else 'trigger')
                if args.once:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


# ───────────────────────────── self test ────────────────────────────────────

def _selftest():
    """Offline checks for the deterministic greedy chain."""
    rng = np.random.default_rng(1)
    sizes = {0: 0.32, 1: 0.19, 2: 0.19, 3: 0.19}

    def rand_pose(scale=1.0):
        return Rot.from_rotvec(rng.normal(0, 0.4, 3)).as_matrix(), rng.normal(0, scale, 3)

    def make_obs(cams_gt, tags_gt, vis):
        obs = []
        for ci, tids in vis.items():
            tags = {}
            for tid in tids:
                R, t = se3_compose(se3_inv(cams_gt[ci]), tags_gt[tid])
                tags[tid] = {'R': R, 't': t, 't_mono': t, 'size': sizes[tid], 'rmse_px': 0.5,
                             'views': 10, 'range_lidar': float(np.linalg.norm(t)),
                             'range_conf': 1.0}
            obs.append({'camera_id': f'CAM{ci}', 'namespace': f'cam_{ci}',
                        'camera_frame': f'cam_{ci}_camera_frame', 'lidar_frame': f'cam_{ci}_lidar_frame',
                        'library_hd': 21, 'marker_size': sizes[tids[0]], 'tags': tags})
        return obs

    def pose_errs(res, cams_gt):
        mt = mr = 0.0
        for ci in res['free_cams']:
            Rg, tg = cams_gt[ci]
            Re, te = res['cam_pose'][ci]
            mt = max(mt, float(np.linalg.norm(te - tg)))
            mr = max(mr, float(np.linalg.norm(Rot.from_matrix(Rg.T @ Re).as_rotvec())))
        return mt, mr

    ok = True

    # (A) bridge chain
    cams_gt = {0: rand_pose(1.2), 1: rand_pose(1.2), 2: rand_pose(1.2)}
    tags_gt = {0: se3_identity(), 1: rand_pose(0.6), 2: rand_pose(0.6), 3: rand_pose(0.6)}
    obs = make_obs(cams_gt, tags_gt, {0: [0, 1], 1: [1, 2], 2: [2, 3]})
    rA = BundleSolver(w_up=0.0).solve(obs, anchor_id=0)
    mtA, mrA = pose_errs(rA, cams_gt)
    bridgeA = rA['ok'] and not rA['isolated_cams'] and len(rA['free_cams']) == 3
    print(f'[selftest A/chain] anchor=id{rA["anchor"]} cams={len(rA["free_cams"])}/3 '
          f'isolated={rA["isolated_cams"]} maxdt={mtA*1000:.3f}mm maxdR={np.degrees(mrA):.4f}deg')
    okA = bridgeA and mtA < 1e-9 and mrA < 1e-9
    print('  ' + ('PASS - 3 cams connected by priority tag chain' if okA else 'FAIL'))
    ok = ok and okA

    # (B) priority: most tags first, lower-id shared tag as link.
    tags_gt = {0: se3_identity(), 1: (np.eye(3), np.array([0.5, 0.0, 0.0])),
               2: (np.eye(3), np.array([1.0, 0.0, 0.0])), 3: (np.eye(3), np.array([1.5, 0.0, 0.0]))}
    cams_gt = {0: (np.eye(3), np.array([0.0, -1.0, 0.0])),
               1: (np.eye(3), np.array([0.0, -2.0, 0.0])),
               2: (np.eye(3), np.array([0.0, -3.0, 0.0]))}
    obs = make_obs(cams_gt, tags_gt, {0: [0], 1: [0, 1, 2], 2: [1, 3]})
    rB = BundleSolver(w_up=0.0).solve(obs, anchor_id=0)
    orderB = list(rB.get('chain', {}))
    okB = rB['ok'] and orderB[:2] == ['CAM1', 'CAM2'] and rB['chain']['CAM2']['link'] == 1
    print(f'[selftest B/priority] chain_order={orderB} CAM2_link=id{rB["chain"].get("CAM2", {}).get("link")}')
    print('  ' + ('PASS - camera/tag priority is deterministic' if okB else 'FAIL'))
    ok = ok and okB

    # (C) link LiDAR range moves every tag rigidly along the link bearing.
    obs = [{
        'camera_id': 'CAMR', 'namespace': 'cam_r', 'tags': {
            0: {'R': np.eye(3), 't': np.array([0.0, 0.0, 1.4]), 't_mono': np.array([0.0, 0.0, 1.0]),
                'size': sizes[0], 'rmse_px': 0.5, 'views': 10, 'range_lidar': 1.4, 'range_conf': 1.0},
            1: {'R': np.eye(3), 't': np.array([1.0, 0.0, 1.0]), 't_mono': np.array([1.0, 0.0, 1.0]),
                'size': sizes[1], 'rmse_px': 0.5, 'views': 10, 'range_lidar': None, 'range_conf': 1.0},
        }
    }]
    rC = BundleSolver(w_up=0.0).solve(obs, anchor_id=0)
    tag1 = rC['tag_pose'][1][1]
    shiftC = rC['chain']['CAMR']['depth_shift_m']
    okC = rC['ok'] and abs(shiftC - 0.4) < 1e-9 and np.linalg.norm(tag1 - np.array([1.0, 0.0, 0.0])) < 1e-9
    print(f'[selftest C/rigid-depth] shift={shiftC:+.3f} tag1={tag1.round(3).tolist()}  '
          + ('PASS' if okC else 'FAIL'))
    ok = ok and okC

    # (D) host-side cloud vote finds the rigid shift with max 50x50x10 cm support.
    solver = BundleSolver(depth_vote_range=0.4, depth_vote_step=0.01)
    d = np.array([0.0, 0.0, 1.0])
    tags = [np.array([0.0, 0.0, 1.0]), np.array([0.2, 0.0, 1.0])]
    true_s = 0.18
    cloud = []
    for t in tags:
        center = t + true_s * d
        cloud.append(center + rng.normal([0.0, 0.0, 0.0], [0.04, 0.04, 0.015], (80, 3)))
    cloud.append(rng.uniform([-1, -1, 0.4], [1, 1, 1.8], (120, 3)))
    cloud = np.concatenate(cloud)
    sD, inlD = solver._depth_vote_rigid(cloud, tags, d)
    okD = abs(sD - true_s) < 0.035 and inlD > 80
    print(f'[selftest D/cloud-vote] shift={sD:+.3f} expected={true_s:+.3f} inliers={inlD}  '
          + ('PASS' if okD else 'FAIL'))
    ok = ok and okD

    # (E) parser keeps edge-published LiDAR-refined depth as primary.
    payload = {'schema': 'nsl_tag_obs_v1', 'camera_id': 'CAMJSON', 'tags': [{
        'id': 0, 'size': sizes[0],
        'R': np.eye(3).reshape(-1).tolist(),
        't': [0.0, 0.0, 2.0],
        't_mono': [0.0, 0.0, 1.0],
        'rmse_px': 0.5, 'views': 10,
        'depth': {'status': 'ok', 'after_m': 2.0, 'inliers': 20, 'used': 20},
    }]}
    parsed = parse_observation(json.dumps(payload))
    okE = parsed is not None and abs(np.linalg.norm(parsed['tags'][0]['t']) - 2.0) < 1e-9
    print('[selftest E/parse-depth] ' + ('PASS' if okE else 'FAIL'))
    ok = ok and okE

    # (F) z-up fit keeps the link translation fixed while reducing marker-normal tilt.
    R_link = Rot.from_euler('x', np.radians(6.0)).as_matrix()
    R_other = Rot.from_euler('y', np.radians(-8.0)).as_matrix()
    obs = [{
        'camera_id': 'CAMZ', 'namespace': 'cam_z', 'tags': {
            0: {'R': R_link, 't': np.array([0.0, 0.0, 1.0]), 't_mono': np.array([0.0, 0.0, 1.0]),
                'size': sizes[0], 'rmse_px': 0.5, 'views': 10, 'range_lidar': 1.0, 'range_conf': 1.0},
            1: {'R': R_other, 't': np.array([0.35, 0.0, 1.0]), 't_mono': np.array([0.35, 0.0, 1.0]),
                'size': sizes[1], 'rmse_px': 0.5, 'views': 10, 'range_lidar': None, 'range_conf': 1.0},
        }
    }]
    rF = BundleSolver(w_up=2.0).solve(obs, anchor_id=0)
    link_pos = rF['cam_pose'][0][0] @ obs[0]['tags'][0]['t_mono'] + rF['cam_pose'][0][1]
    tag1_z = rF['tag_pose'][1][0][:, 2]
    fit_deg = rF['chain']['CAMZ']['normal_fit_deg']
    okF = (rF['ok']
           and np.linalg.norm(link_pos) < 1e-9
           and _angle_deg_between(tag1_z, UP) < 1e-9
           and fit_deg > 0.01)
    print(f'[selftest F/z-up-fit] normal_fit={fit_deg:.3f}deg '
          f'link={link_pos.round(6).tolist()}  ' + ('PASS' if okF else 'FAIL'))
    ok = ok and okF

    print('[selftest] ALL PASS' if ok else '[selftest] FAILED')
    return 0 if ok else 1


# ─────────────────────────────── main ───────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', default=os.path.expanduser('~/mv_bundle_out'),
                    help='Where solved per-camera multiview.yml files are written (then pushed to edges).')
    ap.add_argument('--anchor-id', type=int, default=-1,
                    help='Force a tag id as the stag_marker origin. -1 = auto: prefer --ref-id if '
                         'visible, else the lowest visible id.')
    ap.add_argument('--ref-id', type=int, default=0,
                    help='Preferred anchor tag when --anchor-id is not forced. Default id 0.')
    ap.add_argument('--trigger-topic', default='/fleet/calibrate',
                    help='Solve a settle window after an Empty lands here. Empty string disables.')
    ap.add_argument('--settle', type=float, default=6.0,
                    help='Seconds to wait after the trigger for every edge snapshot to arrive.')
    ap.add_argument('--period', type=float, default=0.0,
                    help='If >0, also solve every N seconds from whatever has been collected.')
    ap.add_argument('--once', action='store_true',
                    help='Settle once, solve, then exit (one-touch from a CLI / mtf).')
    ap.add_argument('--writeback', action='store_true',
                    help='After solving, push each camera its solved multiview.yml to '
                         '/cam_NN/multiview/put (the edge installs it; TF auto-updates). '
                         'Default off = dry-run (write under --out-dir only).')
    ap.add_argument('--writeback-wait', type=float, default=8.0,
                    help='Seconds to wait for each edge /cam_NN/multiview/put service.')
    ap.add_argument('--w-up', type=float, default=2.0,
                    help='If >0, use horizontal-tag up-normal scoring to choose deterministic IPPE flips.')
    ap.add_argument('--fit-z-up', action=argparse.BooleanOptionalAction, default=True,
                    help='After link epipolar depth fixes translation, minimally rotate each camera so '
                         'visible marker normals face world +Z. Default true.')
    ap.add_argument('--max-z-up-correction', type=float, default=35.0,
                    help='Skip z-up camera rotation fitting if the required correction exceeds this many degrees.')
    ap.add_argument('--depth-vote-range', type=float, default=0.60,
                    help='Host cloud-vote search half-range (m) along the linking tag ray.')
    ap.add_argument('--depth-vote-step', type=float, default=0.01,
                    help='Host cloud-vote scan step (m) along the linking tag ray.')
    ap.add_argument('--depth-vote-perp', type=float, default=0.25,
                    help='Host cloud-vote perpendicular half-width (m), 0.25 = 50 cm box.')
    ap.add_argument('--depth-vote-half', type=float, default=0.05,
                    help='Host cloud-vote depth half-thickness (m), 0.05 = 10 cm box.')
    ap.add_argument('--roi-debug-flush-topic', default='/fleet/roi_debug_flush',
                    help='Publish std_msgs/Empty here after solving/writeback so edge ROI snapshots are '
                         'shown only after matching is complete. Empty disables.')
    ap.add_argument('--roi-debug-flush-delay', type=float, default=1.0,
                    help='Seconds to wait after solver output/writeback before flushing ROI debug markers.')
    ap.add_argument('--roi-debug-flush-count', type=int, default=3,
                    help='Number of ROI debug flush pulses to publish.')
    # Compatibility no-ops: accepted so old aliases / solver_args do not fail, but the
    # deterministic chain path no longer runs BA, triangulation, or LiDAR-prior gating.
    ap.add_argument('--w-rot', type=float, default=2.0, help='compatibility no-op')
    ap.add_argument('--rot-angle-pow', type=float, default=1.0, help='compatibility no-op')
    ap.add_argument('--w-lat', type=float, default=4.0, help='compatibility no-op')
    ap.add_argument('--w-depth', type=float, default=0.5, help='compatibility no-op')
    ap.add_argument('--w-lidar', type=float, default=3.5, help='compatibility no-op')
    ap.add_argument('--lidar-gate', type=float, default=1.0, help='compatibility no-op')
    ap.add_argument('--huber', type=float, default=0.05, help='compatibility no-op')
    ap.add_argument('--triangulate', type=int, default=0, help='compatibility no-op')
    ap.add_argument('--tri-min-parallax', type=float, default=3.0, help='compatibility no-op')
    ap.add_argument('--tri-outlier', type=float, default=0.5, help='compatibility no-op')
    ap.add_argument('--selftest', action='store_true', help='offline synthetic math check, then exit')
    args, _ros = ap.parse_known_args()

    if args.selftest:
        return _selftest()
    return run_node(args)


if __name__ == '__main__':
    sys.exit(main())
