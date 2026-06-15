#!/usr/bin/env python3
"""Host-side multi-tag bundle solver — fuse every camera into ONE consistent frame.

Each edge, after a /fleet/calibrate, publishes its per-tag observations as JSON on a
latched `/cam_NN/tag_observations` topic (see multiview_calib_node._publish_observations).
This node collects every camera's snapshot and solves a global SE3 pose graph so that all
cameras and all tags live in one shared `stag_marker` frame — even when no single tag is
visible to every camera. Tags that are co-observed by two cameras BRIDGE them; the graph
only needs to be connected, not to share one universal marker.

TRUST MODEL (the calibration rule):
  • STag rotation + bearing (in-image)  → trusted   (drives the graph)
  • absolute scale                       → from the known tag sizes (RGB only)
  • depth along the camera ray           → WEAK: down-weighted per observation, pinned by
                                           cross-view bridges, and gently nudged toward the
                                           edge's existing LiDAR sliding+RANSAC range.
  • LiDAR depth                          → low-weight soft prior only (never overrides RGB)

Residual per observation (camera i sees tag j, measured tag-in-camera pose R_meas,t_meas):
  rot    = w_rot · log_SO3(R_measᵀ · R_pred)
  trans  = √w_lat · perp + √w_depth · along        (perp/along split about the ray u=t_meas/|t_meas|)
  lidar  = √w_lidar · (|t_pred| − range_lidar)      (only when the edge refined the depth)
where (R_pred,t_pred) = T_camᵢ⁻¹ ∘ T_tagⱼ is the predicted tag-in-camera pose. Robustified
with a Huber loss. The anchor tag is fixed at the origin (gauge); scale comes from tag sizes.

Output: per camera a multiview.yml in the EXACT format multiview_calib_node._save writes
(x_cam = R·x_marker + t, marker = the global anchor tag), so multiview_tf_node and the pose
stack consume it unchanged. Writeback to the edges is a separate step (multiview_put_*).

    ros2 run ros_humanpose multiview_bundle_node.py --out-dir /tmp/mv_bundle
    ros2 run ros_humanpose multiview_bundle_node.py --trigger-topic /fleet/calibrate
    python3 multiview_bundle_node.py --selftest        # offline math check, no ROS traffic
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
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation as Rot
except ImportError:  # pragma: no cover
    print('[mv_bundle] ERROR: scipy required (scipy.optimize + scipy.spatial.transform).',
          file=sys.stderr)
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


def pose_to_params(pose):
    """(R,t) → 6-vector [rotvec(3), t(3)]."""
    R, t = pose
    return np.concatenate([Rot.from_matrix(R).as_rotvec(), np.asarray(t, float).reshape(3)])


def params_to_pose(p):
    return Rot.from_rotvec(p[:3]).as_matrix(), np.asarray(p[3:6], float)


# ─────────────────────────── observation parsing ────────────────────────────

def parse_observation(text):
    """JSON string from /cam_NN/tag_observations → normalized dict, or None if bad."""
    try:
        d = json.loads(text)
    except (ValueError, TypeError):
        return None
    if d.get('schema') != 'nsl_tag_obs_v1' or not d.get('tags'):
        return None
    tags = {}
    for tg in d['tags']:
        tid = int(tg['id'])
        R = np.asarray(tg['R'], float).reshape(3, 3)
        t = np.asarray(tg['t'], float).reshape(3)
        depth = tg.get('depth') or {}
        tags[tid] = {
            'R': R, 't': t,
            't_mono': np.asarray(tg.get('t_mono', t), float).reshape(3),
            'size': float(tg.get('size', 0.0)),
            'rmse_px': float(tg.get('rmse_px', 0.0)),
            'views': int(tg.get('views', 0)),
            'range_lidar': (float(depth['after_m']) if depth.get('status') == 'ok' else None),
        }
    d['tags'] = tags
    return d


# ───────────────────────────── the solver ───────────────────────────────────

class BundleSolver:
    """Pure (no-ROS) global pose-graph solver over camera + tag SE3 poses."""

    def __init__(self, w_rot=2.0, w_lat=4.0, w_depth=0.4, w_lidar=0.2, huber=0.05):
        # depth (along-ray) deliberately far weaker than lateral/rotation — that is the
        # whole point: RGB pins rotation+bearing, bridges/LiDAR settle the weak depth.
        self.w_rot, self.w_lat, self.w_depth = w_rot, w_lat, w_depth
        self.w_lidar, self.huber = w_lidar, huber

    # --- graph -----------------------------------------------------------------
    def _anchor_id(self, observations, forced):
        if forced is not None and forced >= 0:
            return int(forced)
        # the tag seen by the most cameras (most robust origin); ties → lowest id
        seen = {}
        for ob in observations:
            for tid in ob['tags']:
                seen[tid] = seen.get(tid, 0) + 1
        if not seen:
            return None
        best = max(seen.values())
        return min(t for t, c in seen.items() if c == best)

    def _spanning_init(self, observations, anchor):
        """BFS over the bipartite camera<->tag graph from the anchor tag (origin=I).
        Returns (cam_pose{idx}, tag_pose{id}, reached_cams, reached_tags)."""
        cam_tags = [set(ob['tags']) for ob in observations]
        tag_pose = {anchor: se3_identity()}
        cam_pose = {}
        changed = True
        while changed:
            changed = False
            for ci, ob in enumerate(observations):
                for tid, tg in ob['tags'].items():
                    T_ci_tj = (tg['R'], tg['t'])              # tag in camera
                    if tid in tag_pose and ci not in cam_pose:
                        # T_w_ci = T_w_tj ∘ inv(T_ci_tj)
                        cam_pose[ci] = se3_compose(tag_pose[tid], se3_inv(T_ci_tj))
                        changed = True
                    if ci in cam_pose and tid not in tag_pose:
                        # T_w_tj = T_w_ci ∘ T_ci_tj
                        tag_pose[tid] = se3_compose(cam_pose[ci], T_ci_tj)
                        changed = True
        return cam_pose, tag_pose, set(cam_pose), set(tag_pose)

    # --- optimization ----------------------------------------------------------
    def solve(self, observations, anchor_id=None):
        anchor = self._anchor_id(observations, anchor_id)
        if anchor is None:
            return {'ok': False, 'error': 'no tags in any observation'}

        cam_pose, tag_pose, rc, rt = self._spanning_init(observations, anchor)
        # connectivity: anything not reached from the anchor is isolated → reported, dropped
        isolated_cams = [observations[i]['camera_id'] for i in range(len(observations)) if i not in rc]
        all_tags = set().union(*[set(ob['tags']) for ob in observations]) if observations else set()
        isolated_tags = sorted(all_tags - rt)

        free_cams = sorted(rc)
        free_tags = sorted(rt - {anchor})            # anchor stays fixed at the origin
        cam_slot = {ci: k for k, ci in enumerate(free_cams)}
        tag_slot = {tid: len(free_cams) + k for k, tid in enumerate(free_tags)}

        # edges actually used in the cost (both endpoints reached)
        edges = []
        for ci in free_cams:
            for tid, tg in observations[ci]['tags'].items():
                if tid in tag_pose:
                    edges.append((ci, tid, tg))
        if not edges:
            return {'ok': False, 'error': 'no usable observations after connectivity check'}

        def unpack(x):
            cp = {ci: params_to_pose(x[cam_slot[ci] * 6:cam_slot[ci] * 6 + 6]) for ci in free_cams}
            tp = {anchor: se3_identity()}
            for tid in free_tags:
                tp[tid] = params_to_pose(x[tag_slot[tid] * 6:tag_slot[tid] * 6 + 6])
            return cp, tp

        def residuals(x):
            cp, tp = unpack(x)
            res = []
            for ci, tid, tg in edges:
                R_pred, t_pred = se3_compose(se3_inv(cp[ci]), tp[tid])   # tag in camera, predicted
                R_meas, t_meas = tg['R'], tg['t']
                # rotation
                rot = Rot.from_matrix(R_meas.T @ R_pred).as_rotvec()
                res.extend(self.w_rot * rot)
                # translation, split about the measured ray (depth axis = u)
                n = float(np.linalg.norm(t_meas))
                u = t_meas / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])
                dt = t_pred - t_meas
                along = float(dt @ u) * u
                perp = dt - along
                res.extend(np.sqrt(self.w_lat) * perp + np.sqrt(self.w_depth) * along)
                # weak LiDAR range prior on the predicted along-ray distance
                if tg['range_lidar'] is not None:
                    res.append(np.sqrt(self.w_lidar) * (float(np.linalg.norm(t_pred)) - tg['range_lidar']))
            return np.asarray(res, float)

        x0 = np.zeros(6 * (len(free_cams) + len(free_tags)))
        for ci in free_cams:
            x0[cam_slot[ci] * 6:cam_slot[ci] * 6 + 6] = pose_to_params(cam_pose[ci])
        for tid in free_tags:
            x0[tag_slot[tid] * 6:tag_slot[tid] * 6 + 6] = pose_to_params(tag_pose[tid])

        r0 = residuals(x0)
        sol = least_squares(residuals, x0, loss='huber', f_scale=self.huber, method='trf',
                            max_nfev=200)
        cp, tp = unpack(sol.x)
        rms0 = float(np.sqrt(np.mean(r0 ** 2))) if r0.size else 0.0
        rms1 = float(np.sqrt(np.mean(sol.fun ** 2))) if sol.fun.size else 0.0
        return {
            'ok': True, 'anchor': anchor,
            'cam_pose': cp, 'tag_pose': tp,
            'free_cams': free_cams, 'free_tags': free_tags,
            'isolated_cams': isolated_cams, 'isolated_tags': isolated_tags,
            'edges': len(edges), 'rms_before': rms0, 'rms_after': rms1,
            'iterations': int(sol.nfev),
        }


# ─────────────────────────── yml writer (same format) ───────────────────────

def write_multiview_yml(path, ob, anchor, T_w_cam, tag_pose):
    """Write ONE camera's multiview.yml in multiview_calib_node._save's exact format.

    x_cam = R·x_marker + t with marker = the global anchor tag. Per-tag poses are the
    globally-consistent tag-in-this-camera transforms derived from the solved world poses."""
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
    fs.write('R', np.ascontiguousarray(R_anchor, np.float64))
    fs.write('t', np.ascontiguousarray(t_anchor.reshape(3, 1), np.float64))
    # every tag this camera saw, in this camera's frame, from the GLOBAL solution
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
            super().__init__('multiview_bundle_node')
            self.obs = {}                       # camera_id -> observation dict
            self._subs = {}
            self._put_clients = {}              # service name -> PutWeight client (writeback)
            self.solver = BundleSolver(args.w_rot, args.w_lat, args.w_depth,
                                       args.w_lidar, args.huber)
            self.create_timer(1.0, self._discover)
            if args.trigger_topic:
                self.create_subscription(Empty, args.trigger_topic, self._on_trigger, 10)
                self.get_logger().info(
                    f'[mv_bundle] listening; will solve {args.settle:.0f}s after '
                    f'{args.trigger_topic} (or every {args.period:.0f}s if --period>0).')
            if args.period > 0:
                self.create_timer(args.period, lambda: self.solve_now('periodic'))
            self._pending = None

        def _discover(self):
            names = dict(self.get_topic_names_and_types())
            for name in names:
                m = topic_re.match(name)
                if m and name not in self._subs:
                    ns = m.group(1)
                    self._subs[name] = self.create_subscription(
                        String, name, (lambda ns: lambda msg: self._on_obs(ns, msg))(ns), latched)
                    self.get_logger().info(f'[mv_bundle] subscribed {name}')

        def _on_obs(self, ns, msg):
            ob = parse_observation(msg.data)
            if ob is None:
                self.get_logger().warn(f'[mv_bundle] {ns}: unparsable/empty observation')
                return
            self.obs[ob['camera_id']] = ob
            self.get_logger().info(
                f'[mv_bundle] {ns} ({ob["camera_id"]}): {len(ob["tags"])} tags '
                f'{sorted(ob["tags"])}')

        def _on_trigger(self, _msg):
            # solve a settle window after the broadcast so every edge's fresh snapshot lands
            self._pending = self.create_timer(args.settle, self._fire_pending)
            self.get_logger().info(f'[mv_bundle] trigger → solving in {args.settle:.0f}s ...')

        def _fire_pending(self):
            if self._pending is not None:
                self._pending.cancel()
                self._pending = None
            self.solve_now('trigger')

        def solve_now(self, why):
            obs = list(self.obs.values())
            if len(obs) < 1:
                self.get_logger().warn('[mv_bundle] no observations collected yet.')
                return
            res = self.solver.solve(obs, args.anchor_id)
            if not res['ok']:
                self.get_logger().error(f'[mv_bundle] solve failed: {res["error"]}')
                return
            self._report(obs, res, why)

        def _report(self, obs, res, why):
            ob_by_id = {ob['camera_id']: ob for ob in obs}
            self.get_logger().info(
                f'[mv_bundle] ({why}) anchor=id{res["anchor"]}  cams={len(res["free_cams"])}  '
                f'tags={len(res["free_tags"]) + 1}  edges={res["edges"]}  '
                f'rms {res["rms_before"]:.4f}→{res["rms_after"]:.4f}')
            if res['isolated_cams']:
                self.get_logger().warn(
                    f'[mv_bundle] ISOLATED cameras (no shared tag → NOT anchored): '
                    f'{res["isolated_cams"]}')
            if res['isolated_tags']:
                self.get_logger().warn(f'[mv_bundle] isolated tags: {res["isolated_tags"]}')
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
                    f'[mv_bundle] pushed {pushed}/{len(res["free_cams"])} multiview.yml to edges '
                    f'→ each multiview_tf_node re-publishes /tf_static (live).')
            else:
                self.get_logger().info(
                    f'[mv_bundle] wrote {len(res["free_cams"])} multiview.yml under {args.out_dir} '
                    f'(dry-run; re-run with --writeback to push to the edges).')

        def _push(self, ns, yml_path):
            """Chunked + sha256 push of one solved multiview.yml to /<ns>/multiview/put (WRITEBACK).
            Reuses the PutWeight contract; the edge installs it atomically and its multiview_tf_node
            re-publishes /tf_static on the next mtime poll. PutWeight is imported lazily so dry-run
            and --selftest never need ros_humanpose."""
            try:
                from ros_humanpose.srv import PutWeight
            except ImportError as exc:
                self.get_logger().error(f'[mv_bundle] writeback needs ros_humanpose PutWeight: {exc}')
                return False
            svc = f'/{ns}/multiview/put'
            cli = self._put_clients.get(svc)
            if cli is None:
                cli = self.create_client(PutWeight, svc)
                self._put_clients[svc] = cli
            if not cli.wait_for_service(timeout_sec=args.writeback_wait):
                self.get_logger().error(
                    f'[mv_bundle] writeback: {svc} unavailable (waited {args.writeback_wait:.0f}s)')
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
                            f'[mv_bundle] writeback {svc}: '
                            f'{resp.message if resp else "service call timed out"}')
                        return False
                    offset += len(data)
                    if offset >= total:
                        if not resp.complete:
                            self.get_logger().error(
                                f'[mv_bundle] writeback {svc}: server did not confirm completion')
                            return False
                        break
            self.get_logger().info(f'[mv_bundle] writeback → {svc} ({total} B, sha verified)')
            return True

    rclpy.init()
    node = BundleNode()
    try:
        if args.once:
            # one-shot: settle, solve, exit (handy from mtf / a CLI)
            end = time.monotonic() + args.settle
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.solve_now('once')
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


# ───────────────────────────── self test ────────────────────────────────────

def _selftest():
    """Two synthetic checks, NO universally-shared tag in either:
      (A) BRIDGE  — a 3-camera chain (cam0{0,1} cam1{1,2} cam2{2,3}): every camera must be
          anchored under tag0 purely through bridge tags, proving connectivity works.
      (B) LOOP    — a 3-camera cycle (cam0{0,1} cam1{1,2} cam2{2,0} + a leaf tag3): the loop
          closure gives redundancy, so the optimizer must REDUCE rms and recover poses well."""
    rng = np.random.default_rng(1)
    sizes = {0: 0.32, 1: 0.19, 2: 0.19, 3: 0.19}

    def rand_pose(scale=1.0):
        return Rot.from_rotvec(rng.normal(0, 0.4, 3)).as_matrix(), rng.normal(0, scale, 3)

    def make_obs(cams_gt, tags_gt, vis, t_noise, r_noise):
        obs = []
        for ci, tids in vis.items():
            tags = {}
            for tid in tids:
                R, t = se3_compose(se3_inv(cams_gt[ci]), tags_gt[tid])      # tag in camera
                R = (Rot.from_matrix(R) * Rot.from_rotvec(rng.normal(0, r_noise, 3))).as_matrix()
                t = t + rng.normal(0, t_noise, 3)
                tags[tid] = {'R': R, 't': t, 't_mono': t, 'size': sizes[tid], 'rmse_px': 0.5,
                             'views': 10, 'range_lidar': float(np.linalg.norm(t))}
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

    # (A) BRIDGE / connectivity — pure chain, no shared tag, loose accuracy (noise accumulates)
    cams_gt = {0: rand_pose(1.2), 1: rand_pose(1.2), 2: rand_pose(1.2)}
    tags_gt = {0: se3_identity(), 1: rand_pose(0.6), 2: rand_pose(0.6), 3: rand_pose(0.6)}
    obs = make_obs(cams_gt, tags_gt, {0: [0, 1], 1: [1, 2], 2: [2, 3]}, 0.003, 0.004)
    rA = BundleSolver().solve(obs, anchor_id=0)
    mtA, mrA = pose_errs(rA, cams_gt)
    bridgeA = rA['ok'] and not rA['isolated_cams'] and len(rA['free_cams']) == 3
    print(f'[selftest A/bridge] anchor=id{rA["anchor"]} cams={len(rA["free_cams"])}/3 '
          f'edges={rA["edges"]} isolated={rA["isolated_cams"]}  '
          f'maxΔt={mtA*1000:.1f}mm maxΔR={np.degrees(mrA):.2f}°')
    okA = bridgeA and mtA < 0.06
    print('  ' + ('PASS — 3 cams fused via bridge tags, no shared marker' if okA else 'FAIL'))
    ok = ok and okA

    # (B) LOOP / optimization — 3-cycle gives redundancy; rms must drop and poses tighten
    cams_gt = {0: rand_pose(1.2), 1: rand_pose(1.2), 2: rand_pose(1.2)}
    tags_gt = {0: se3_identity(), 1: rand_pose(0.6), 2: rand_pose(0.6), 3: rand_pose(0.6)}
    obs = make_obs(cams_gt, tags_gt, {0: [0, 1], 1: [1, 2], 2: [2, 0, 3]}, 0.003, 0.004)
    rB = BundleSolver().solve(obs, anchor_id=0)
    mtB, mrB = pose_errs(rB, cams_gt)
    print(f'[selftest B/loop]   anchor=id{rB["anchor"]} cams={len(rB["free_cams"])}/3 '
          f'edges={rB["edges"]} rms {rB["rms_before"]:.4f}→{rB["rms_after"]:.4f}  '
          f'maxΔt={mtB*1000:.1f}mm maxΔR={np.degrees(mrB):.2f}°')
    okB = (rB['ok'] and not rB['isolated_cams'] and rB['rms_after'] <= rB['rms_before'] + 1e-9
           and mtB < 0.02 and mrB < 0.02)
    print('  ' + ('PASS — loop closure optimized, poses recovered' if okB else 'FAIL'))
    ok = ok and okB

    print('[selftest] ALL PASS' if ok else '[selftest] FAILED')
    return 0 if ok else 1


# ─────────────────────────────── main ───────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', default=os.path.expanduser('~/mv_bundle_out'),
                    help='Where solved per-camera multiview.yml files are written (then pushed to edges).')
    ap.add_argument('--anchor-id', type=int, default=-1,
                    help='Tag id to fix as the stag_marker origin. -1 = the most-co-observed tag.')
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
    ap.add_argument('--w-rot', type=float, default=2.0, help='rotation residual weight')
    ap.add_argument('--w-lat', type=float, default=4.0, help='lateral (in-image) translation weight')
    ap.add_argument('--w-depth', type=float, default=0.4, help='along-ray (depth) weight — kept LOW')
    ap.add_argument('--w-lidar', type=float, default=0.2, help='weak LiDAR range prior weight')
    ap.add_argument('--huber', type=float, default=0.05, help='Huber loss scale')
    ap.add_argument('--selftest', action='store_true', help='offline synthetic math check, then exit')
    args, _ros = ap.parse_known_args()

    if args.selftest:
        return _selftest()
    return run_node(args)


if __name__ == '__main__':
    sys.exit(main())
