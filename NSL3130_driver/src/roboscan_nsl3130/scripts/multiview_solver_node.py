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
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation as Rot
except ImportError:  # pragma: no cover
    print('[mv_solver] ERROR: scipy required (scipy.optimize + scipy.spatial.transform).',
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

    The tag pose is RE-DERIVED on the host from the published mean corners (both IPPE solutions)
    so the solver can disambiguate the planar flip globally; falls back to the edge's pose if no
    corners were sent."""
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
        poses = _poses_from_corners(tg.get('corners_px'), size, K, D, fish)
        if poses:
            R, t = poses[0]
            R_alt, t_alt = poses[1] if len(poses) > 1 else (None, None)
        else:                                          # no corners → trust the edge's single pose
            R = np.asarray(tg['R'], float).reshape(3, 3)
            t = np.asarray(tg['t'], float).reshape(3)
            R_alt = t_alt = None
        range_lidar, range_conf = None, 1.0
        if depth.get('status') == 'ok':
            range_lidar = float(depth['after_m'])    # LiDAR sliding+RANSAC refined range (the fine-correction)
            used = float(depth.get('used') or 0.0)
            if 'confidence' in depth:
                range_conf = max(0.05, min(1.0, float(depth.get('confidence') or 0.0)))
            else:
                range_conf = (max(0.05, min(1.0, float(depth.get('inliers') or 0.0) / used))
                              if used > 0 else 0.5)    # RANSAC plane inlier ratio = depth confidence
        tags[tid] = {
            'R': R, 't': t, 'R_alt': R_alt, 't_alt': t_alt,
            't_mono': np.asarray(tg.get('t_mono', t), float).reshape(3),
            'size': size,
            'rmse_px': float(tg.get('rmse_px', 0.0)),
            'views': int(tg.get('views', 0)),
            'range_lidar': range_lidar, 'range_conf': range_conf,
            'corners_px': tg.get('corners_px'),    # kept for stage-2 robust PnP against the tag map
        }
    d['tags'] = tags
    return d


# ───────────────────────────── the solver ───────────────────────────────────

class BundleSolver:
    """Pure (no-ROS) global pose-graph solver over camera + tag SE3 poses."""

    def __init__(self, w_rot=2.0, w_lat=4.0, w_depth=0.5, w_lidar=3.5, huber=0.05,
                 rot_angle_pow=1.0, ref_id=7, lidar_gate=1.0, w_up=2.0):
        # RGB bearing (lateral, w_lat) is the strongest cue. The ALONG-RAY DEPTH is driven by the
        # LiDAR sliding+RANSAC range (w_lidar, scaled per-tag by plane confidence) — the fine
        # correction — while w_depth is only a weak monocular fallback for tags with no LiDAR plane.
        # Huber + the confidence scaling keep a grossly-wrong range from hurting. rot_angle_pow
        # relaxes heading for grazing tag views (residuals()).
        # ref_id = the preferred anchor (the big 0.32 m reference tag) — the one shared origin.
        self.w_rot, self.w_lat, self.w_depth = w_rot, w_lat, w_depth
        self.w_lidar, self.huber = w_lidar, huber
        self.rot_angle_pow = rot_angle_pow
        self.ref_id = ref_id
        self.lidar_gate = lidar_gate
        # w_up: all tags are physically HORIZONTAL (lie flat on floor/steps), so each tag's normal
        # must equal the shared up-axis = stag_marker +Z (the anchor's normal). This constraint
        # deterministically resolves the planar IPPE tilt/flip (the horizontal solution wins) and
        # stabilises tag orientation. Set 0 if tags are NOT all horizontal (e.g. on walls).
        self.w_up = w_up

    def _edge_residual(self, R_pred, t_pred, R_meas, t_meas, range_lidar, range_conf=1.0):
        """Residual list for ONE (camera sees tag) observation. Rotation (heading relaxed for
        grazing views) + ray-split translation (lateral strong; the along-ray depth is driven by
        the LiDAR range prior, scaled by its plane confidence). Reused by the optimizer AND the
        IPPE flip-repair scorer."""
        out = []
        n = float(np.linalg.norm(t_meas))
        u = t_meas / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])
        rot = Rot.from_matrix(R_meas.T @ R_pred).as_rotvec()
        if self.rot_angle_pow > 0.0:
            cos_inc = abs(float(R_meas[:, 2] @ u))          # 1 = frontal tag, 0 = edge-on
            w_rot_eff = self.w_rot * (cos_inc ** self.rot_angle_pow)
        else:
            w_rot_eff = self.w_rot
        out.extend(w_rot_eff * rot)
        dt = t_pred - t_meas
        along = float(dt @ u) * u
        perp = dt - along
        out.extend(np.sqrt(self.w_lat) * perp + np.sqrt(self.w_depth) * along)
        if range_lidar is not None:
            # LiDAR range prior, weighted by the RANSAC plane confidence (inlier ratio): a clean
            # plane (conf≈1) pins this tag's depth hard; a weak/noisy one is trusted proportionally
            # less. This is the depth fine-correction — and what makes a high w_lidar safe.
            out.append(np.sqrt(self.w_lidar * max(0.0, range_conf))
                       * (float(np.linalg.norm(t_pred)) - range_lidar))
        return out

    # --- graph -----------------------------------------------------------------
    def _priority(self, tid):
        """Main-tag priority for anchoring + init order, per the rule
        id 7 (the big 0.32 m tag) → id 0 → id 1 → … :  ref_id first, then ascending id.
        Lower tuple = higher priority."""
        return (0, 0) if int(tid) == self.ref_id else (1, int(tid))

    def _anchor_id(self, observations, forced):
        seen = set()
        for ob in observations:
            seen.update(ob['tags'])
        if not seen:
            return None
        if forced is not None and forced >= 0 and int(forced) in seen:
            return int(forced)                       # explicit --anchor-id wins (if observed)
        # else the highest-PRIORITY visible tag (ref_id, then lowest id) — one deterministic,
        # stable origin so cameras can never anchor to different "mains".
        return min(seen, key=self._priority)

    @staticmethod
    def _obs_quality(tg):
        """How trustworthy this observation is for building the initial TF: big, frontal,
        low-reprojection tags pin pose best and resist the planar flip."""
        t = np.asarray(tg['t'], float)
        n = float(np.linalg.norm(t))
        front = abs(float(np.asarray(tg['R'], float)[:, 2] @ (t / n))) if n > 1e-9 else 0.0
        return float(tg.get('size', 0.19)) * front / (1.0 + float(tg.get('rmse_px', 1.0)))

    def _spanning_init(self, observations, anchor):
        """Spanning tree over the bipartite camera<->tag graph from the anchor tag (origin=I),
        growing in MAIN-TAG PRIORITY order (id 7 → 0 → 1 → …), and within a priority by observation
        quality. Building the backbone from the high-priority (big, reliable) tags first — and
        chaining the rest onto them in order — is what keeps the TF from twisting. Returns
        (cam_pose, tag_pose, reached_*)."""
        order = sorted(((self._priority(tid), -self._obs_quality(tg), ci, tid)
                        for ci, ob in enumerate(observations) for tid, tg in ob['tags'].items()),
                       key=lambda c: (c[0], c[1]))
        cand = [(ci, tid) for _p, _q, ci, tid in order]
        tag_pose = {anchor: se3_identity()}
        cam_pose = {}
        changed = True
        while changed:
            changed = False
            for ci, tid in cand:
                tg = observations[ci]['tags'][tid]
                T_ci_tj = (tg['R'], tg['t'])                 # tag in camera (current IPPE solution)
                if tid in tag_pose and ci not in cam_pose:
                    cam_pose[ci] = se3_compose(tag_pose[tid], se3_inv(T_ci_tj)); changed = True
                if ci in cam_pose and tid not in tag_pose:
                    tag_pose[tid] = se3_compose(cam_pose[ci], T_ci_tj); changed = True
        return cam_pose, tag_pose, set(cam_pose), set(tag_pose)

    # --- optimization ----------------------------------------------------------
    def _solve_once(self, observations, anchor):
        """Spanning-tree init + one bundle adjustment for the tag poses AS THEY STAND in
        `observations` (a tag's 'R'/'t' is whichever IPPE solution is currently selected).
        Returns (cp, tp, rms, free_cams, free_tags, reached_cams, reached_tags) or None."""
        cam_pose, tag_pose, rc, rt = self._spanning_init(observations, anchor)
        free_cams = sorted(rc)
        free_tags = sorted(rt - {anchor})            # anchor stays fixed at the origin
        cam_slot = {ci: k for k, ci in enumerate(free_cams)}
        tag_slot = {tid: len(free_cams) + k for k, tid in enumerate(free_tags)}
        edges = [(ci, tid, tg['R'], tg['t'], tg['range_lidar'], tg.get('range_conf', 1.0))
                 for ci in free_cams for tid, tg in observations[ci]['tags'].items() if tid in tag_pose]
        if not edges:
            return None

        def unpack(x):
            cp = {ci: params_to_pose(x[cam_slot[ci] * 6:cam_slot[ci] * 6 + 6]) for ci in free_cams}
            tp = {anchor: se3_identity()}
            for tid in free_tags:
                tp[tid] = params_to_pose(x[tag_slot[tid] * 6:tag_slot[tid] * 6 + 6])
            return cp, tp

        up = np.array([0.0, 0.0, 1.0])

        def residuals(x):
            cp, tp = unpack(x)
            res = []
            for ci, tid, R_meas, t_meas, rl, rc in edges:
                R_pred, t_pred = se3_compose(se3_inv(cp[ci]), tp[tid])
                res.extend(self._edge_residual(R_pred, t_pred, R_meas, t_meas, rl, rc))
            # up-normal: every tag is horizontal → its normal (R[:,2]) must equal stag_marker +Z.
            if self.w_up > 0.0:
                for tid in free_tags:
                    res.extend(self.w_up * (tp[tid][0][:, 2] - up))
            return np.asarray(res, float)

        x0 = np.zeros(6 * (len(free_cams) + len(free_tags)))
        for ci in free_cams:
            x0[cam_slot[ci] * 6:cam_slot[ci] * 6 + 6] = pose_to_params(cam_pose[ci])
        for tid in free_tags:
            x0[tag_slot[tid] * 6:tag_slot[tid] * 6 + 6] = pose_to_params(tag_pose[tid])
        sol = least_squares(residuals, x0, loss='huber', f_scale=self.huber, method='trf', max_nfev=200)
        cp, tp = unpack(sol.x)
        rms = float(np.sqrt(np.mean(sol.fun ** 2))) if sol.fun.size else 0.0
        return cp, tp, rms, free_cams, free_tags, rc, rt

    def _obs_resid(self, cp, tp, ci, tg):
        R_pred, t_pred = se3_compose(se3_inv(cp[ci]), tp[tg['_tid']])
        r = np.asarray(self._edge_residual(R_pred, t_pred, tg['R'], tg['t'],
                                           tg['range_lidar'], tg.get('range_conf', 1.0)))
        return float(r @ r)

    def _relocalize_pnp(self, observations, cp, tp, free_cams):
        """Stage 2 (Approach A): FREEZE the tag map (tp) and re-localize EACH camera by robust PnP
        of its tag corners against the map's 3D corners. solvePnPRansac inliers are the VOTE that
        drops this camera's own bad tag observations — so a noisy camera can only hurt itself, not
        the map, and its depth comes from the 3D-spread map (not its weak per-tag depth). Needs the
        published corners + intrinsics; cameras with <2 usable tags keep their stage-1 pose.
        Returns (cam_pose, pnp_info{camera_id: (inliers, n_tags)})."""
        if cv2 is None:
            return dict(cp), {}, []
        new_cp, pnp_info, outliers = dict(cp), {}, []
        for ci in free_cams:
            ob = observations[ci]
            if ob.get('K') is None:
                continue
            K = np.asarray(ob['K'], float).reshape(3, 3)
            D = np.asarray(ob.get('D') or [0, 0, 0, 0], float).reshape(1, -1)
            fish = bool(ob.get('fisheye', False))
            objp, imgp, tid_order = [], [], []
            for tid, tg in ob['tags'].items():
                c = tg.get('corners_px')
                if tid not in tp or not c or len(c) != 8:
                    continue
                h = float(tg['size']) / 2.0
                local = np.array([[-h, h, 0.], [h, h, 0.], [h, -h, 0.], [-h, -h, 0.]], float)
                Rt, tt = tp[tid]
                objp.append((Rt @ local.T).T + tt)              # tag corners in stag_marker (world)
                imgp.append(np.asarray(c, float).reshape(4, 2))
                tid_order.append(tid)
            if len(objp) < 2:                                   # need ≥2 spread tags for a stable PnP
                continue
            obj = np.concatenate(objp).astype(np.float64)
            img = np.concatenate(imgp).reshape(-1, 1, 2).astype(np.float64)
            try:
                if fish:
                    und = cv2.fisheye.undistortPoints(img, K, D[:, :4].reshape(1, 4), P=K)
                else:
                    und = cv2.undistortPoints(img, K, D, P=K)
                und = np.ascontiguousarray(und.reshape(-1, 1, 2))
                Rcw, tcw = se3_inv(cp[ci])                       # world→camera init from stage-1
                rvec0 = cv2.Rodrigues(np.ascontiguousarray(Rcw))[0]
                tvec0 = np.asarray(tcw, float).reshape(3, 1)
                ok, rvec, tvec, inl = cv2.solvePnPRansac(
                    obj, und, K, None, rvec0.copy(), tvec0.copy(), True, 200, 3.0)
            except cv2.error:
                ok = False
            if not ok or rvec is None:
                continue
            R_cw = cv2.Rodrigues(rvec)[0]
            new_cp[ci] = se3_inv((R_cw, np.asarray(tvec, float).reshape(3)))   # camera-in-world
            inl_set = set(int(i) for i in np.asarray(inl).reshape(-1)) if inl is not None else set()
            for k, tid in enumerate(tid_order):                 # a tag is an OUTLIER if ≤1/4 corners inlier
                if len({4 * k, 4 * k + 1, 4 * k + 2, 4 * k + 3} & inl_set) < 2:
                    outliers.append((ci, tid))
            pnp_info[ob['camera_id']] = (len(inl_set), len(tid_order))
        return new_cp, pnp_info, outliers

    def _refine_reprojection(self, observations, cp, tp, free_cams, anchor):
        """Final JOINT bundle adjustment that minimises the actual PIXEL REPROJECTION of every tag
        corner across all cameras + tag poses (anchor fixed). This is the ML estimator — the
        pose-residual BA above only approximates it with hand-weighted rot/trans terms. Minimising
        pixels couples every corner of every camera optimally and naturally weights near vs far
        tags, so it extracts the most the (possibly weak) geometry allows and spreads the anchor's
        error globally instead of hinging on one tag. Needs corners+intrinsics; init from the PnP
        solution (so it can't diverge). Returns refined (cp, tp), or the inputs if no corners."""
        if cv2 is None:
            return cp, tp
        obs_px = []
        for ci in free_cams:
            ob = observations[ci]
            if ob.get('K') is None:
                continue
            K = np.asarray(ob['K'], float).reshape(3, 3)
            D = np.asarray(ob.get('D') or [0, 0, 0, 0], float).reshape(1, -1)
            fish = bool(ob.get('fisheye', False))
            for tid, tg in ob['tags'].items():
                c = tg.get('corners_px')
                if tid not in tp or not c or len(c) != 8:
                    continue
                pts = np.asarray(c, float).reshape(4, 1, 2)
                try:
                    und = (cv2.fisheye.undistortPoints(pts, K, D[:, :4].reshape(1, 4), P=K) if fish
                           else cv2.undistortPoints(pts, K, D, P=K))
                except cv2.error:
                    continue
                obs_px.append((ci, tid, und.reshape(4, 2), K, float(tg['size'])))
        if len(obs_px) < 3:
            return cp, tp
        free_tags = sorted(set(tp) - {anchor})
        cam_slot = {ci: k for k, ci in enumerate(free_cams)}
        tag_slot = {tid: len(free_cams) + k for k, tid in enumerate(free_tags)}

        def unpack(x):
            cpx = {ci: params_to_pose(x[cam_slot[ci] * 6:cam_slot[ci] * 6 + 6]) for ci in free_cams}
            tpx = {anchor: se3_identity()}
            for tid in free_tags:
                tpx[tid] = params_to_pose(x[tag_slot[tid] * 6:tag_slot[tid] * 6 + 6])
            return cpx, tpx

        def residuals(x):
            cpx, tpx = unpack(x)
            res = []
            for ci, tid, und, K, size in obs_px:
                Rcw, tcw = se3_inv(cpx[ci])
                Rt, tt = tpx[tid]
                h = size / 2.0
                local = np.array([[-h, h, 0.], [h, h, 0.], [h, -h, 0.], [-h, -h, 0.]], float)
                cam = (Rcw @ ((Rt @ local.T).T + tt).T).T + tcw     # corners in camera frame
                proj = (K @ cam.T).T
                z = np.clip(proj[:, 2:3], 1e-3, None)
                res.extend((proj[:, :2] / z - und).reshape(-1))     # pixel reprojection error
            return np.asarray(res, float)

        x0 = np.zeros(6 * (len(free_cams) + len(free_tags)))
        for ci in free_cams:
            x0[cam_slot[ci] * 6:cam_slot[ci] * 6 + 6] = pose_to_params(cp[ci])
        for tid in free_tags:
            x0[tag_slot[tid] * 6:tag_slot[tid] * 6 + 6] = pose_to_params(tp[tid])
        try:
            sol = least_squares(residuals, x0, loss='huber', f_scale=2.0, method='trf', max_nfev=100)
        except Exception:                                            # numerically fragile → keep PnP
            return cp, tp
        return unpack(sol.x)

    def _gate_lidar_priors(self, observations, anchor):
        """Reject LiDAR ranges that disagree with a first RGB/bridge-only solve."""
        gate_m = float(self.lidar_gate)
        if gate_m <= 0.0:
            return [], []
        rgb_only = [
            {**ob, 'tags': {tid: dict(tg, range_lidar=None, range_conf=0.0)
                            for tid, tg in ob['tags'].items()}}
            for ob in observations
        ]
        probe = self._solve_once(rgb_only, anchor)
        if probe is None:
            return [], []
        cp0, tp0, _rms, free_cams, _free_tags, _rc, _rt = probe
        rejected, softened = [], []
        soft_start = 0.5 * gate_m
        for ci in free_cams:
            ob = observations[ci]
            for tid, tg in ob['tags'].items():
                if tid not in tp0 or tg.get('range_lidar') is None:
                    continue
                _R_pred, t_pred = se3_compose(se3_inv(cp0[ci]), tp0[tid])
                predicted = float(np.linalg.norm(t_pred))
                measured = float(tg['range_lidar'])
                err = abs(predicted - measured)
                tg['range_lidar_rgb_error_m'] = err
                if err > gate_m:
                    rejected.append((ob['camera_id'], tid, err, measured, predicted))
                    tg['range_lidar_rejected_m'] = measured
                    tg['range_lidar'] = None
                    tg['range_conf'] = 0.0
                elif err > soft_start:
                    scale = max(0.10, (gate_m - err) / max(1e-9, gate_m - soft_start))
                    tg['range_conf'] = float(tg.get('range_conf', 1.0)) * scale
                    softened.append((ob['camera_id'], tid, err, measured, predicted, scale))
        return rejected, softened

    def solve(self, observations, anchor_id=None):
        anchor = self._anchor_id(observations, anchor_id)
        if anchor is None:
            return {'ok': False, 'error': 'no tags in any observation'}
        # mutable working copy — we physically swap a tag's pose to its IPPE alternate during the
        # flip-repair, so the spanning-tree init and the BA both see the chosen solution.
        observations = [{**ob, 'tags': {tid: dict(tg, _tid=tid) for tid, tg in ob['tags'].items()}}
                        for ob in observations]
        lidar_rejected, lidar_softened = self._gate_lidar_priors(observations, anchor)

        base = self._solve_once(observations, anchor)
        if base is None:
            return {'ok': False, 'error': 'no usable observations after connectivity check'}
        cp, tp, best_rms, free_cams, free_tags, rc, rt = base
        rms0 = best_rms

        # IPPE flip-repair by GLOBAL descent: flip a tag's pose to its planar alternate, re-init +
        # re-solve FROM SCRATCH, and keep the flip only if the whole-graph rms drops. Re-init (not
        # warm start) so a wrong tilt that pulled the graph into a bad basin is actually escaped —
        # a local per-edge check can't see that, only the global rms can.
        flippable = [(ci, tid) for ci in free_cams for tid in observations[ci]['tags']
                     if tid in tp and observations[ci]['tags'][tid].get('R_alt') is not None]
        flipped = set()
        for _ in range(3):
            improved = False
            flippable.sort(key=lambda k: self._obs_resid(cp, tp, k[0], observations[k[0]]['tags'][k[1]]),
                           reverse=True)
            for ci, tid in flippable:
                tg = observations[ci]['tags'][tid]
                tg['R'], tg['R_alt'] = tg['R_alt'], tg['R']      # try the flip
                tg['t'], tg['t_alt'] = tg['t_alt'], tg['t']
                trial = self._solve_once(observations, anchor)
                if trial is not None and trial[2] < best_rms - 1e-6:
                    cp, tp, best_rms = trial[0], trial[1], trial[2]
                    flipped ^= {(ci, tid)}
                    improved = True
                else:
                    tg['R'], tg['R_alt'] = tg['R_alt'], tg['R']  # revert — that flip did not help
                    tg['t'], tg['t_alt'] = tg['t_alt'], tg['t']
            if not improved:
                break

        # Stage 2 (Approach A): freeze the tag map → robust per-camera PnP against it. PnP RANSAC
        # flags outlier observations (a mis-detected tag in ONE view) via reprojection — a far more
        # sensitive detector than a pose residual. We drop those from the MAP, re-solve once, and
        # re-PnP, so a noisy camera can't corrupt the SHARED tag positions (the others). Generic
        # voting: whichever observation reprojects badly is the one removed — no per-camera guessing.
        # (No-op for cameras without published corners → they keep the stage-1 pose.)
        cp, pnp_info, outliers = self._relocalize_pnp(observations, cp, tp, free_cams)
        dropped = []
        if outliers:
            for ci, tid in outliers:
                observations[ci]['tags'].pop(tid, None)
                dropped.append((observations[ci]['camera_id'], tid))
            re = self._solve_once(observations, anchor)
            if re is not None:
                cp, tp, best_rms, free_cams, free_tags, rc, rt = re
                cp, pnp_info, _ = self._relocalize_pnp(observations, cp, tp, free_cams)

        # Stage 3 — final JOINT corner-reprojection bundle adjustment (ML estimator). Couples every
        # camera + tag through the actual pixels (init from the PnP solution so it can't diverge),
        # spreading residual error globally instead of hinging on one anchor tag.
        cp, tp = self._refine_reprojection(observations, cp, tp, free_cams, anchor)

        # diagnostics
        isolated_cams = [ob['camera_id'] for i, ob in enumerate(observations) if i not in rc]
        all_tags = set().union(*[set(ob['tags']) for ob in observations]) if observations else set()
        isolated_tags = sorted(all_tags - rt)
        flips = sorted((observations[ci]['camera_id'], tid) for ci, tid in flipped)
        edge_resid = sorted(((observations[ci]['camera_id'], tid,
                              float(np.sqrt(self._obs_resid(cp, tp, ci, observations[ci]['tags'][tid]))))
                             for ci in free_cams for tid in observations[ci]['tags'] if tid in tp),
                            key=lambda r: r[2], reverse=True)
        bridges = {}
        for ob in observations:
            for tid in ob['tags']:
                bridges.setdefault(tid, []).append(ob['camera_id'])
        return {
            'ok': True, 'anchor': anchor,
            'cam_pose': cp, 'tag_pose': tp,
            'free_cams': free_cams, 'free_tags': free_tags,
            'isolated_cams': isolated_cams, 'isolated_tags': isolated_tags,
            'edges': len(edge_resid), 'rms_before': rms0, 'rms_after': best_rms,
            'flips': flips, 'edge_resid': edge_resid, 'bridges': bridges,
            'lidar_rejected': lidar_rejected, 'lidar_softened': lidar_softened,
            'n_alt': len(flippable), 'pnp': pnp_info, 'dropped': dropped,
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
            super().__init__('multiview_solver_node')
            self.obs = {}                       # camera_id -> observation dict
            self._subs = {}
            self._put_clients = {}              # service name -> PutWeight client (writeback)
            self.solver = BundleSolver(args.w_rot, args.w_lat, args.w_depth,
                                       args.w_lidar, args.huber, args.rot_angle_pow, args.ref_id,
                                       args.lidar_gate, args.w_up)
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
            ob_by_id = {ob['camera_id']: ob for ob in obs}
            self.get_logger().info(
                f'[mv_solver] ({why}) anchor=id{res["anchor"]} (origin)  cams={len(res["free_cams"])}  '
                f'tags={len(res["free_tags"]) + 1}  edges={res["edges"]}  '
                f'rms {res["rms_before"]:.4f}→{res["rms_after"]:.4f}')
            # bridge graph — which tag id is seen by which cameras (★ = the anchor/origin). A tag
            # seen by ≥2 cameras BRIDGES them; this is the correspondence that ties the views.
            for tid in sorted(res.get('bridges', {})):
                cams = sorted(set(res['bridges'][tid]))
                star = ' ★anchor' if tid == res['anchor'] else (' (bridge)' if len(cams) >= 2 else '')
                self.get_logger().info(f'    id{tid}: {", ".join(cams)}{star}')
            if res.get('flips'):
                self.get_logger().warn(
                    f'[mv_solver] IPPE flip-repaired (wrong tilt → corrected): {res["flips"]}')
            if res.get('dropped'):
                self.get_logger().warn(
                    f'[mv_solver] outlier observations VOTED OUT of the map (bad tag in a view): '
                    f'{res["dropped"]}')
            if res.get('pnp'):
                pnp_str = ', '.join(f'{c}:{i}/{n*4}in' for c, (i, n) in res['pnp'].items())
                self.get_logger().info(f'    stage-2 PnP (camera ← frozen tag map): {pnp_str}')
            for cam, tid, err, measured, predicted in res.get('lidar_rejected', [])[:5]:
                self.get_logger().warn(
                    f'    lidar gate rejected {cam}/id{tid}: LiDAR {measured:.3f} m vs '
                    f'RGB-bridge {predicted:.3f} m (Δ{err:.3f} m)')
            if len(res.get('lidar_rejected', [])) > 5:
                self.get_logger().warn(
                    f'    lidar gate rejected {len(res["lidar_rejected"]) - 5} more observations')
            for cam, tid, err, measured, predicted, scale in res.get('lidar_softened', [])[:3]:
                self.get_logger().info(
                    f'    lidar gate softened {cam}/id{tid}: LiDAR {measured:.3f} m vs '
                    f'RGB-bridge {predicted:.3f} m (Δ{err:.3f} m, conf×{scale:.2f})')
            if res.get('n_alt', 0) == 0:
                self.get_logger().warn(
                    '[mv_solver] no corner data in observations → IPPE flip-repair is OFF. Deploy the '
                    'OBSERVE update to the edges (mgp && mcb) so wrong tilts can be corrected.')
            # worst-fitting observations — a high residual means a bad detection / mismatch here.
            # NOTE: separate warn/info call sites on purpose — rclpy forbids changing a log call
            # site's severity between calls ("Logger severity cannot be changed between calls").
            for cam, tid, rr in res.get('edge_resid', [])[:3]:
                if rr > 0.5:
                    self.get_logger().warn(f'    residual {cam}/id{tid} = {rr:.3f}  ← HIGH, check this tag')
                else:
                    self.get_logger().info(f'    residual {cam}/id{tid} = {rr:.3f}')
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
                R0, t0 = se3_compose(se3_inv(cams_gt[ci]), tags_gt[tid])    # clean tag in camera
                R = (Rot.from_matrix(R0) * Rot.from_rotvec(rng.normal(0, r_noise, 3))).as_matrix()
                t = t0 + rng.normal(0, t_noise, 3)
                # LiDAR range = an INDEPENDENT depth measurement of the true range (a separate
                # sensor), not |noisy t| — so the depth constraint adds real info as it does live.
                range_lidar = float(np.linalg.norm(t0) + rng.normal(0, t_noise))
                tags[tid] = {'R': R, 't': t, 't_mono': t, 'size': sizes[tid], 'rmse_px': 0.5,
                             'views': 10, 'range_lidar': range_lidar, 'range_conf': 1.0}
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
    # rot_angle_pow=0 here: the synthetic tags have RANDOM orientations (not the real grazing
    # geometry the angle-relaxation targets), so we validate the core solver math at full
    # rotation trust. The cos(incidence) relaxation is exercised in the field, not this scene.
    rA = BundleSolver(rot_angle_pow=0.0, w_up=0.0).solve(obs, anchor_id=0)
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
    rB = BundleSolver(rot_angle_pow=0.0, w_up=0.0).solve(obs, anchor_id=0)
    mtB, mrB = pose_errs(rB, cams_gt)
    print(f'[selftest B/loop]   anchor=id{rB["anchor"]} cams={len(rB["free_cams"])}/3 '
          f'edges={rB["edges"]} rms {rB["rms_before"]:.4f}→{rB["rms_after"]:.4f}  '
          f'maxΔt={mtB*1000:.1f}mm maxΔR={np.degrees(mrB):.2f}°')
    okB = (rB['ok'] and not rB['isolated_cams'] and rB['rms_after'] <= rB['rms_before'] + 1e-9
           and mtB < 0.02 and mrB < 0.02)
    print('  ' + ('PASS — loop closure optimized, poses recovered' if okB else 'FAIL'))
    ok = ok and okB

    # (D) IPPE FLIP-REPAIR — corrupt cam0's view of the SHARED tag1 with a 35° wrong tilt and offer
    # the correct pose as the alternate; the solver must detect the bad tilt (it fights cam1's
    # correct view of tag1) and flip it back, still recovering all poses.
    cams_gt = {0: rand_pose(1.2), 1: rand_pose(1.2), 2: rand_pose(1.2)}
    tags_gt = {0: se3_identity(), 1: rand_pose(0.6), 2: rand_pose(0.6), 3: rand_pose(0.6)}
    obs = make_obs(cams_gt, tags_gt, {0: [0, 1], 1: [1, 2], 2: [2, 0, 3]}, 0.003, 0.004)
    for ob in obs:                                          # every tag: correct pose as alternate
        for tg in ob['tags'].values():
            tg['R_alt'], tg['t_alt'] = tg['R'].copy(), tg['t'].copy()
    # clean baseline for THIS exact scene (shallow copy is enough — we reassign R, never mutate it)
    obs_clean = [{**ob, 'tags': {tid: dict(tg) for tid, tg in ob['tags'].items()}} for ob in obs]
    mt_clean = pose_errs(BundleSolver(rot_angle_pow=0.0, w_up=0.0).solve(obs_clean, anchor_id=0), cams_gt)[0]
    obs[0]['tags'][1]['R'] = obs[0]['tags'][1]['R'] @ Rot.from_rotvec([0.61, 0.0, 0.0]).as_matrix()
    rD = BundleSolver(rot_angle_pow=0.0, w_up=0.0).solve(obs, anchor_id=0)
    mtD, mrD = pose_errs(rD, cams_gt)
    print(f'[selftest D/flip]   flips={rD["flips"]} maxΔt={mtD*1000:.1f}mm (clean {mt_clean*1000:.1f}mm) '
          f'maxΔR={np.degrees(mrD):.2f}°')
    okD = rD['ok'] and ('CAM0', 1) in rD['flips'] and mtD < mt_clean + 0.005
    print('  ' + ('PASS — wrong IPPE tilt flipped back, recovered to clean baseline' if okD else 'FAIL'))
    ok = ok and okD

    # (E) REF ANCHOR — with ref_id=2 and no forced anchor, the big reference tag must win as origin.
    rE = BundleSolver(ref_id=2, w_up=0.0).solve(obs, anchor_id=-1)
    okE = rE['ok'] and rE['anchor'] == 2
    print(f'[selftest E/anchor] ref_id=2 → anchor=id{rE["anchor"]}  ' + ('PASS' if okE else 'FAIL'))
    ok = ok and okE

    # (F) LiDAR OUTLIER GATE — corrupt one LiDAR plane by +1 m. The RGB/bridge-only prepass should
    # reject that range before the weighted BA, so one bad depth plane cannot pull a camera away.
    cams_gt = {0: rand_pose(1.2), 1: rand_pose(1.2), 2: rand_pose(1.2)}
    tags_gt = {0: se3_identity(), 1: rand_pose(0.6), 2: rand_pose(0.6), 3: rand_pose(0.6)}
    obs = make_obs(cams_gt, tags_gt, {0: [0, 1], 1: [1, 2], 2: [2, 0, 3]}, 0.003, 0.004)
    obs[2]['tags'][0]['range_lidar'] += 1.0
    rF = BundleSolver(rot_angle_pow=0.0, w_lidar=10.0, lidar_gate=0.25, w_up=0.0).solve(obs, anchor_id=0)
    mtF, mrF = pose_errs(rF, cams_gt)
    rejectedF = {(cam, tid) for cam, tid, _err, _meas, _pred in rF.get('lidar_rejected', [])}
    okF = rF['ok'] and ('CAM2', 0) in rejectedF and mtF < 0.03 and mrF < 0.02
    print(f'[selftest F/lidar]  rejected={rF.get("lidar_rejected", [])} '
          f'maxΔt={mtF*1000:.1f}mm maxΔR={np.degrees(mrF):.2f}°  '
          + ('PASS' if okF else 'FAIL'))
    ok = ok and okF

    # (G) APPROACH A end-to-end — HORIZONTAL tags spread in 3D + per-camera corners; one NOISY
    # camera (cam2, 4 px corner noise vs 0.3 px). up-normal + stage-2 robust PnP must localize ALL
    # cameras (incl the noisy one) against the frozen map, with NO special-casing of which is bad.
    Kg = np.array([[1000., 0, 960.], [0, 1000., 540.], [0, 0, 1.]])
    tags_gtG = {0: (np.eye(3), np.array([0., 0., 0.])),          # all horizontal (normal = +Z),
                1: (np.eye(3), np.array([1.3, 0.3, 0.45])),      # spread in x/y AND height (3D)
                2: (np.eye(3), np.array([0.4, 1.2, 0.85])),
                3: (np.eye(3), np.array([-0.9, 0.7, 0.25]))}
    def look_at(eye, target):                                   # camera-in-world (R,t) facing target
        eye = np.asarray(eye, float); target = np.asarray(target, float)
        f = target - eye; f = f / np.linalg.norm(f)             # +Z optical (forward)
        r = np.cross(f, np.array([0., 0., 1.]))
        r = r / np.linalg.norm(r) if np.linalg.norm(r) > 1e-6 else np.array([1., 0., 0.])  # +X right
        d = np.cross(f, r)                                      # +Y down
        return np.column_stack([r, d, f]), eye
    ctrG = np.array([0.2, 0.55, 0.4])                           # tag-cluster centroid
    cams_gtG = {0: look_at([-2.0, -1.2, 1.9], ctrG),            # cameras spread + looking AT the tags
                1: look_at([1.6, -1.8, 2.0], ctrG),
                2: look_at([2.4, 1.6, 1.7], ctrG)}

    def corners_uv(ci, tid, pix_noise):
        Rcw, tcw = se3_inv(cams_gtG[ci])
        Rt, tt = tags_gtG[tid]
        h = sizes[tid] / 2.0
        local = np.array([[-h, h, 0.], [h, h, 0.], [h, -h, 0.], [-h, -h, 0.]], float)
        world = (Rt @ local.T).T + tt
        cam = (Rcw @ world.T).T + tcw
        uv = (Kg @ cam.T).T
        uv = uv[:, :2] / uv[:, 2:3]
        return uv + rng.normal(0, pix_noise, uv.shape)

    obsG = []
    for ci in (0, 1, 2):
        tags = {}
        for tid in (0, 1, 2, 3):
            uv = corners_uv(ci, tid, 0.3)
            if ci == 2 and tid == 1:           # cam2 has ONE gross-outlier tag — RANSAC must vote it out
                uv = uv + np.array([38.0, -27.0])
            poses = _poses_from_corners(uv.reshape(-1).tolist(), sizes[tid], Kg.reshape(-1).tolist(),
                                        [0, 0, 0, 0], False)
            if not poses:
                continue
            R, t = poses[0]
            R_alt, t_alt = poses[1] if len(poses) > 1 else (None, None)
            tags[tid] = {'R': R, 't': t, 'R_alt': R_alt, 't_alt': t_alt, 't_mono': t,
                         'size': sizes[tid], 'rmse_px': 0.5, 'views': 10, 'range_lidar': None,
                         'range_conf': 1.0, 'corners_px': uv.reshape(-1).tolist(), '_tid': tid}
        obsG.append({'camera_id': f'CAM{ci}', 'namespace': f'cam_{ci}',
                     'camera_frame': f'cam_{ci}_camera_frame', 'lidar_frame': f'cam_{ci}_lidar_frame',
                     'library_hd': 21, 'marker_size': sizes[0], 'tags': tags,
                     'K': Kg.reshape(-1).tolist(), 'D': [0, 0, 0, 0], 'fisheye': False})
    rG = BundleSolver(rot_angle_pow=0.0, w_up=2.0, w_lidar=0.0).solve(obsG, anchor_id=0)
    mtG, mrG = pose_errs(rG, cams_gtG)
    # tag map should be horizontal (up-normal worked)
    tilt = max((np.degrees(np.arccos(max(-1, min(1, rG['tag_pose'][t][0][2, 2]))))
                for t in rG['tag_pose']), default=0.0)
    perG = {f'CAM{ci}': round(float(np.linalg.norm(rG['cam_pose'][ci][1] - cams_gtG[ci][1])) * 1000, 1)
            for ci in rG['free_cams']}
    print('   per-cam Δt(mm):', perG, ' dropped:', rG.get('dropped'))
    cam2_inl = rG.get('pnp', {}).get('CAM2', (16, 4))[0]    # outlier tag's 4 corners must be voted out
    print(f'[selftest G/approachA] cams={len(rG["free_cams"])}/3 pnp={rG.get("pnp")} '
          f'maxΔt={mtG*1000:.1f}mm maxΔR={np.degrees(mrG):.2f}° max_tag_tilt={tilt:.2f}°')
    okG = (rG['ok'] and len(rG['free_cams']) == 3 and mtG < 0.02 and mrG < 0.01
           and tilt < 1.0 and cam2_inl <= 12)
    print('  ' + ('PASS — up-normal map + RANSAC voted out the outlier tag, all cams localized'
                  if okG else 'FAIL'))
    ok = ok and okG

    print('[selftest] ALL PASS' if ok else '[selftest] FAILED')
    return 0 if ok else 1


# ─────────────────────────────── main ───────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', default=os.path.expanduser('~/mv_bundle_out'),
                    help='Where solved per-camera multiview.yml files are written (then pushed to edges).')
    ap.add_argument('--anchor-id', type=int, default=-1,
                    help='Force a tag id as the stag_marker origin. -1 = auto: prefer --ref-id if '
                         'visible, else the most-co-observed tag.')
    ap.add_argument('--ref-id', type=int, default=7,
                    help='Preferred anchor — the big reference tag used as the ONE shared origin '
                         'when --anchor-id is not forced and this tag is visible. Default 7.')
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
    ap.add_argument('--w-rot', type=float, default=2.0, help='rotation (heading) residual weight')
    ap.add_argument('--rot-angle-pow', type=float, default=1.0,
                    help='RELAX heading for grazing tag views: the rotation weight is scaled by '
                         'cos(incidence)^POW per observation. 0 = off (full heading trust); 1 = '
                         'moderate; higher = more relaxed. Raise this if a camera that only sees '
                         'tags obliquely (e.g. cam_53) tilts its point cloud / leans the pillars.')
    ap.add_argument('--w-lat', type=float, default=4.0, help='lateral (in-image) translation weight')
    ap.add_argument('--w-depth', type=float, default=0.5,
                    help='weak monocular along-ray fallback weight (used only where there is no '
                         'LiDAR plane); keep below w-lidar so the LiDAR fine-correction drives depth')
    ap.add_argument('--w-lidar', type=float, default=3.5,
                    help='LiDAR range prior weight — the DEPTH fine-correction (LiDAR drives depth; '
                         'a far/small tag\'s RGB monocular depth is much less reliable than the direct '
                         'ToF range). Scaled per tag by RANSAC plane confidence. Raise to trust it more.')
    ap.add_argument('--lidar-gate', type=float, default=1.0,
                    help='Reject a LiDAR range prior only when it disagrees with the first RGB/bridge '
                         'solution by more than this many meters (GROSS errors — a plane locked onto a '
                         'wrong surface). Permissive by default since LiDAR depth is generally trusted; '
                         'the plane-confidence scaling handles the rest. <=0 disables the gate.')
    ap.add_argument('--huber', type=float, default=0.05, help='Huber loss scale')
    ap.add_argument('--w-up', type=float, default=2.0,
                    help='Up-normal constraint weight: force every tag horizontal (normal = '
                         'stag_marker +Z). Resolves the IPPE tilt/flip deterministically and '
                         'stabilises tag orientation. Set 0 if tags are NOT all horizontal.')
    ap.add_argument('--selftest', action='store_true', help='offline synthetic math check, then exit')
    args, _ros = ap.parse_known_args()

    if args.selftest:
        return _selftest()
    return run_node(args)


if __name__ == '__main__':
    sys.exit(main())
