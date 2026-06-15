# Multiview TF Solver — Multi-Tag Bundle Recalibration

How the fleet ties every NSL-3130 camera into **one shared `stag_marker` frame** so RViz / the
pose stack can fuse all views. This replaces the old single-reference-tag calibration with a
**multi-tag bundle adjustment** that is robust to cameras that do *not* share one common marker.

---

## 1. Why this exists

The previous `multiview_calib` flow anchored everything to **one** reference tag (id 7) that had
to be visible in **every** camera:

- If a camera could not see tag 7, the node silently fell back to "the most-collected tag", so
  different cameras anchored to **different physical tags** while both called the frame
  `stag_marker` → they were misaligned by the tag-to-tag offset, with no warning.
- The auxiliary tags were stored but never **fused** — each camera placed them from its own lone
  reference observation, so there was no cross-camera agreement.
- Depth around the floor/tags came from a 1-D ray slide + 1-D RANSAC that easily locked onto the
  wrong surface.

The fix: scatter several tags (e.g. ids 0–3, at varying / stair-step heights), let any tag that two
cameras both see act as a **bridge** between them, and solve **all** camera and tag poses jointly.
No single tag needs to be universally visible — the tag-visibility graph only has to be *connected*.

---

## 2. Trust model (the calibration rule)

The square-marker RGB pose is accurate in heading and bearing but weak in metric depth. The solver
encodes exactly that:

| Quantity | Source | How it is treated |
|---|---|---|
| Rotation / heading | STag corners (PnP) | **trusted** — drives the graph |
| Bearing (in-image direction) | STag corners | **trusted** |
| Absolute scale | known tag side lengths | **from RGB alone** — no LiDAR needed |
| Depth along the camera ray | — | **weak**: down-weighted, pinned by cross-view bridges, gently nudged by the LiDAR range |
| LiDAR depth | point cloud (sliding + RANSAC) | **low-weight soft prior only**; never overrides RGB |

No **GICP** / cloud ICP: it needs geometric overlap that sparse, widely-separated views do not have.
Tags are engineered correspondences — a pair of cameras only needs to share *one* tag to be bridged.

No single global floor plane: tags sit at **different (stepped) heights**, so each tag pose is a free
6-DoF unknown; the bridges and the weak LiDAR prior settle the depths.

---

## 3. Protocol — three stages

```
mtf  ─────▶  /fleet/calibrate (std_msgs/Empty broadcast)
                 │
   ┌─────────────┼─────────────┐   (edges in parallel)
   ▼             ▼             ▼
╔══ EDGE cam_51/52/53 :: (1) OBSERVE  [multiview_calib_node.py] ══╗
║ 1. detect STag (HD21) over an RGB window                        ║
║ 2. per tag: solvePnP (IPPE_SQUARE) -> R,t  (tag in camera)      ║
║ 3. median R/t over N views + reprojection RMSE                  ║
║ 4. LiDAR sliding + RANSAC depth refine  (unchanged, weak prior) ║
║ 5. pack JSON {K,D,fisheye, per-tag: R,t,corners,size,depth}     ║
║    -> publish once on  /cam_NN/tag_observations  (latched)      ║
╚══════════════════════════════════════════════════════════════════╝
                 │  │  │   (one latched JSON topic per camera)
                 ▼  ▼  ▼
╔══ HOST :: (2) SOLVE  [multiview_bundle_node.py] ════════════════╗
║ a. collect every camera's snapshot for a settle window          ║
║ b. build graph: nodes = cameras + tags, edges = observations    ║
║ c. connectivity check: nodes unreachable from the anchor are    ║
║    WARNED and dropped (no silent mis-anchoring)                 ║
║ d. anchor tag = stag_marker origin (fixed); spanning-tree init  ║
║ e. global SE3 bundle adjustment:                                ║
║       min  Σ [ w_rot·rot                                         ║
║             + √w_lat·perp + √w_depth·along    (ray split)       ║
║             + √w_lidar·(|t_pred| − range_lidar) ]               ║
║       Huber robust loss, scipy.optimize.least_squares           ║
║ f. -> per-camera multiview.yml (exact _save format)             ║
╚══════════════════════════════════════════════════════════════════╝
                 │  │  │   (each camera's solved yml)
                 ▼  ▼  ▼
╔══ (3) WRITEBACK  (host -> each edge, reverse direction) ════════╗
║ host streams cam_NN's solved multiview.yml to                   ║
║   /cam_NN/multiview/put   (chunked + sha256, reuses PutWeight)  ║
║ edge installs it atomically at calib_output/{serial}/multiview.yml║
╚══════════════════════════════════════════════════════════════════╝
                 │
                 ▼  (on the edge, automatically)
   multiview_tf_node sees the mtime change (1 s poll)
   -> re-publishes /tf_static  -> host RViz re-anchors live
```

### What "writeback" means

The **SOLVE** step runs on the **host**, but the camera that actually broadcasts each TF is the
**edge** (fleet-local: every edge owns and publishes its own camera frame). So the host has to
**write the solved result *back*** to each edge — push the per-camera `multiview.yml` to the edge,
which installs it; `multiview_tf_node` then re-publishes `/tf_static` on its own. Without writeback
the solution would live only in the host's memory and never reach the actual TF tree.

> Alternative considered: have the host publish **all** cameras' TFs itself (no writeback). Simpler,
> but the TF only lives while the host solver runs, it breaks the "each edge owns its frame" model,
> and the edges' `multiview.yml` (read by `configs.py` for training anchors) would go stale. We chose
> writeback (persist per-edge) to keep the existing fleet-local architecture intact.

---

## 4. The bundle adjustment

A pose is `(R, t)` meaning `x_parent = R · x_child + t`.

- **Variables**: each camera world pose `T_w_cam_i` and each tag world pose `T_w_tag_j` (6 DoF each).
- **Gauge**: the anchor tag is fixed at the origin (`T = I`) and *is* `stag_marker`. Default anchor =
  the tag co-observed by the most cameras (override with `--anchor-id`). Scale is fixed by the known
  tag sizes, so no external scale reference is needed.
- **Initialization**: breadth-first spanning tree from the anchor over the bipartite
  camera↔tag graph — `T_w_cam = T_w_tag ∘ (T_cam_tag)⁻¹` and `T_w_tag = T_w_cam ∘ T_cam_tag`.
- **Residual** for each observation (camera *i* sees tag *j*, measured `R_meas, t_meas`), with
  `(R_pred, t_pred) = T_cam_i⁻¹ ∘ T_w_tag_j`:
  - rotation: `w_rot · log_SO3(R_measᵀ R_pred)`
  - translation, split about the ray `u = t_meas/‖t_meas‖`: `√w_lat · perp + √w_depth · along`
    where `along = ((t_pred − t_meas)·u) u` and `perp = (t_pred − t_meas) − along`
  - LiDAR prior (only when the edge refined the depth): `√w_lidar · (‖t_pred‖ − range_lidar)`
- **Robust loss**: Huber (`f_scale = --huber`) so one bad detection cannot warp the graph.
- **Solver**: `scipy.optimize.least_squares` (TRF). For a 3-camera / ~5-tag fleet this is instant.

`w_depth ≪ w_lat, w_rot` on purpose: rotation and the in-image (lateral) directions are pinned by
RGB, while the weak depth axis is left for the bridges and the LiDAR prior to settle.

### Output / invariants

Each camera's `multiview.yml` is written in the **exact format** of `multiview_calib_node._save`
(`x_cam = R·x_marker + t`, `marker_id` = the global anchor, plus every `tag_k_*`), so
`multiview_tf_node.py` and `configs.py` consume it **unchanged**. A `bundle_solved: 1` flag marks
files produced by the global solve.

---

## 5. Components

| Where | File | Role |
|---|---|---|
| edge | `roboscan_nsl3130/scripts/multiview_calib_node.py` | OBSERVE — detect + publish `/cam_NN/tag_observations` |
| host | `roboscan_nsl3130/scripts/multiview_bundle_node.py` | SOLVE — collect, bundle-adjust, write yml, (writeback) |
| edge | `roboscan_nsl3130/scripts/multiview_put_server.py` | WRITEBACK receiver — `/cam_NN/multiview/put` |
| edge | `roboscan_nsl3130/scripts/multiview_tf_node.py` | publish `stag_marker → frames`; mtime-poll re-publish |

Topics / services: `/fleet/calibrate` (Empty, host→edges), `/cam_NN/tag_observations`
(String JSON, latched, edge→host), `/cam_NN/multiview/put` (`PutWeight`, host→edge).

---

## 6. Running it

```bash
# 0. deploy the OBSERVE publisher to the edges
mgp && mcb

# 1. (dry-run) solve on the host and inspect, WITHOUT touching the edges
ros2 run roboscan_nsl3130 multiview_bundle_node.py --out-dir /tmp/mv_bundle
mtf                       # broadcast /fleet/calibrate; edges publish observations
# -> logs each camera position + rms, writes /tmp/mv_bundle/{serial}/multiview.yml

# 2. go live (push solved yml to the edges, TF auto-updates)
ros2 run roboscan_nsl3130 multiview_bundle_node.py --writeback

# offline math check (no ROS traffic)
python3 multiview_bundle_node.py --selftest
```

Useful flags: `--anchor-id <id>` (fix the origin tag), `--settle <s>` (collection window),
`--w-depth` / `--w-lidar` (depth confidence), `--huber` (robust scale).
