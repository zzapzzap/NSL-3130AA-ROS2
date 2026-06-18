#!/usr/bin/env bash
# NSL fleet — HOST-side control of the edge machines.
# Source from ~/.bashrc:  source ~/colcon_ws/src/NSL-3130AA-ROS2/setup/fleet_tools.sh
# Requires host->edge SSH keys (installed by setup_fleet_edge.bash).
#
#   mgp        multi push   — mirror host code to online edges (rsync --delete; NOT git).
#                            PRESERVES each edge's own calib_output/ + weight/ + dataset/
#                            (those are per-edge / huge / distributed separately).
#   mcb        multi build+restart — colcon build --symlink-install on each edge, THEN restart
#                            its services (kills running stream/pose → reloads code + weights).
#   msb        multi service restart — sudo systemctl restart EVERY edge's services, NO rebuild.
#                            Fast "reload all" for config/calib/weight changes (e.g. a re-written
#                            calib_output/{serial}/extrinsic.yml). Same restart step as mcb, minus
#                            the colcon build. (Edges only — the host mvw/solver is interactive.)
#   mtr        multi training — mode:=all. Edge single-view train restarts the edge runtime
#                            (new weight) + host multi-view train. (run `mvp` to use MV weight.)
#   mtf        multi tf/anchor — broadcast /fleet/calibrate; multiview_tf re-publishes /tf_static
#                            within ~1-2 s (host RViz re-anchors LIVE, no restart). Reports each
#                            edge's calib result — if it says "not saving", the STag markers were
#                            not visible to that camera (that, not a restart, is the real fix).
#   mvw        multiview viewer — ros2 launch roboscan_nsl3130 multiview.launch.py
#   mvp        multiview pose   — ros2 launch ros_humanpose multiview_pose.launch.py (fusion)
#
# Typical update flow:  mgp  →  mcb            (push code, rebuild+restart)
# Targets = whichever edges in NSL_EDGE_LO..HI (default 51..60) are ONLINE now; the host finds
# its own octet so it never targets itself. Override with NSL_EDGES="51 52".

NSL_FLEET_USER="${NSL_FLEET_USER:-sb}"
NSL_WS="${NSL_WS:-$HOME/colcon_ws}"
NSL_EDGE_LO="${NSL_EDGE_LO:-51}"
NSL_EDGE_HI="${NSL_EDGE_HI:-60}"
# Never mirrored by mgp (per-edge state, huge binaries, or distributed elsewhere):
NSL_MIRROR_EXCLUDES=(--exclude='.git/' --exclude='.claude/' --exclude='__pycache__/' --exclude='*.pyc'
                     --exclude='calib_output/' --exclude='weight/' --exclude='dataset/'
                     --exclude='build/' --exclude='install/' --exclude='log/')

_nsl_self_ip()    { ip -4 -o addr show 2>/dev/null | grep -oE '192\.168\.0\.[0-9]+' | head -1; }
_nsl_self_octet() { _nsl_self_ip | cut -d. -f4; }

# Online edges in [LO..HI] excluding this host (one octet per line). Override: NSL_EDGES.
_nsl_edges() {
    if [[ -n "${NSL_EDGES:-}" ]]; then printf '%s\n' $NSL_EDGES; return; fi
    local self o; self=$(_nsl_self_octet)
    {
        for o in $(seq "$NSL_EDGE_LO" "$NSL_EDGE_HI"); do
            [[ "$o" == "$self" ]] && continue
            ( ping -c1 -W1 "192.168.0.$o" >/dev/null 2>&1 && echo "$o" ) &
        done
        wait
    } | sort -n
}

_nsl_ssh() {  # _nsl_ssh <octet> <command-string>
    ssh -o BatchMode=yes -o ConnectTimeout=6 "${NSL_FLEET_USER}@192.168.0.$1" "$2"
}

# internal: restart edge runtime services (kills stream/pose, reloads code+weight, re-pubs TF)
_nsl_restart() {
    local o u="${NSL_FLEET_USER}"
    for o in $(_nsl_edges); do
        _nsl_ssh "$o" "sudo systemctl restart nsl-edge-agent@${u}.service nsl-train-listener@${u}.service \
            && echo restarted || echo RESTART-FAIL" | sed "s/^/  edge $o: /"
    done
}

# ── msb : multi service restart (no rebuild) — fast reload of calib/weights/code ──
msb() {
    echo "[msb] restarting all edge services (reload code/calib/weights, re-publish TF; no rebuild)…"
    _nsl_restart
}

# ── mgp : mirror host code to edges (rsync, not git) ────────────────────────
# Used instead of a full-src zip on purpose: a whole-src copy is ~1.1 GB and would
# DELETE each edge's own calib_output/ (269 MB, per-serial calibration incl. intrinsics)
# plus weight/ (332 MB) + dataset/ (235 MB). rsync --delete with those excluded gives a
# clean code mirror (removes stale/renamed files) while leaving per-edge state intact.
mgp() {
    local o pids=()
    echo "[mgp] mirroring host code -> online edges (rsync --delete; keeps each edge's calib_output/weight/dataset)…"
    { set +m; } 2>/dev/null
    for o in $(_nsl_edges); do
        (
            rsync -az --delete -e 'ssh -o BatchMode=yes -o ConnectTimeout=6' \
                "${NSL_MIRROR_EXCLUDES[@]}" \
                "${NSL_WS}/src/" "${NSL_FLEET_USER}@192.168.0.$o:${NSL_WS}/src/" \
                && echo synced || echo "rsync FAILED"
        ) 2>&1 | sed "s/^/  edge $o: /" &
        pids+=("$!")
    done
    wait "${pids[@]}" 2>/dev/null
    { set -m; } 2>/dev/null
    echo "[mgp] done -> run  mcb  to rebuild + restart on the edges."
}

# ── mcb : multi colcon build + restart ──────────────────────────────────────
mcb() {
    local o pids=() edges; edges=$(_nsl_edges)
    { set +m; } 2>/dev/null
    for o in $edges; do
        _nsl_ssh "$o" "
            source /opt/ros/humble/setup.bash
            cd ~/colcon_ws && colcon build --symlink-install >/tmp/mcb.log 2>&1 \
                && echo OK || { echo FAIL; tail -8 /tmp/mcb.log; }" >"/tmp/mcb_$o.out" 2>&1 &
        pids+=("$!")
    done
    wait "${pids[@]}" 2>/dev/null
    { set -m; } 2>/dev/null
    for o in $edges; do
        echo "===== edge $o build ====="; sed 's/^/  /' "/tmp/mcb_$o.out" 2>/dev/null; rm -f "/tmp/mcb_$o.out"
    done
    echo "[mcb] restarting edge services (kills running stream/pose → reload code + weights, re-publish TF)…"
    _nsl_restart
}

# ── mtr : multi training (mode:=all) ────────────────────────────────────────
mtr() {
    ros2 launch ros_humanpose train.launch.py mode:=all "$@"
}

# ── mtf : multi tf / anchor-camera re-calibration ───────────────────────────
mtf() {
    local o u="${NSL_FLEET_USER}" wait_sec="${NSL_CALIB_WAIT_SEC:-5}"
    echo "[mtf] broadcasting /fleet/calibrate → edges re-anchor cameras to stag_marker"
    ros2 topic pub --once /fleet/calibrate std_msgs/msg/Empty "{}" >/dev/null 2>&1
    echo "[mtf] waiting ${wait_sec}s for edges to calibrate, then reading each edge's result…"
    sleep "$wait_sec"
    for o in $(_nsl_edges); do
        _nsl_ssh "$o" "sudo journalctl -u nsl-edge-agent@${u}.service --since '25 sec ago' --no-pager 2>/dev/null \
            | grep -iE 'multiview.yml|tags, ref|needs >=|not saving|reproj' | tail -2 \
            | sed 's#.*roboscan_publish_node.*##' || true" | grep -v '^$' | sed "s/^/  edge $o: /"
    done
    echo "[mtf] TF auto-re-publishes (/tf_static) within ~1-2 s → host RViz re-anchors live."
    echo "      An edge that printed 'needs >= N views / not saving' could NOT see its STag markers"
    echo "      (a restart will NOT help that — make the markers visible to that camera and re-run mtf)."
}

# ── viewers ─────────────────────────────────────────────────────────────────
mvw() { ros2 launch roboscan_nsl3130 multiview.launch.py "$@"; }     # RViz viewer (clouds/RGB/TF)
mvp() { ros2 launch ros_humanpose  multiview_pose.launch.py "$@"; }  # multi-view fusion runtime
