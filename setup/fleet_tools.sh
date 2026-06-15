#!/usr/bin/env bash
# NSL fleet — HOST-side control of the edge machines.
# Source from ~/.bashrc:  source ~/colcon_ws/src/NSL-3130AA-ROS2/setup/fleet_tools.sh
# Requires host->edge SSH keys (installed by setup_fleet_edge.bash).
#
#   mgp         multi git pull   — online edges hard-sync both repos to THIS host's commit
#   mcb [-r]    multi colcon build (optionally restart edge services after)
#   mrs         multi restart    — restart edge services (nsl-edge-agent + nsl-train-listener)
#   mtr         multi training   — mode:=all. Edge single-view train RESTARTS the edge runtime
#                                  (new weight loaded) + host multi-view train. Host reflects live.
#   mtf [--no-restart]
#               multi tf/anchor  — broadcast /fleet/calibrate; multiview_tf re-publishes /tf_static
#                                  so the host RViz re-anchors LIVE (no restart needed for TF).
#                                  By default also restarts streaming for a clean re-init.
#   mvw         multiview viewer — ros2 launch roboscan_nsl3130 multiview.launch.py
#   mvp         multiview fusion — ros2 launch ros_humanpose multiview_pose.launch.py
#                                  (run this to USE the multi-view weight; hot-reloads on retrain)
#
# Targets = whichever edges in NSL_EDGE_LO..HI (default 51..60) are ONLINE right now; the host
# finds its own octet so it never targets itself. Override with NSL_EDGES="51 52".

NSL_FLEET_USER="${NSL_FLEET_USER:-sb}"
NSL_WS="${NSL_WS:-$HOME/colcon_ws}"
NSL_REPOS="${NSL_REPOS:-NSL-3130AA-ROS2 ros_humanpose}"
NSL_EDGE_LO="${NSL_EDGE_LO:-51}"
NSL_EDGE_HI="${NSL_EDGE_HI:-60}"

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

# ── mgp : multi git pull ────────────────────────────────────────────────────
mgp() {
    local host_ip o r br url; host_ip=$(_nsl_self_ip)
    for o in $(_nsl_edges); do
        echo "===== edge $o ====="
        for r in $NSL_REPOS; do
            br=$(git -C "$NSL_WS/src/$r" rev-parse --abbrev-ref HEAD 2>/dev/null) \
                || { echo "  $r: not a host repo"; continue; }
            url="${NSL_FLEET_USER}@${host_ip}:${NSL_WS}/src/${r}"
            _nsl_ssh "$o" "
                cd ~/colcon_ws/src/$r 2>/dev/null || { echo '  $r: missing on edge'; exit 0; }
                git remote get-url host >/dev/null 2>&1 || git remote add host '$url'
                git remote set-url host '$url'
                if git fetch -q host $br && git reset --hard -q FETCH_HEAD; then
                    echo \"  $r: -> \$(git rev-parse --short HEAD) [$br]\"
                else echo \"  $r: FETCH/RESET FAILED\"; fi"
        done
    done
}

# ── mcb : multi colcon build (mcb -r also restarts services) ────────────────
mcb() {
    local o pids=() edges; edges=$(_nsl_edges)
    { set +m; } 2>/dev/null            # quiet "[1] Done ..." job-control noise
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
        echo "===== edge $o build ====="
        sed 's/^/  /' "/tmp/mcb_$o.out" 2>/dev/null
        rm -f "/tmp/mcb_$o.out"
    done
    if [[ "${1:-}" == "-r" || "${1:-}" == "restart" ]]; then
        mrs
    else
        echo "[mcb] done. (add  -r  to also restart edge services, or run  mrs )"
    fi
}

# ── mrs : multi restart edge services ───────────────────────────────────────
mrs() {
    local o u="${NSL_FLEET_USER}"
    echo "[mrs] restarting nsl-edge-agent + nsl-train-listener on online edges…"
    for o in $(_nsl_edges); do
        _nsl_ssh "$o" "sudo systemctl restart nsl-edge-agent@${u}.service nsl-train-listener@${u}.service \
            && echo OK || echo FAIL" | sed "s/^/  edge $o: /"
    done
}

# ── mtr : multi training (mode:=all) ────────────────────────────────────────
# Edge single-view training restarts the edge runtime (restart_runtime:=true) so the new
# weight loads; host multi-view training writes multiview_pose.pkl (mvp hot-reloads it).
# The host RViz reflects live — no extra restart needed.
mtr() {
    ros2 launch ros_humanpose train.launch.py mode:=all "$@"
}

# ── mtf : multi tf / anchor-camera re-calibration ───────────────────────────
# /tf_static is auto-re-published by multiview_tf_node on calib change, so the host RViz
# re-anchors LIVE without a restart. By default we ALSO restart streaming afterwards for a
# fully clean re-init (pass --no-restart to skip and rely on the live TF update only).
mtf() {
    local wait_sec="${NSL_CALIB_WAIT_SEC:-7}"
    echo "[mtf] /fleet/calibrate → edges re-anchor cameras to stag_marker (multiview_tf re-publishes /tf_static live)"
    ros2 topic pub --once /fleet/calibrate std_msgs/msg/Empty "{}"
    if [[ "${1:-}" == "--no-restart" ]]; then
        echo "[mtf] TF re-published live; skipping restart (--no-restart)."
        return 0
    fi
    echo "[mtf] letting calibration finish (${wait_sec}s) then restarting streaming for a clean re-init…"
    sleep "$wait_sec"
    mrs
}

# ── viewers ─────────────────────────────────────────────────────────────────
mvw() { ros2 launch roboscan_nsl3130 multiview.launch.py "$@"; }   # RViz viewer (clouds/RGB/TF)
mvp() { ros2 launch ros_humanpose  multiview_pose.launch.py "$@"; } # multi-view fusion runtime
