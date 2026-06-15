#!/usr/bin/env bash
# NSL fleet — HOST-side control of the edge machines (51/52/53).
# Source this from ~/.bashrc:  source ~/colcon_ws/src/NSL-3130AA-ROS2/setup/fleet_tools.sh
# Requires host->edge SSH keys (already installed). Edits nothing in git on its own;
# `mgp` only mirrors the edges to the HOST's already-committed state.
#
#   mgp     multi git pull   — each edge hard-syncs both repos to THIS host's commit
#   mcb     multi colcon build — `cb` on every edge (colcon build --symlink-install)
#   mcb -r  same, then restart each edge's services (nsl-edge-agent + nsl-train-listener)
#   mtr     multi training   — host drives mode:=all (edges single-view + host multiview)
#   mtf     multi tf/anchor  — broadcast /fleet/calibrate (re-anchor cameras to stag_marker)
#   mvw     multiview viewer — ros2 launch roboscan_nsl3130 multiview.launch.py
#   mrs     multi restart    — restart edge services only (no build)

NSL_EDGES="${NSL_EDGES:-51 52 53}"
NSL_HOST_IP="${NSL_HOST_IP:-192.168.0.61}"
NSL_WS="${NSL_WS:-$HOME/colcon_ws}"
NSL_REPOS="${NSL_REPOS:-NSL-3130AA-ROS2 ros_humanpose}"

_nsl_ssh() {  # _nsl_ssh <octet> <single-command-string>
    ssh -o BatchMode=yes -o ConnectTimeout=6 "sb@192.168.0.$1" "$2"
}

# ── mgp : multi git pull ────────────────────────────────────────────────────
# Each edge fetches from THIS host and hard-resets to the host's current branch,
# so the fleet mirrors the host's committed code. (Run after you commit on the host.)
mgp() {
    local o r br url remote_cmd
    for o in $NSL_EDGES; do
        echo "===== edge $o ====="
        for r in $NSL_REPOS; do
            br=$(git -C "$NSL_WS/src/$r" rev-parse --abbrev-ref HEAD 2>/dev/null) || { echo "  $r: not a host repo"; continue; }
            url="sb@${NSL_HOST_IP}:${NSL_WS}/src/${r}"
            remote_cmd="
                cd ~/colcon_ws/src/$r 2>/dev/null || { echo '  $r: missing on edge'; exit 0; }
                git remote get-url host >/dev/null 2>&1 || git remote add host '$url'
                git remote set-url host '$url'
                if git fetch -q host $br && git reset --hard -q FETCH_HEAD; then
                    echo \"  $r: -> \$(git rev-parse --short HEAD) [$br]\"
                else
                    echo \"  $r: FETCH/RESET FAILED\"
                fi"
            _nsl_ssh "$o" "$remote_cmd"
        done
    done
}

# ── mcb : multi colcon build ────────────────────────────────────────────────
mcb() {
    local o pids=()
    { set +m; } 2>/dev/null            # quiet the "[1] Done ..." job-control noise
    for o in $NSL_EDGES; do
        _nsl_ssh "$o" "
            source /opt/ros/humble/setup.bash
            cd ~/colcon_ws && colcon build --symlink-install >/tmp/mcb.log 2>&1 \
                && echo OK || { echo FAIL; tail -8 /tmp/mcb.log; }" >"/tmp/mcb_$o.out" 2>&1 &
        pids+=("$!")
    done
    wait "${pids[@]}" 2>/dev/null
    { set -m; } 2>/dev/null
    for o in $NSL_EDGES; do
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

# ── mrs : multi restart edge services (no build) ────────────────────────────
mrs() {
    local o
    echo "[mrs] restarting nsl-edge-agent + nsl-train-listener on each edge…"
    for o in $NSL_EDGES; do
        _nsl_ssh "$o" "sudo systemctl restart nsl-edge-agent@sb.service nsl-train-listener@sb.service \
            && echo OK || echo FAIL" | sed "s/^/  edge $o: /"
    done
}

# ── mvw : multiview viewer (host RViz) ──────────────────────────────────────
mvw() {
    ros2 launch roboscan_nsl3130 multiview.launch.py "$@"
}

# ── mtr : multi training (host-driven, no SSH) ──────────────────────────────
mtr() {
    ros2 launch ros_humanpose train.launch.py mode:=all "$@"
}

# ── mtf : multi tf / anchor-camera re-calibration ───────────────────────────
mtf() {
    echo "[mtf] broadcasting /fleet/calibrate — edges re-anchor their cameras to stag_marker"
    ros2 topic pub --once /fleet/calibrate std_msgs/msg/Empty "{}"
}
