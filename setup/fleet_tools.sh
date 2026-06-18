#!/usr/bin/env bash
# NSL fleet — HOST-side control of the edge machines.
# Source from ~/.bashrc:  source ~/colcon_ws/src/NSL-3130AA-ROS2/setup/fleet_tools.sh
# Requires host->edge SSH keys by default. Mixed edge users can be listed in
# ~/.edges.conf:
#
#   Edge 192.168.0.51
#     id sb
#     pw sb                # optional sshpass/sudo fallback; SSH keys are preferred
#     workspace ~/colcon_ws
#
#   Edge 192.168.0.53
#     id kitech
#     pw 1234
#     workspace ~/colcon_ws
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

# If NSL_FLEET_USER is empty, ssh uses ~/.ssh/config or the local username.
# Set NSL_FLEET_USER=sb only when every edge uses the same account.
NSL_FLEET_USER="${NSL_FLEET_USER:-}"
NSL_FLEET_CONFIG="${NSL_FLEET_CONFIG:-$HOME/.edges.conf}"
NSL_FLEET_CONFIG_LEGACY="${NSL_FLEET_CONFIG_LEGACY:-$HOME/.nsl_fleet/edges.conf}"
NSL_LOCAL_WS="${NSL_LOCAL_WS:-${NSL_WS:-$HOME/colcon_ws}}"
NSL_REMOTE_WS="${NSL_REMOTE_WS:-~/colcon_ws}"
NSL_EDGE_LO="${NSL_EDGE_LO:-51}"
NSL_EDGE_HI="${NSL_EDGE_HI:-60}"
# Never mirrored by mgp (per-edge state, huge binaries, or distributed elsewhere):
NSL_MIRROR_EXCLUDES=(--exclude='.git/' --exclude='.claude/' --exclude='__pycache__/' --exclude='*.pyc'
                     --exclude='calib_output/' --exclude='weight/' --exclude='dataset/'
                     --exclude='build/' --exclude='install/' --exclude='log/')

_nsl_self_ip()    { ip -4 -o addr show 2>/dev/null | grep -oE '192\.168\.0\.[0-9]+' | head -1; }
_nsl_self_octet() { _nsl_self_ip | cut -d. -f4; }

_nsl_config_file() {
    if [[ -f "${NSL_FLEET_CONFIG:-}" ]]; then
        printf '%s\n' "$NSL_FLEET_CONFIG"
    elif [[ -f "${NSL_FLEET_CONFIG_LEGACY:-}" ]]; then
        printf '%s\n' "$NSL_FLEET_CONFIG_LEGACY"
    else
        return 0
    fi
}

_nsl_config_edges() {
    local cfg; cfg="$(_nsl_config_file)"
    [[ -n "$cfg" ]] || return 0
    awk '
        /^[[:space:]]*#/ || /^[[:space:]]*$/ { next }
        $1 == "Edge" {
            edge = $2
            sub(/^192[.]168[.]0[.]/, "", edge)
            if (edge ~ /^[0-9]+$/) print edge
        }
    ' "$cfg" | sort -n -u
}

_nsl_edge_prop() {  # _nsl_edge_prop <octet> <key>
    local octet="$1" key="$2" cfg
    cfg="$(_nsl_config_file)"
    [[ -n "$cfg" ]] || return 1
    awk -v want_octet="$octet" -v want_ip="192.168.0.$octet" -v want_key="$key" '
        /^[[:space:]]*#/ || /^[[:space:]]*$/ { next }
        $1 == "Edge" {
            edge = $2
            short = edge
            sub(/^192[.]168[.]0[.]/, "", short)
            active = (edge == want_ip || short == want_octet)
            next
        }
        active && $1 == want_key {
            $1 = ""
            sub(/^[[:space:]]+/, "")
            print
            found = 1
            exit
        }
        END { if (!found) exit 1 }
    ' "$cfg"
}

_nsl_edge_user() {
    _nsl_edge_prop "$1" id || _nsl_edge_prop "$1" user
}

_nsl_edge_password() {
    _nsl_edge_prop "$1" pw || _nsl_edge_prop "$1" password
}

_nsl_edge_sudo_password() {
    _nsl_edge_prop "$1" sudo_pw || _nsl_edge_prop "$1" sudo_password || _nsl_edge_password "$1"
}

_nsl_edge_workspace() {
    _nsl_edge_prop "$1" workspace || printf '%s\n' "$NSL_REMOTE_WS"
}

# Online edges in [LO..HI] excluding this host (one octet per line). Override: NSL_EDGES.
_nsl_edges() {
    if [[ -n "${NSL_EDGES:-}" ]]; then printf '%s\n' $NSL_EDGES; return; fi
    local self o configured; self=$(_nsl_self_octet)
    configured="$(_nsl_config_edges)"
    if [[ -n "$configured" ]]; then
        {
            for o in $configured; do
                [[ "$o" == "$self" ]] && continue
                ( ping -c1 -W1 "192.168.0.$o" >/dev/null 2>&1 && echo "$o" ) &
            done
            wait
        } | sort -n
        return
    fi
    {
        for o in $(seq "$NSL_EDGE_LO" "$NSL_EDGE_HI"); do
            [[ "$o" == "$self" ]] && continue
            ( ping -c1 -W1 "192.168.0.$o" >/dev/null 2>&1 && echo "$o" ) &
        done
        wait
    } | sort -n
}

_nsl_target() {  # _nsl_target <octet>
    local user
    if [[ -n "${NSL_FLEET_USER:-}" ]]; then
        printf '%s@192.168.0.%s' "$NSL_FLEET_USER" "$1"
    elif user="$(_nsl_edge_user "$1")" && [[ -n "$user" ]]; then
        printf '%s@192.168.0.%s' "$user" "$1"
    else
        printf '192.168.0.%s' "$1"
    fi
}

_nsl_ssh() {  # _nsl_ssh <octet> <command-string>
    local octet="$1" cmd="$2" password
    password="$(_nsl_edge_password "$octet")"
    if [[ -n "$password" ]]; then
        if ! command -v sshpass >/dev/null 2>&1; then
            echo "sshpass is required for password login to 192.168.0.$octet" >&2
            return 127
        fi
        SSHPASS="$password" sshpass -e ssh -o BatchMode=no -o ConnectTimeout=6 "$(_nsl_target "$octet")" "$cmd"
    else
        ssh -o BatchMode=yes -o ConnectTimeout=6 "$(_nsl_target "$octet")" "$cmd"
    fi
}

_nsl_login_user() {
    local octet="$1" user
    if [[ -n "${NSL_FLEET_USER:-}" ]]; then
        printf '%s\n' "$NSL_FLEET_USER"
    elif user="$(_nsl_edge_user "$octet")" && [[ -n "$user" ]]; then
        printf '%s\n' "$user"
    else
        _nsl_ssh "$octet" 'id -un'
    fi
}

_nsl_sudo() {  # _nsl_sudo <octet> <remote-command>
    local octet="$1" cmd="$2" sudo_password quoted
    sudo_password="$(_nsl_edge_sudo_password "$octet")"
    quoted="$(printf '%q' "$cmd")"
    if [[ -n "$sudo_password" ]]; then
        printf '%s\n' "$sudo_password" | _nsl_ssh "$octet" "sudo -S -p '' bash -lc $quoted"
    else
        _nsl_ssh "$octet" "sudo -n bash -lc $quoted"
    fi
}

# internal: restart edge runtime services (kills stream/pose, reloads code+weight, re-pubs TF)
_nsl_restart() {
    local o u
    for o in $(_nsl_edges); do
        u="$(_nsl_login_user "$o" 2>/dev/null || true)"
        if [[ -z "$u" ]]; then
            echo "  edge $o: LOGIN-FAIL"
            continue
        fi
        if _nsl_sudo "$o" "systemctl restart nsl-edge-agent@${u}.service nsl-train-listener@${u}.service"; then
            echo "  edge $o: restarted (${u})"
        else
            echo "  edge $o: RESTART-FAIL (${u})"
        fi
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
    local o pids=() target password remote_ws
    echo "[mgp] mirroring host code -> online edges (rsync --delete; keeps each edge's calib_output/weight/dataset)…"
    { set +m; } 2>/dev/null
    for o in $(_nsl_edges); do
        target="$(_nsl_target "$o")"
        password="$(_nsl_edge_password "$o")"
        remote_ws="$(_nsl_edge_workspace "$o")"
        (
            if [[ -n "$password" ]]; then
                if ! command -v sshpass >/dev/null 2>&1; then
                    echo "sshpass is required for password login to 192.168.0.$o"
                    exit 127
                fi
                SSHPASS="$password" rsync -az --delete -e 'sshpass -e ssh -o BatchMode=no -o ConnectTimeout=6' \
                    "${NSL_MIRROR_EXCLUDES[@]}" \
                    "${NSL_LOCAL_WS}/src/" "${target}:${remote_ws}/src/" \
                    && echo synced || echo "rsync FAILED"
            else
                rsync -az --delete -e 'ssh -o BatchMode=yes -o ConnectTimeout=6' \
                    "${NSL_MIRROR_EXCLUDES[@]}" \
                    "${NSL_LOCAL_WS}/src/" "${target}:${remote_ws}/src/" \
                    && echo synced || echo "rsync FAILED"
            fi
        ) 2>&1 | sed "s/^/  edge $o: /" &
        pids+=("$!")
    done
    wait "${pids[@]}" 2>/dev/null
    { set -m; } 2>/dev/null
    echo "[mgp] done -> run  mcb  to rebuild + restart on the edges."
}

# ── mcb : multi colcon build + restart ──────────────────────────────────────
mcb() {
    local o pids=() edges remote_ws; edges=$(_nsl_edges)
    { set +m; } 2>/dev/null
    for o in $edges; do
        remote_ws="$(_nsl_edge_workspace "$o")"
        _nsl_ssh "$o" "
            source /opt/ros/humble/setup.bash
            cd ${remote_ws} && colcon build --symlink-install >/tmp/mcb.log 2>&1 \
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
    local o u out wait_sec="${NSL_CALIB_WAIT_SEC:-5}"
    echo "[mtf] broadcasting /fleet/calibrate → edges re-anchor cameras to stag_marker"
    ros2 topic pub --once /fleet/calibrate std_msgs/msg/Empty "{}" >/dev/null 2>&1
    echo "[mtf] waiting ${wait_sec}s for edges to calibrate, then reading each edge's result…"
    sleep "$wait_sec"
    for o in $(_nsl_edges); do
        u="$(_nsl_login_user "$o" 2>/dev/null || true)"
        if [[ -z "$u" ]]; then
            echo "  edge $o: LOGIN-FAIL"
            continue
        fi
        if ! out="$(_nsl_sudo "$o" "journalctl -u nsl-edge-agent@${u}.service --since '25 sec ago' --no-pager 2>/dev/null")"; then
            echo "  edge $o: JOURNAL-FAIL (${u})"
            continue
        fi
        printf '%s\n' "$out" \
            | grep -iE 'multiview.yml|tags, ref|needs >=|not saving|reproj' \
            | tail -2 \
            | sed 's#.*roboscan_publish_node.*##' \
            | grep -v '^$' \
            | sed "s/^/  edge $o: /"
    done
    echo "[mtf] TF auto-re-publishes (/tf_static) within ~1-2 s → host RViz re-anchors live."
    echo "      An edge that printed 'needs >= N views / not saving' could NOT see its STag markers"
    echo "      (a restart will NOT help that — make the markers visible to that camera and re-run mtf)."
}

# ── viewers ─────────────────────────────────────────────────────────────────
mvw() { ros2 launch roboscan_nsl3130 multiview.launch.py "$@"; }     # RViz viewer (clouds/RGB/TF)
mvp() { ros2 launch ros_humanpose  multiview_pose.launch.py "$@"; }  # multi-view fusion runtime
