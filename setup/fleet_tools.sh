#!/usr/bin/env bash
# SiriusRobotDT — HOST commands and explicit edge operations.
# Source from ~/.bashrc: source ~/SiriusRobotDT/setup/env.bash
# Public workflow: msb, mcb, mvw, mtf, mtr, mvp, mtd, mdt (no launch flags).
# sensors start|stop|status controls the main service and its Edge agents.
# mdt bag [directory] replays offline in domain 42; mloc opens the saved TF editor.
# mrb records RGB/clouds/robot poses/TF; mba inspects a recording.
# Native module tools: mvw (cloud viewer), mvp (pose fusion), mtf (STag calibration),
# mcalpull (raw profile fetch), mtr (new marker-relative training).
# Explicit administration: msb restarts camera units; mcb shares/builds the project.
# SSH keys are preferred; mixed accounts and optional password fallback belong in
# ~/.edges.conf. Mirroring preserves calib_output/, weight/, and dataset/.
# Targets are online Edges 161..165, excluding this host; NSL_EDGES can select a subset.

# If NSL_FLEET_USER is empty, ssh uses ~/.ssh/config or the local username.
# Set NSL_FLEET_USER=sb only when every edge uses the same account.
SIRIUS_ROBOT_DT_ROOT="${SIRIUS_ROBOT_DT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
NSL_FLEET_USER="${NSL_FLEET_USER:-}"
NSL_FLEET_CONFIG="${NSL_FLEET_CONFIG:-$HOME/.edges.conf}"
NSL_FLEET_CONFIG_LEGACY="${NSL_FLEET_CONFIG_LEGACY:-$HOME/.nsl_fleet/edges.conf}"
NSL_LOCAL_WS="${NSL_LOCAL_WS:-${SIRIUS_ROBOT_DT_ROOT}/controller}"
NSL_REMOTE_PROJECT="${NSL_REMOTE_PROJECT:-~/SiriusRobotDT}"
NSL_EDGE_LO="${NSL_EDGE_LO:-161}"
NSL_EDGE_HI="${NSL_EDGE_HI:-165}"
# Never mirrored by mgp (per-edge state, huge binaries, or distributed elsewhere):
NSL_MIRROR_EXCLUDES=(--exclude='.git' --exclude='.claude/' --exclude='__pycache__/' --exclude='*.pyc'
                     --exclude='calib_output/' --exclude='weight/' --exclude='dataset/'
                     --exclude='build/' --exclude='install/' --exclude='log/' --exclude='state/' --exclude='config/calibration/')

_nsl_self_ip()    { ip -4 -o addr show 2>/dev/null | grep -oE '192\.168\.3\.[0-9]+' | head -1; }
_nsl_self_octet() { _nsl_self_ip | cut -d. -f4; }

_nsl_camera_settings() {
    local arguments=()
    [[ -z "${SIRIUS_CONFIG:-}" ]] || arguments+=(--config "$SIRIUS_CONFIG")
    python3 "${SIRIUS_ROBOT_DT_ROOT}/controller/src/NSL-3130AA-ROS2/setup/camera_network.py" \
        --inventory-file "${arguments[@]}"
}

_nsl_edge_address() {
    local octet="$1" address='' helper="${SIRIUS_ROBOT_DT_ROOT}/controller/src/NSL-3130AA-ROS2/setup/camera_network.py"
    if [[ -f "$helper" ]]; then
        address="$(python3 "$helper" --octet "$octet" --field host 2>/dev/null)" || return
    fi
    printf '%s\n' "${address:-192.168.3.$octet}"
}

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
    # Ignore stale credential entries after fleet renumbering. Discovery scans
    # the full current range independently of this personal account list.
    awk -v low="$NSL_EDGE_LO" -v high="$NSL_EDGE_HI" '
        /^[[:space:]]*#/ || /^[[:space:]]*$/ { next }
        $1 == "Edge" {
            edge = $2
            sub(/^192[.]168[.]3[.]/, "", edge)
            if (edge ~ /^[0-9]+$/ && (edge + 0) >= low && (edge + 0) <= high) print edge + 0
        }
    ' "$cfg" | sort -n -u
}

_nsl_edge_prop() {  # _nsl_edge_prop <octet> <key>
    local octet="$1" key="$2" cfg
    cfg="$(_nsl_config_file)"
    [[ -n "$cfg" ]] || return 1
    awk -v want_octet="$octet" -v want_ip="192.168.3.$octet" -v want_key="$key" '
        /^[[:space:]]*#/ || /^[[:space:]]*$/ { next }
        $1 == "Edge" {
            edge = $2
            short = edge
            sub(/^192[.]168[.]3[.]/, "", short)
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


_nsl_edge_project() {
    _nsl_edge_prop "$1" project || printf '%s\n' "$NSL_REMOTE_PROJECT"
}

# Online edges in [LO..HI] excluding this host (one octet per line). Override: NSL_EDGES.
_nsl_edges() {
    if [[ -n "${NSL_EDGES:-}" ]]; then printf '%s\n' $NSL_EDGES; return; fi
    # Credentials choose an account/password, not which agents the main unit
    # owns. Include key-authenticated devices omitted from ~/.edges.conf so a
    # service stop can reach every camera started by the same native scan.
    local self o; self=$(_nsl_self_octet)
    {
        for o in $(seq "$NSL_EDGE_LO" "$NSL_EDGE_HI"); do
            [[ "$o" == "$self" ]] && continue
            ( ping -c1 -W1 "$(_nsl_edge_address "$o")" >/dev/null 2>&1 && echo "$o" ) &
        done
        wait
    } | sort -n
}

_nsl_target() {  # _nsl_target <octet>
    local user address
    address="$(_nsl_edge_address "$1")" || return
    if [[ -n "${NSL_FLEET_USER:-}" ]]; then
        printf '%s@%s' "$NSL_FLEET_USER" "$address"
    elif user="$(_nsl_edge_user "$1")" && [[ -n "$user" ]]; then
        printf '%s@%s' "$user" "$address"
    else
        printf '%s' "$address"
    fi
}

_nsl_ssh() {  # _nsl_ssh <octet> <command-string>
    local octet="$1" cmd="$2" password limiter=()
    if [[ -n "${NSL_SSH_COMMAND_TIMEOUT:-}" ]]; then
        [[ "$NSL_SSH_COMMAND_TIMEOUT" =~ ^[1-9][0-9]*$ ]] || {
            echo 'NSL_SSH_COMMAND_TIMEOUT must be a positive integer.' >&2
            return 2
        }
        limiter=(timeout --signal=TERM --kill-after=2s "${NSL_SSH_COMMAND_TIMEOUT}s")
    fi
    password="$(_nsl_edge_password "$octet" || true)"
    if [[ -n "$password" ]]; then
        if ! command -v sshpass >/dev/null 2>&1; then
            echo "sshpass is required for password login to 192.168.3.$octet" >&2
            return 127
        fi
        SSHPASS="$password" "${limiter[@]}" sshpass -e ssh -o BatchMode=no -o ConnectTimeout=6 -o ServerAliveInterval=5 -o ServerAliveCountMax=2 "$(_nsl_target "$octet")" "$cmd"
    else
        "${limiter[@]}" ssh -o BatchMode=yes -o ConnectTimeout=6 -o ServerAliveInterval=5 -o ServerAliveCountMax=2 "$(_nsl_target "$octet")" "$cmd"
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
    sudo_password="$(_nsl_edge_sudo_password "$octet" || true)"
    quoted="$(printf '%q' "$cmd")"
    if [[ -n "$sudo_password" ]]; then
        printf '%s\n' "$sudo_password" | _nsl_ssh "$octet" "sudo -S -p '' bash -lc $quoted"
    else
        _nsl_ssh "$octet" "sudo -n bash -lc $quoted"
    fi
}

# Restart only each connected Edge camera agent; training listeners stay independent.
_nsl_restart() {
    local o u failed=false NSL_SSH_COMMAND_TIMEOUT=8 edges
    edges="$(_nsl_edges)"
    [[ -n "$edges" ]] || { echo '[msb] No connected Edge camera.' >&2; return 1; }
    for o in $edges; do
        u="$(_nsl_login_user "$o" 2>/dev/null || true)"
        if [[ -z "$u" ]]; then
            echo "  edge $o: LOGIN-FAIL"
            failed=true
            continue
        fi
        if NSL_SSH_COMMAND_TIMEOUT=40 _nsl_sudo "$o" "timeout --signal=TERM --kill-after=2s 30s systemctl restart nsl-edge-agent@${u}.service"; then
            echo "  edge $o: restarted (${u})"
        else
            echo "  edge $o: RESTART-FAIL (${u})"
            failed=true
        fi
    done
    [[ "$failed" != true ]]
}

# Restart connected camera agents only.
msb() {
    (($# == 0)) || { echo 'Usage: msb'; return 2; }
    echo '[msb] restarting connected Edge camera services.'
    _nsl_workflow invalidate restart
    _nsl_restart
}

# ── mgp : mirror host code to edges (rsync, not git) ────────────────────────
# Used instead of a full-src zip on purpose: a whole-src copy is ~1.1 GB and would
# DELETE each edge's own calib_output/ (269 MB, per-serial calibration incl. intrinsics)
# plus weight/ (332 MB) + dataset/ (235 MB). rsync --delete with those excluded gives a
# clean code mirror (removes stale/renamed files) while leaving per-edge state intact.
mgp() {
    local o pids=() target password remote_ws failed=false
    echo "[mgp] mirroring SiriusRobotDT -> online Edges; local calibration/weights/datasets stay preserved."
    for o in $(_nsl_edges); do
        target="$(_nsl_target "$o")"
        password="$(_nsl_edge_password "$o" || true)"
        remote_ws="$(_nsl_edge_project "$o")"
        (
            set -o pipefail
            (
            # Prepare the scoped project destination; unrelated remote src stays intact.
            _nsl_ssh "$o" "mkdir -p ${remote_ws}" || exit
            if [[ -n "$password" ]]; then
                if ! command -v sshpass >/dev/null 2>&1; then
                    echo "sshpass is required for password login to 192.168.3.$o"
                    exit 127
                fi
                SSHPASS="$password" rsync -az --delete -e 'sshpass -e ssh -o BatchMode=no -o ConnectTimeout=6' \
                    "${NSL_MIRROR_EXCLUDES[@]}" \
                    "${SIRIUS_ROBOT_DT_ROOT}/" "${target}:${remote_ws}/" \
                    || { echo 'rsync FAILED'; exit 1; }
            else
                rsync -az --delete -e 'ssh -o BatchMode=yes -o ConnectTimeout=6' \
                    "${NSL_MIRROR_EXCLUDES[@]}" \
                    "${SIRIUS_ROBOT_DT_ROOT}/" "${target}:${remote_ws}/" \
                    || { echo 'rsync FAILED'; exit 1; }
            fi
            echo synced
            ) 2>&1 | sed "s/^/  edge $o: /"
        ) &
        pids+=("$!")
    done
    for o in "${pids[@]}"; do wait "$o" || failed=true; done
    [[ "$failed" != true ]] || return 1
}

# ── mcb : share the whole project, then build each connected Edge ───────────
mcb() {
    (($# == 0)) || { echo 'Usage: mcb'; return 2; }
    local o pids=() edges remote_ws failed=false; edges=$(_nsl_edges)
    [[ -n "$edges" ]] || { echo '[mcb] No connected Edge to share/build.'; return 1; }
    NSL_EDGES="$edges" mgp || return
    _nsl_workflow invalidate build
    for o in $edges; do
        remote_ws="$(_nsl_edge_project "$o")"
        _nsl_ssh "$o" "
            source /opt/ros/humble/setup.bash
            cd ${remote_ws}/controller && colcon build --symlink-install >/tmp/mcb.log 2>&1 \
                && echo OK || { echo FAIL; tail -8 /tmp/mcb.log; exit 1; }" >"/tmp/mcb_$o.out" 2>&1 &
        pids+=("$!")
    done
    for o in "${pids[@]}"; do wait "$o" || failed=true; done
    for o in $edges; do
        echo "[mcb] Edge $o build:"; sed 's/^/  /' "/tmp/mcb_$o.out" 2>/dev/null; rm -f "/tmp/mcb_$o.out"
    done
    [[ "$failed" != true ]] || return 1
    echo '[mcb] Project shared and built. Camera services were not restarted.'
}

# Native state and PID guards belong to the camera module.
_nsl_workflow() { python3 "${SIRIUS_ROBOT_DT_ROOT}/controller/src/NSL-3130AA-ROS2/setup/workflow.py" "$@"; }

_nsl_share_pose_config() {
    local file="$1" relative
    relative="$(realpath --relative-to="$SIRIUS_ROBOT_DT_ROOT" "$file")" || return
    [[ "$relative" == controller/src/ros_humanpose/config/* ]] || { echo 'Prepared pose config must exist in the human module' >&2; return 2; }
    _nsl_share_project_file "$@"
}

_nsl_share_project_file() {
    local file="$1" relative hash octet project target password remote_file staged quoted_project quoted_relative
    shift
    (($# > 0)) || { echo 'Prepared pose config needs connected camera IDs.' >&2; return 2; }
    relative="$(python3 - "$SIRIUS_ROBOT_DT_ROOT" "$file" <<'PY'
from pathlib import Path
import sys
root, file = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
try:
    relative = file.relative_to(root)
except ValueError:
    raise SystemExit('Pose config must be inside SiriusRobotDT')
allowed = (str(relative) == 'config/config.yaml'
           or str(relative).startswith('controller/src/ros_humanpose/config/')
           or (str(relative).startswith('controller/src/NSL-3130AA-ROS2/calib_output/runtime/')
               and 'overrides' in relative.parts))
if not allowed or not file.is_file():
    raise SystemExit('Only public config, prepared pose config and runtime calibration overrides may be shared')
print(relative)
PY
)" || return
    hash="$(sha256sum "$file")"; hash="${hash%% *}"
    for octet in "$@"; do
        [[ "$octet" =~ ^16[1-5]$ ]] || { echo 'Invalid pose camera octet.' >&2; return 2; }
        project="$(_nsl_edge_project "$octet")"
        [[ "$project" =~ ^(~|/)?[A-Za-z0-9_./-]+$ && "$project" != *..* ]] || { echo 'Invalid remote project directory.' >&2; return 2; }
        printf -v quoted_project '%q' "$project"
        printf -v quoted_relative '%q' "$relative"
        remote_file="${project}/${relative}"
        staged="${remote_file}.incoming-${hash}"
        NSL_SSH_COMMAND_TIMEOUT=8 _nsl_ssh "$octet" "project=$quoted_project; [[ \$project != '~/'* ]] || project=\"\$HOME/\${project#'~/'}\"; install -d -- \"\$project/$(dirname "$relative")\"" || return
        target="$(_nsl_target "$octet")"
        password="$(_nsl_edge_password "$octet" || true)"
        if [[ -n "$password" ]]; then
            SSHPASS="$password" timeout --signal=TERM --kill-after=2s 25s sshpass -e scp \
                -o BatchMode=no -o ConnectTimeout=6 "$file" "${target}:${staged}" || return
        else
            timeout --signal=TERM --kill-after=2s 25s scp -o BatchMode=yes -o ConnectTimeout=6 \
                "$file" "${target}:${staged}" || return
        fi
        NSL_SSH_COMMAND_TIMEOUT=8 _nsl_ssh "$octet" "set -e; project=$quoted_project; [[ \$project != '~/'* ]] || project=\"\$HOME/\${project#'~/'}\"; relative=$quoted_relative; staged=\"\$project/\$relative.incoming-$hash\"; actual=\"\$(sha256sum \"\$staged\")\"; [[ \${actual%% *} == $hash ]] || exit 1; mv -- \"\$staged\" \"\$project/\$relative\"" || return
        echo "[share] Edge $octet $(basename "$file") SHA verified."
    done
}

# New single/multi-view training using the saved second alignment.
mtr() {
    (($# == 0)) || { echo 'Usage: mtr'; return 2; }
    local status_code
    if python3 "${SIRIUS_ROBOT_DT_ROOT}/controller/src/ros_humanpose/setup/distributed_training.py" --attach-running; then
        return 0
    else
        status_code=$?
        [[ "$status_code" != 130 ]] || return 0
        [[ "$status_code" == 3 ]] || return "$status_code"
    fi
    _nsl_workflow pose
}

# ── mtf : multi tf / anchor-camera re-calibration ───────────────────────────
mcalpull() {
    if [[ "${1:-}" == -h || "${1:-}" == --help || -z "${1:-}" ]]; then
        echo 'Usage: mcalpull 161 [remote_profile_directory]'
        echo 'SCP the configured camera profile, validate YAML, and refresh profile references.'
        return 0
    fi
    local octet="${1#cam_}" user password project
    [[ "$octet" =~ ^[0-9]+$ ]] || { echo '[mcalpull] camera must be an octet or cam_<octet>.' >&2; return 2; }
    user="${NSL_FLEET_USER:-$(_nsl_edge_user "$octet" 2>/dev/null || true)}"
    password="$(_nsl_edge_password "$octet" 2>/dev/null || true)"
    project="$(_nsl_edge_project "$octet")"
    SSHPASS="$password" python3 "${SIRIUS_ROBOT_DT_ROOT}/controller/src/NSL-3130AA-ROS2/setup/camera_calibration_pull.py" "$@" \
        --config "$(_nsl_camera_settings)" \
        --remote-project "$project" --user "$user"
}

mtf() {
    (($# == 0)) || { echo 'Usage: mtf'; return 2; }
    _nsl_workflow require mvw || return
    local o u out observe nosave pub_out pub_status successful=() failed=false edges NSL_SSH_COMMAND_TIMEOUT=8
    edges="$(_nsl_edges)"
    [[ -n "$edges" ]] || { echo '[mtf] No connected Edge camera.' >&2; return 1; }
    local calib_started; calib_started="$(date +%s)"
    local wait_sec="${NSL_CALIB_WAIT_SEC:-8}"
    local pub_times="${NSL_CALIB_PUB_TIMES:-5}"
    local pub_rate="${NSL_CALIB_PUB_RATE:-2}"
    local pub_timeout="${NSL_CALIB_PUB_TIMEOUT_SEC:-15}"
    local log_since="@${calib_started}"
    echo "[mtf] broadcasting /fleet/calibrate (${pub_times} pulses @ ${pub_rate} Hz) → edges re-anchor cameras to stag_marker"
    # A finite publish still waits indefinitely when DDS finds no subscribers.
    if pub_out="$(PYTHONUNBUFFERED=1 timeout --signal=INT --kill-after=2s "${pub_timeout}s" \
        ros2 topic pub -t "$pub_times" -r "$pub_rate" /fleet/calibrate std_msgs/msg/Empty "{}" 2>&1)"; then
        :
    else
        pub_status=$?
        if [[ "$pub_status" == 124 || "$pub_status" == 137 ]]; then
            echo "[mtf] PUBLISH-TIMEOUT after ${pub_timeout}s; check DDS discovery and /fleet/calibrate subscribers." >&2
        else
            echo "[mtf] PUBLISH-FAIL (exit ${pub_status})." >&2
        fi
        [[ -z "$pub_out" ]] || printf '%s\n' "$pub_out" >&2
        echo "[mtf] Reload this terminal with: source ~/.bashrc" >&2
        return "$pub_status"
    fi
    echo "[mtf] waiting ${wait_sec}s for edges to calibrate, then reading each edge's result…"
    sleep "$wait_sec"
    for o in $edges; do
        u="$(_nsl_login_user "$o" 2>/dev/null || true)"
        if [[ -z "$u" ]]; then
            echo "  edge $o: LOGIN-FAIL"
            failed=true
            continue
        fi
        local deadline=$((SECONDS + ${NSL_CALIB_WRITEBACK_WAIT_SEC:-30})) journal_ok=true
        while :; do
            if ! out="$(NSL_SSH_COMMAND_TIMEOUT=12 _nsl_sudo "$o" "journalctl -u nsl-edge-agent@${u}.service --since '${log_since}' --no-pager 2>/dev/null")"; then
                journal_ok=false
                break
            fi
            if printf '%s\n' "$out" | grep -qi '\[observe\] published' && \
                    printf '%s\n' "$out" | grep -qF '[multiview_put] complete, verified, installed'; then
                break
            fi
            (( SECONDS < deadline )) || break
            sleep 1
        done
        if [[ "$journal_ok" != true ]]; then
            echo "  edge $o: JOURNAL-FAIL (${u})"
            failed=true
            continue
        fi
        observe="$(printf '%s\n' "$out" | grep -i '\[observe\] published' | tail -1 || true)"
        if [[ -n "$observe" ]]; then
            if ! printf '%s\n' "$out" | grep -qF '[multiview_put] complete, verified, installed'; then
                echo "  edge $o: observations received, but completed solver writeback was not confirmed"
                failed=true
                continue
            fi
            # Pull the verified persisted result now instead of racing the main
            # controller's periodic profile import before mark-mtf checks freshness.
            if ! mcalpull "$o"; then
                echo "  edge $o: persisted calibration import failed"
                failed=true
                continue
            fi
            printf '  edge %s: %s\n' "$o" "$(
                printf '%s\n' "$observe" \
                    | sed -E 's/.*\[observe\] published ([0-9]+) tag obs on ([^ ]+) \(ref id ([0-9]+)\).*/OK: \1 tags -> \2 (ref id \3)/'
            )"
            successful+=("$o")
            continue
        fi
        nosave="$(printf '%s\n' "$out" | grep -iE 'needs >=|not saving' | tail -1 || true)"
        if [[ -n "$nosave" ]]; then
            printf '  edge %s: %s\n' "$o" "$(
                printf '%s\n' "$nosave" \
                    | sed -E 's/.*multiview_calib[^:]*: //'
            )"
            failed=true
            continue
        fi
        if printf '%s\n' "$out" | grep -qi 'trigger received'; then
            local progress
            progress="$(printf '%s\n' "$out" | grep -F '[collect]' | tail -1 || true)"
            if [[ -n "$progress" ]]; then
                printf '  edge %s: TRIGGERED; %s\n' "$o" "${progress#*\[collect\] }"
            else
                echo "  edge $o: TRIGGERED, but no tag observation yet"
            fi
        else
            echo "  edge $o: NO-TRIGGER"
        fi
        failed=true
    done
    echo "[mtf] importing only persisted calibration files written since this request..."
    if ! python3 -m sirius_config.sync \
        --config "$(_nsl_camera_settings)" \
        --calibration-root "${NSL_CALIB_DIR:-${SIRIUS_ROBOT_DT_ROOT}/controller/src/NSL-3130AA-ROS2/calib_output}" \
        --min-mtime "$calib_started"; then
        echo '[mtf] global config import failed; SDK/profile state and the report need review.' >&2
        return 1
    fi
    echo "[mtf] STag observations and completed SDK writeback are separate. Stale files are skipped; the writeback receiver syncs after its verified install."
    [[ "$failed" != true ]] || { echo '[mtf] Current anchor detection is incomplete; mtr remains unavailable.' >&2; return 1; }
    _nsl_workflow mark-mtf --since "$calib_started" "${successful[@]}"
}

# Record on the host; no SSH, camera restart, or calibration writeback.
mrb() {
    local transport="${NSL_ROSBAG_RGB_TRANSPORT:-compressed}" suffix='' output
    if [[ "${1:-}" == -h || "${1:-}" == --help ]]; then
        echo 'Usage: mrb [extra ros2 bag record options/topics]'
        echo 'One HOST bag: camera 161..165 JPEG RGB + RGB PointCloud + loading/unloading pose + TF. Stop/save: Ctrl+C.'
        echo 'Optional raw Image input: NSL_ROSBAG_RGB_TRANSPORT=raw mrb'
        return 0
    fi
    case "$transport" in raw|compressed) ;;
        *) echo '[mrb] NSL_ROSBAG_RGB_TRANSPORT must be raw or compressed.' >&2; return 2 ;;
    esac
    [[ "$transport" != compressed ]] || suffix='/compressed'
    output="$HOME/Desktop/ROSBAG/$(date +%Y-%m-%d_%H-%M-%S_%N)"
    mkdir -p "$HOME/Desktop/ROSBAG" || return
    echo "[mrb] HOST -> $output (RGB: $transport; Ctrl+C to stop/save)"
    local camera topics=()
    for camera in ${NSL_ROSBAG_CAMERAS:-161 162 163 164 165}; do
        [[ "$camera" =~ ^[0-9]+$ ]] || { echo '[mrb] NSL_ROSBAG_CAMERAS needs numeric camera IDs.' >&2; return 2; }
        topics+=("/cam_${camera}/camera/rgb/image_raw${suffix}" "/cam_${camera}/camera/point_cloud_rgb")
    done
    ros2 bag record -o "$output" "${topics[@]}" \
        /robot/loading/pose /robot/unloading/pose /tf /tf_static "$@"
}

# Managed camera/pose viewers reuse the existing module nodes.
mvw() { (($# == 0)) || { echo 'Usage: mvw'; return 2; }; _nsl_workflow view camera; }
mvp() { (($# == 0)) || { echo 'Usage: mvp'; return 2; }; _nsl_workflow view map_pose; }
mtd() {
    case "${1:-}" in
        '') _nsl_workflow mtd ;;
        auto_mode:=true) (($# == 1)) || return 2; _nsl_workflow mtd --auto ;;
        auto_mode:=false) (($# == 1)) || return 2; _nsl_workflow mtd ;;
        save) (($# == 1)) || { echo 'Usage: mtd [save]'; return 2; }; _nsl_workflow save-mtd ;;
        restore) (($# == 1)) || { echo 'Usage: mtd [restore]'; return 2; }
                 bash "${SIRIUS_ROBOT_DT_ROOT}/controller/src/robot_dt_bridge/localization/start_two_robots.sh" restore ;;
        *) echo 'Usage: mtd [save]'; return 2 ;;
    esac
}

# Saved localization TF + multiview clouds/poses + both robot models.
# Live sensors and replay use domain 42; stop the main systemd service before replay.
mdt() {
    local mode='live' arg bag_argument
    case "${1:-}" in
        ''|auto_mode:=true|auto_mode:=false|save|restore) mtd "$@"; return ;;
        -h|--help|help)
            cat <<'HELP'
Usage: mdt [save|restore] | mdt live | mdt bag [bag_directory]
  mdt                     # second alignment editor; save cameras, STag and robot bases
  mdt save                # save the open editor
  mdt live                # view the saved global placement
  sensors stop
  mdt bag                 # saved 90-second recording
  mdt bag /path/to/bag
  mloc                    # edit and save the stored TF
  sensors start
Live sensors and offline replay use domain 42. Replay sets local discovery automatically.
mtf -> mtr trains and shows measured camera/marker geometry independently of MDT.
mdt + save sets independent map placements; mvp publishes poses on the saved marker placement.
HELP
            return 0
            ;;
        bag|live) mode="$1"; shift ;;
    esac
    if [[ "$mode" == bag && -n "${1:-}" && "$1" != *:=* && "$1" != -* ]]; then
        bag_argument="bag:=$1"
        shift
        set -- "$bag_argument" "$@"
    fi
    for arg in "$@"; do
        if [[ "$arg" == source:=* ]]; then
            echo '[mdt] Use "mdt bag ..." or "mdt live ..." instead of source:=.' >&2
            return 2
        fi
    done
    if [[ "$mode" == bag ]]; then
        if systemctl is-active --quiet sensor_streaming.service; then
            echo '[mdt] Stop live streaming first: sensors stop' >&2
            return 1
        fi
        local selected_session
        selected_session="$(_nsl_workflow session-file)" || return
        if [[ -f "${SIRIUS_ROBOT_DT_ROOT}/config/config.yaml" ]] && python3 - "${SIRIUS_ROBOT_DT_ROOT}/config/config.yaml" <<'PY'
import sys
import yaml
data = yaml.safe_load(open(sys.argv[1])) or {}
raise SystemExit(0 if data.get('schema_version') == 2 and 'robots' in data and 'cameras' in data else 1)
PY
        then
            selected_session="${SIRIUS_ROBOT_DT_ROOT}/config/config.yaml"
        fi
        (
            export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
            # Offline replay needs no discovery-server service or device network profile.
            unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
            unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
            unset RMW_FASTRTPS_USE_QOS_FROM_XML CYCLONEDDS_URI
            exec ros2 launch sirius_localization mdt.launch.py source:=bag use_mvp:=false \
                "session_file:=$selected_session" "$@"
        )
    else
        (($# == 0)) || { echo 'Usage: mdt | mdt bag [bag_directory]'; return 2; }
        _nsl_workflow view camera_pose_robot
    fi
}

# Read-only recorded bag autopsy from its localization module.
mba() { python3 "${SIRIUS_ROBOT_DT_ROOT}/controller/src/robot_dt_bridge/localization/tools/bag_autopsy.py" "$@"; }

mloc() { bash "${SIRIUS_ROBOT_DT_ROOT}/controller/src/robot_dt_bridge/localization/start_two_robots.sh" "$@"; }

srbuild() { (cd "${SIRIUS_ROBOT_DT_ROOT}/controller" && colcon build --symlink-install "$@"); }

# One public control for the main controller and its Edge camera/inference units.
sensors() {
    case "${1:-}" in
        start|stop) sudo systemctl "$1" sensor_streaming.service ;;
        status) systemctl status sensor_streaming.service ;;
        *) echo 'Usage: sensors start|stop|status'; return 2 ;;
    esac
}

# ROOT launch frontends call these functions directly; sourced interactive aliases
# are intentionally absent from this subprocess entry point.
if [[ "${BASH_SOURCE[0]}" == "$0" && ${#BASH_SOURCE[@]} == 1 ]]; then
    if [[ "${1:-}" != --internal-command || $# -lt 2 ]]; then
        echo 'Use the SiriusRobotDT shell commands, or --internal-command <command>.' >&2
        exit 2
    fi
    shift
    command_name="$1"
    shift
    case "$command_name" in
        msb|mcb|mvw|mtf|mtr|mvp) source "${SIRIUS_ROBOT_DT_ROOT}/setup/env.bash" ;;
        mtd) case "${1:-}" in save|restore) ;; *) source "${SIRIUS_ROBOT_DT_ROOT}/setup/env.bash" ;; esac ;;
        mdt) [[ "${1:-}" == bag ]] || source "${SIRIUS_ROBOT_DT_ROOT}/setup/env.bash" ;;
    esac
    # env.bash sources public frontend functions; restore the native backend
    # definitions with alias expansion disabled before dispatching them.
    shopt -u expand_aliases
    source "${BASH_SOURCE[0]}"
    case "$command_name" in
        msb|mcb|mvw|mtf|mtr|mvp|mtd|mdt|mgp|mrb|mcalpull|sensors) "$command_name" "$@" ;;
        share-pose-config) _nsl_share_pose_config "$@" ;;
        share-runtime-file) _nsl_share_project_file "$@" ;;
        *) echo "Unknown internal command: $command_name" >&2; exit 2 ;;
    esac
fi
