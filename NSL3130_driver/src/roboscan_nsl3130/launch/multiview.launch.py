#!/usr/bin/env python3
"""
Single multiview entry point (viewer + calibration in one launch file).

  ros2 launch roboscan_nsl3130 multiview.launch.py
      → VIEWER: RViz with every CONNECTED camera + tag anchored under the shared
        `stag_marker` frame. The layout is generated dynamically from the cameras
        that are actually publishing a colour cloud. Override with cameras:=56,57.
        Compressed RGB image panels are on by default; disable them with
        show_rgb_images:=false when only the colour clouds are needed.

  ros2 launch roboscan_nsl3130 multiview.launch.py calibration:=True
      → CALIBRATION only (headless one-touch, NO RViz): collect ~10 STag views
        (num_frames) then median-average the pose (id 7 = 0.32 m REFERENCE/origin,
        other ids = 0.19 m auxiliaries; each tag gets a LiDAR-RANSAC depth refine)
        and save calib_output/{serial}/multiview.yml. ~couple of seconds.
      add use_gui:=true to watch detection live ([s]save [r]reset [q]quit);
      duration:=N for a time-based window instead of a fixed count.

Viewer layout choices: the Displays (topic/status) panel sits on TOP of the left
column; optional per-camera RGB image panels can be stacked below it (Time at the
bottom). The Views panel is omitted (3D orbit navigation still works), background
unified to 48;48;48, Fixed Frame is always the shared `stag_marker`. The dock
arrangement is emitted as a QMainWindow State blob by gen_multiview_rviz_layout.py.
"""

import glob
import os
import re
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


REFERENCE_FRAME = 'stag_marker'

# Distinct cloud colours, cycled per camera.
PALETTE = ['255; 100; 100', '100; 255; 100', '100; 100; 255', '255; 255; 100',
           '255; 100; 255', '100; 255; 255', '255; 180; 100', '180; 120; 255']


def _is_true(context, arg):
    return LaunchConfiguration(arg).perform(context).strip().lower() in ('true', '1', 'yes')


def _ws_root():
    pkg_share = get_package_share_directory('roboscan_nsl3130')
    return os.path.normpath(os.path.join(pkg_share, '..', '..', '..', '..'))


def _scripts_dir():
    return os.path.join(_ws_root(), 'src', 'NSL-3130AA-ROS2', 'NSL3130_driver',
                        'src', 'roboscan_nsl3130', 'scripts')


def _calib_dir():
    return os.environ.get(
        'NSL_CALIB_DIR', os.path.join(_ws_root(), 'src', 'NSL-3130AA-ROS2', 'calib_output'))


def _write_tmp(content):
    out = os.path.join('/tmp', f'multiview_{REFERENCE_FRAME}.rviz')
    with open(out, 'w') as f:
        f.write(content)
    return out


# ─────────────────────────── CALIBRATION (calibration:=true) ────────────────

def _detect_ip_octet():
    """Last octet of this machine's 192.168.0.x address ('' if not found)."""
    try:
        out = subprocess.check_output(['ip', '-4', 'addr'], text=True, stderr=subprocess.DEVNULL)
        for line in out.splitlines():
            m = re.search(r'inet 192\.168\.0\.(\d+)/', line)
            if m:
                return m.group(1)
    except Exception:
        pass
    return ''


def _is_host_machine():
    octet = _detect_ip_octet()
    return octet.isdigit() and 61 <= int(octet) <= 70


def _calib_actions(context):
    """calibration:=true.

    is_host:=true  → broadcast ONE std_msgs/Empty on /fleet/calibrate so every edge
                     running an idle calib listener (camera.launch.py calib_listener:=true)
                     calibrates its own camera locally and saves its own multiview.yml.
                     No SSH/accounts: the trigger rides the shared DDS domain.
    is_host:=false → run the local calibration node on THIS machine's camera.
    """
    trigger_topic = LaunchConfiguration('trigger_topic').perform(context).strip() or '/fleet/calibrate'
    is_host_arg = LaunchConfiguration('is_host').perform(context).strip().lower()
    is_host = _is_host_machine() if is_host_arg == 'auto' else is_host_arg in ('true', '1', 'yes')
    if is_host:
        print(f'[multiview] is_host → broadcasting {trigger_topic} to the fleet '
              '(each edge calibrates locally).')
        # publish a few times so every edge is discovered; edges ignore repeats while armed.
        return [ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '5', '-r', '2',
                 trigger_topic, 'std_msgs/msg/Empty', '{}'],
            output='screen')]

    scripts_dir = _scripts_dir()
    detect      = os.path.join(scripts_dir, 'detect_camera_id.py')
    node_script = os.path.join(scripts_dir, 'multiview_calib_node.py')

    camera_id = LaunchConfiguration('camera_id').perform(context)
    if not camera_id:
        try:
            camera_id = subprocess.check_output(
                ['python3', detect], text=True, stderr=subprocess.DEVNULL).strip()
            print(f'[multiview] auto-detected camera_id: {camera_id}')
        except subprocess.CalledProcessError:
            print('[multiview] ERROR: camera not detected; pass camera_id:=<serial>')
            camera_id = ''

    image_topic = LaunchConfiguration('image_topic').perform(context)
    if image_topic == 'auto':
        octet = _detect_ip_octet()
        image_topic = f'/cam_{octet}/camera/rgb/image_raw' if octet else '/camera/rgb/image_raw'
    print(f'[multiview] calibration: camera_id={camera_id} image_topic={image_topic}')

    return [ExecuteProcess(
        cmd=['python3', node_script,
             '--camera-id',     camera_id,
             '--calib-dir',     _calib_dir(),
             '--image-topic',   image_topic,
             '--library-hd',    LaunchConfiguration('library_hd').perform(context),
             '--num-frames',    LaunchConfiguration('num_frames').perform(context),
             '--duration',      LaunchConfiguration('duration').perform(context),
             '--reproj-thresh', LaunchConfiguration('reproj_thresh').perform(context),
             '--display',       LaunchConfiguration('use_gui').perform(context),
             '--depth-refine',  LaunchConfiguration('depth_refine').perform(context),
             '--depth-band',    LaunchConfiguration('depth_band').perform(context),
             '--ransac-tol',    LaunchConfiguration('ransac_tol').perform(context),
             '--min-plane-inlier-ratio', LaunchConfiguration('min_plane_inlier_ratio').perform(context),
             '--max-depth-delta', LaunchConfiguration('max_depth_delta').perform(context),
             '--slide-crop-x',  LaunchConfiguration('slide_crop_x').perform(context),
             '--slide-crop-y',  LaunchConfiguration('slide_crop_y').perform(context),
             '--slide-z-band',  LaunchConfiguration('slide_z_band').perform(context),
             '--slide-stride',  LaunchConfiguration('slide_stride').perform(context),
             '--slide-min-points', LaunchConfiguration('slide_min_points').perform(context),
             '--slide-min-range', LaunchConfiguration('slide_min_range').perform(context),
             '--slide-max-range', LaunchConfiguration('slide_max_range').perform(context),
             '--slide-search-radius', LaunchConfiguration('slide_search_radius').perform(context),
             '--debug-roi', LaunchConfiguration('debug_roi').perform(context),
             '--debug-roi-max-points', LaunchConfiguration('debug_roi_max_points').perform(context)],
        output='screen')]


# ─────────────────────────────── VIEWER (default) ───────────────────────────

def _sort_ids(ids):
    return sorted(ids, key=lambda s: (0, int(s)) if s.isdigit() else (1, s))


def _detect_live_cameras(timeout=8.0, settle_after_first=3.0):
    """cam ids publishing a colour cloud right now, e.g. {'56','57'}.
    Keep collecting briefly after the first hit so late DDS discovery does not
    make the viewer launch with only one camera."""
    pat = re.compile(r'^/cam_([^/]+)/camera/point_cloud_rgb$')
    deadline = time.time() + timeout
    settle_deadline = None
    found = set()
    while time.time() < deadline:
        try:
            out = subprocess.check_output(['ros2', 'topic', 'list', '--no-daemon'], text=True,
                                          stderr=subprocess.DEVNULL, timeout=5)
        except Exception:
            out = ''
        before = len(found)
        for ln in out.splitlines():
            m = pat.match(ln.strip())
            if m:
                found.add(m.group(1))
        if found and settle_deadline is None:
            settle_deadline = time.time() + settle_after_first
        elif len(found) > before:
            settle_deadline = time.time() + settle_after_first
        if settle_deadline is not None and time.time() >= settle_deadline:
            break
        time.sleep(1.0)
    return found


def _calib_cameras(calib_dir):
    """Fallback: cam ids that have a saved multiview.yml (namespace cam_<id>)."""
    ids = set()
    for mv in glob.glob(os.path.join(calib_dir, '*', 'multiview.yml')):
        try:
            with open(mv) as f:
                m = re.search(r'/cam_([^/]+)/', f.read())
            if m:
                ids.add(m.group(1))
        except OSError:
            pass
    return ids


def _resolve_cameras(context, calib_dir):
    """cameras:= override → live detection → calib fallback."""
    override = LaunchConfiguration('cameras').perform(context).strip()
    if override:
        ids = {c.strip().lstrip('cam_') for c in re.split(r'[,\s]+', override) if c.strip()}
        print(f'[multiview] cameras (manual): {_sort_ids(ids)}')
        return _sort_ids(ids)
    live = _detect_live_cameras()
    if live:
        live_sorted = _sort_ids(live)
        print(f'[multiview] cameras (live): {live_sorted}')
        return live_sorted
    calib = _calib_cameras(calib_dir)
    if calib:
        print(f'[multiview] no live cloud found; falling back to calib_output: {_sort_ids(calib)}')
        return _sort_ids(calib)
    print('[multiview] WARNING: no cameras detected; RViz will be empty. '
          'Pass cameras:=56,57 to force.')
    return []


def _camera_group(cam_id, color, show_rgb_image, cloud_topic_depth, cloud_filter_size,
                  cloud_point_size_pixels, roi_debug_decay):
    """One RViz Group = colour PointCloud2 + uniquely-named RGB Image."""
    image_display = ''
    if show_rgb_image:
        image_display = f"""\
        - Class: rviz_default_plugins/Image
          Enabled: true
          Max Value: 1
          Min Value: 0
          Name: cam_{cam_id} RGB
          Normalize Range: true
          Topic:
            Depth: 1
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /cam_{cam_id}/camera/rgb/image_raw/compressed
          Value: true
"""
    return f"""\
    # ── cam_{cam_id} ──────────────────────────────────────────────────────
    - Class: rviz_common/Group
      Displays:
        - Alpha: 1
          Autocompute Intensity Bounds: true
          Autocompute Value Bounds:
            Max Value: 12
            Min Value: 0
            Value: true
          Axis: Z
          Channel Name: rgb
          Class: rviz_default_plugins/PointCloud2
          Color: {color}
          Color Transformer: RGB8
          Decay Time: 0
          Enabled: true
          Max Color: 255; 255; 255
          Min Color: 0; 0; 0
          Name: RGB PointCloud
          Position Transformer: XYZ
          Size (Pixels): {cloud_point_size_pixels}
          Size (m): 0.009999999776482582
          Style: Points
          Topic:
            Depth: {cloud_topic_depth}
            Durability Policy: Volatile
            Filter size: {cloud_filter_size}
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /cam_{cam_id}/camera/point_cloud_rgb
          Use Fixed Frame: true
          Value: true
        - Alpha: 1
          Autocompute Intensity Bounds: true
          Autocompute Value Bounds:
            Max Value: 1
            Min Value: 0
            Value: true
          Axis: Z
          Channel Name: rgb
          Class: rviz_default_plugins/PointCloud2
          Color: {color}
          Color Transformer: FlatColor
          Decay Time: {roi_debug_decay}
          Enabled: true
          Max Color: 255; 255; 255
          Min Color: 0; 0; 0
          Name: ROI Points cam_{cam_id}
          Position Transformer: XYZ
          Size (Pixels): 5
          Size (m): 0.009999999776482582
          Style: Points
          Topic:
            Depth: 1
            Durability Policy: Volatile
            Filter size: {cloud_filter_size}
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /cam_{cam_id}/multiview_debug/roi_points
          Use Fixed Frame: true
          Value: true
        - Class: rviz_default_plugins/MarkerArray
          Enabled: true
          Name: ROI Boxes cam_{cam_id}
          Topic:
            Depth: 1
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /cam_{cam_id}/multiview_debug/roi_markers
          Value: true
{image_display.rstrip()}
      Enabled: true
      Name: cam_{cam_id}_PointCloud
"""


def _layout_state(cam_ids):
    """QMainWindow State hex for the viewer layout (Displays on top, each RGB image
    below it, Time at the bottom). Built per camera-set by gen_multiview_rviz_layout.py
    (PyQt, offscreen) so the arrangement survives a relaunch. Returns '' on any
    failure → rviz falls back to its default docking."""
    script = os.path.join(_scripts_dir(), 'gen_multiview_rviz_layout.py')
    try:
        return subprocess.check_output(
            ['python3', script, *[str(c) for c in cam_ids]],
            text=True, stderr=subprocess.DEVNULL, timeout=20).strip()
    except Exception:
        return ''


def _build_rviz_config(
    cam_ids,
    fixed_frame,
    show_rgb_images,
    cloud_topic_depth,
    cloud_filter_size,
    cloud_point_size_pixels,
    rviz_frame_rate,
    roi_debug_decay,
):
    """Render a complete rviz2 config for exactly `cam_ids` (already sorted)."""
    expanded = '\n'.join(f'        - /cam_{c}_PointCloud1' for c in cam_ids) or \
        '        - /Global Options1'
    groups = ''.join(_camera_group(
        c,
        PALETTE[i % len(PALETTE)],
        show_rgb_images,
        cloud_topic_depth,
        cloud_filter_size,
        cloud_point_size_pixels,
        roi_debug_decay,
    )
                     for i, c in enumerate(cam_ids))
    img_geometry = ('\n'.join(f'  cam_{c} RGB:\n    collapsed: false' for c in cam_ids)
                    if show_rgb_images else '')
    state = _layout_state(cam_ids)
    qmw = f'  QMainWindow State: {state}\n' if state else ''

    return f"""\
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
{expanded}
      Splitter Ratio: 0.5
    Tree Height: 600
  - Class: rviz_common/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: ""
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 0.5
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Update Interval: 0
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Topic:
        Depth: 1
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /visualization_marker_array
      Name: Pose Training Bounds
      Namespaces:
        pose_training_bounds: true
        pose_training_bounds_outline: true
      Value: true
{groups}  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: {fixed_frame}
    Frame Rate: {rviz_frame_rate}
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 5
      Enable Stereo Rendering:
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.785398
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
{img_geometry}
{qmw}  Height: 1016
  Width: 1850
  X: 70
  Y: 27
"""


def _viewer_actions(context):
    """RViz viewer (dynamic per-camera layout) + optional local multiview_tf scan."""
    calib_dir = _calib_dir()
    actions = []
    explicit_cfg = LaunchConfiguration('rviz_config').perform(context).strip()
    if explicit_cfg:
        # Power-user path: use a hand-made config as-is, only patch the Fixed Frame.
        try:
            with open(explicit_cfg) as f:
                content = f.read().replace('Fixed Frame: world',
                                           f'Fixed Frame: {REFERENCE_FRAME}')
            rviz_config = _write_tmp(content)
        except OSError:
            rviz_config = explicit_cfg
    else:
        cam_ids = _resolve_cameras(context, calib_dir)
        cloud_topic_depth = max(1, int(LaunchConfiguration('cloud_topic_depth').perform(context)))
        cloud_filter_size = max(1, int(LaunchConfiguration('cloud_filter_size').perform(context)))
        cloud_point_size_pixels = max(1, int(LaunchConfiguration('cloud_point_size_pixels').perform(context)))
        rviz_frame_rate = max(1, int(LaunchConfiguration('rviz_frame_rate').perform(context)))
        roi_debug_decay = LaunchConfiguration('roi_debug_decay').perform(context).strip()
        rviz_config = _write_tmp(
            _build_rviz_config(
                cam_ids,
                REFERENCE_FRAME,
                _is_true(context, 'show_rgb_images'),
                cloud_topic_depth,
                cloud_filter_size,
                cloud_point_size_pixels,
                rviz_frame_rate,
                roi_debug_decay,
            ))

    # Optional LOCAL scan: only to review this machine's saved calib_output when no
    # live edge is publishing (offline). Off by default — running it while edges also
    # publish would duplicate /tf_static (TF_REPEATED_DATA spam).
    if _is_true(context, 'use_multiview_tf'):
        tf_script = os.path.join(_scripts_dir(), 'multiview_tf_node.py')
        # Offline review on a host: anchor EVERY saved camera (the node now defaults to
        # local-serial-only, so this path opts back into the scan-all behaviour).
        actions.append(ExecuteProcess(
            cmd=['python3', tf_script, '--calib-dir', calib_dir, '--scan-all'], output='screen'))

    # Host deterministic chain solver — pinned here so ONE `mtf` (broadcast /fleet/calibrate) is a one-touch
    # fleet recalibration while the viewer runs: it collects every edge's tag observations, runs
    # the priority tag/camera chain, and (writeback) pushes each solved multiview.yml back to its
    # edge, whose multiview_tf_node then re-publishes /tf_static → RViz re-anchors live. Host-only
    # (guarded) so an edge that happens to run the viewer never spawns a second solver.
    if _is_true(context, 'solver') and _is_host_machine():
        solver_script = os.path.join(_scripts_dir(), 'multiview_solver_node.py')
        cmd = ['python3', solver_script,
               '--trigger-topic', LaunchConfiguration('trigger_topic').perform(context)]
        if _is_true(context, 'solver_writeback'):
            cmd.append('--writeback')
        extra = LaunchConfiguration('solver_args').perform(context).strip()
        if extra:
            import shlex
            cmd += shlex.split(extra)
        actions.append(ExecuteProcess(cmd=cmd, output='screen'))

    if _is_true(context, 'use_rviz'):
        rviz_node = Node(package='rviz2', executable='rviz2', name='multi_sensor_viewer',
                         arguments=['-d', rviz_config], output='screen')
        # Start RViz a few seconds late so latched /tf_static (from the edges) is received
        # before the cloud displays subscribe.
        actions.append(TimerAction(period=3.0, actions=[rviz_node]))
    return actions


# ──────────────────────────────── dispatch ──────────────────────────────────

def _setup(context):
    if _is_true(context, 'calibration'):
        return _calib_actions(context)
    return _viewer_actions(context)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration', default_value='false',
            description='true → STag calibration (headless one-touch, no RViz); '
                        'false (default) → the RViz multiview viewer'),
        DeclareLaunchArgument('is_host', default_value='auto',
            description='With calibration:=true, broadcast /fleet/calibrate so every edge '
                        '(calib_listener) calibrates its own camera at once. auto = host IP 192.168.0.61-70.'),
        DeclareLaunchArgument('trigger_topic', default_value='/fleet/calibrate',
            description='Topic the host broadcasts std_msgs/Empty on to start fleet calibration.'),
        DeclareLaunchArgument('solver', default_value='true',
            description='Run the host deterministic multi-tag chain solver alongside the viewer '
                        '(host machine only): it listens on trigger_topic, fuses every edge into stag_marker, and '
                        'writes results back. This is what makes `mtf` a one-touch recalibration.'),
        DeclareLaunchArgument('solver_writeback', default_value='true',
            description='true → solver pushes each solved multiview.yml to its edge (live). '
                        'false → dry-run: write under the solver out-dir only, do not touch edges.'),
        DeclareLaunchArgument('solver_args', default_value='',
            description='Extra args appended to the host chain solver, e.g. '
                        'solver_args:="--ref-id 0 --depth-vote-range 0.8" '
                        'to retune deterministic linking/depth voting without editing code.'),

        # ── viewer options (calibration:=false) ──
        DeclareLaunchArgument('use_rviz', default_value='true',
            description='Start the Host RViz multiview viewer. Set false for headless checks.'),
        DeclareLaunchArgument('cameras', default_value='',
            description="Comma list of cam ids to show (e.g. 56,57). Empty = auto-detect "
                        "live point_cloud_rgb topics, falling back to calib_output."),
        DeclareLaunchArgument('rviz_config', default_value='',
            description='Optional hand-made rviz config to use as-is (Fixed Frame patched). '
                        'Empty = generate the layout dynamically from connected cameras.'),
        DeclareLaunchArgument('show_rgb_images', default_value='true',
            description='Also display RGB image panels using compressed image transport. '
                        'Set false for point-cloud-only RViz.'),
        DeclareLaunchArgument('cloud_topic_depth', default_value='1',
            description='RViz PointCloud2 topic queue depth. 1 keeps only the newest frame.'),
        DeclareLaunchArgument('cloud_filter_size', default_value='1',
            description='RViz PointCloud2 TF/message-filter queue size. 1 avoids backlog.'),
        DeclareLaunchArgument('cloud_point_size_pixels', default_value='3',
            description='RViz PointCloud2 point size in pixels (default 3 = easier to see; '
                        'lower to 1 for the lightest multi-camera view).'),
        DeclareLaunchArgument('roi_debug_decay', default_value='30',
            description='RViz decay time in seconds for ROI debug point clouds. Main RGB point clouds stay live.'),
        DeclareLaunchArgument('rviz_frame_rate', default_value='15',
            description='RViz render frame rate cap. Lower values reduce GPU/CPU pressure.'),
        DeclareLaunchArgument('use_multiview_tf', default_value='false',
            description='Also run a LOCAL multiview_tf scan of this machine\'s calib_output '
                        '(offline review when no live edge publishes the stag_marker TFs). '
                        f'Default false. RViz Fixed Frame is always "{REFERENCE_FRAME}".'),

        # ── calibration-node options (calibration:=true) ──
        DeclareLaunchArgument('camera_id', default_value='',
            description='Camera serial; auto-detected via USB if empty'),
        DeclareLaunchArgument('image_topic', default_value='auto',
            description="'auto' → /cam_{ip_octet}/camera/rgb/image_raw; or an explicit topic"),
        DeclareLaunchArgument('library_hd', default_value='21',
            description='STag HD library [11,13,15,17,19,21,23]'),
        DeclareLaunchArgument('num_frames', default_value='10',
            description='One-touch (default): collect this many good reference views, then '
                        'median-average + LiDAR depth-refine + save. ~10 is plenty.'),
        DeclareLaunchArgument('duration', default_value='0',
            description='Optional time-based collection (seconds). 0 (default) = count-based (num_frames).'),
        DeclareLaunchArgument('reproj_thresh', default_value='3.0',
            description='Max per-view reprojection RMSE (px) to accept a view'),
        DeclareLaunchArgument('use_gui', default_value='false',
            description='Show the live detection window (default false = headless one-touch). '
                        'true just adds the viewer + [s]save [r]reset [q]quit; it still auto-saves after duration.'),
        DeclareLaunchArgument('depth_refine', default_value='true',
            description='Per-tag LiDAR-plane RANSAC depth refine (needs extrinsic.yml)'),
        DeclareLaunchArgument('depth_band', default_value='0.05',
            description='± depth band (m) for the LiDAR RANSAC/refinement crop'),
        DeclareLaunchArgument('ransac_tol', default_value='0.08',
            description='RANSAC inlier tolerance (m) for the LiDAR marker-plane fit (capped at depth_band)'),
        DeclareLaunchArgument('min_plane_inlier_ratio', default_value='0.0',
            description='Reject LiDAR depth refine unless this fraction of the selected crop agrees with one plane; 0 keeps only absolute support'),
        DeclareLaunchArgument('max_depth_delta', default_value='0.0',
            description='Reject LiDAR depth refine when the final range correction exceeds this many meters; 0 disables so large pulls are visible'),
        DeclareLaunchArgument('slide_crop_x', default_value='0.35',
            description='Sliding mode: marker-frame left/right crop half-width in meters'),
        DeclareLaunchArgument('slide_crop_y', default_value='0.35',
            description='Sliding mode: marker-frame up/down crop half-height in meters'),
        DeclareLaunchArgument('slide_z_band', default_value='0.03',
            description='Sliding mode: count points this close to the marker plane in marker-frame z'),
        DeclareLaunchArgument('slide_stride', default_value='0.05',
            description='Sliding mode: camera-ray range stride in meters'),
        DeclareLaunchArgument('slide_min_points', default_value='30',
            description='Sliding mode: minimum LiDAR points required for a valid marker crop'),
        DeclareLaunchArgument('slide_min_range', default_value='0.05',
            description='Sliding mode: minimum non-zero camera-ray range in meters'),
        DeclareLaunchArgument('slide_max_range', default_value='0.0',
            description='Sliding mode: optional maximum camera-ray range in meters; <=0 uses cloud max'),
        DeclareLaunchArgument('slide_search_radius', default_value='0.0',
            description='Sliding mode: only search this many meters around the monocular STag range; 0 searches all cloud ranges'),
        DeclareLaunchArgument('debug_roi', default_value='false',
            description='Publish RViz debug ROI boxes/points for each tag depth-refine result'),
        DeclareLaunchArgument('debug_roi_max_points', default_value='3000',
            description='Maximum selected LiDAR points to publish per tag ROI debug snapshot'),

        OpaqueFunction(function=_setup),
    ])
