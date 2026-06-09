#!/usr/bin/env python3

import os
import re
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, conditions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


FLEET_FIRST_OCTET = 51
FLEET_LAST_OCTET = 59
CAMERA_HOST_OFFSET = 50
CAMERA_SENSOR_OFFSET = 150
DEFAULT_CAMERA_IP = '192.168.2.201'
DEFAULT_CAMERA_GATEWAY = '192.168.2.1'


def generate_launch_description():

    pkg_share = get_package_share_directory('roboscan_nsl3130')
    param_path = os.path.join(pkg_share, 'rqt.yaml')
    params = [param_path] if os.path.exists(param_path) else []

    rviz_config = os.path.join(pkg_share, 'rviz', 'roboscan_nsl3130rviz.rviz')

    # Standard colcon layout: install/<pkg>/share/<pkg>/ → 4 levels up = workspace root
    ws_root = os.path.normpath(os.path.join(pkg_share, '..', '..', '..', '..'))
    calib_dir = os.environ.get(
        'NSL_CALIB_DIR',
        os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2', 'calib_output'))

    repo_scripts = os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2', 'NSL3130_driver',
                                'src', 'roboscan_nsl3130', 'scripts')
    extrinsic_tf_script = os.path.join(repo_scripts, 'extrinsic_tf_node.py')
    multiview_tf_script = os.path.join(repo_scripts, 'multiview_tf_node.py')
    detect_script       = os.path.join(repo_scripts, 'detect_camera_id.py')

    # Sensor-tuning profiles. The driver reads the file in NSL_PARAMS_FILE
    # (see roboscan_publish_node.cpp). Defaults are the repo (zzapzzap) baseline.
    general_params = os.path.join(pkg_share, 'lidar_params.yaml')
    calib_params   = os.path.join(pkg_share, 'lidar_params_calibration.yaml')

    def _detect_serial():
        """Return the USB serial of the attached camera ('' if none)."""
        try:
            return subprocess.check_output(
                ['python3', detect_script], text=True,
                stderr=subprocess.DEVNULL).strip()
        except Exception:
            return ''

    def _detect_ip_octet():
        """Return last octet of this machine's 192.168.0.x address ('' if not found)."""
        try:
            out = subprocess.check_output(
                ['ip', '-4', 'addr'], text=True, stderr=subprocess.DEVNULL)
            for line in out.splitlines():
                m = re.search(r'inet 192\.168\.0\.(\d+)/', line)
                if m:
                    return m.group(1)
        except Exception:
            pass
        return ''

    def _resolve_namespace(context, ip_octet):
        """Resolve the per-device namespace based on the machine's IP last octet.

          namespace:=auto    → cam_{last_octet}  e.g. cam_51; DEFAULT.
                               Falls back to '' when no 192.168.0.x address found.
          namespace:=''      → no namespace → /camera/point_cloud, ...
          namespace:=<name>  → used verbatim.
        """
        ns = LaunchConfiguration('namespace').perform(context).strip().strip('/')
        if ns.lower() == 'auto':
            if ip_octet:
                ns = f'cam_{ip_octet}'
                print(f'[camera] namespace: /{ns}  (auto from 192.168.0.{ip_octet})')
            else:
                ns = ''
                print('[camera] namespace:=auto but no 192.168.0.x address found → no namespace')
        if ns:
            print(f'[camera] topics → /{ns}/camera/point_cloud, /{ns}/camera/rgb/image_raw, ...')
        return ns

    def _fleet_octet(ip_octet):
        try:
            octet = int(ip_octet)
        except (TypeError, ValueError):
            return None
        if FLEET_FIRST_OCTET <= octet <= FLEET_LAST_OCTET:
            return octet
        return None

    def _resolve_camera_network(context, ip_octet):
        """Resolve camera-link addresses from the fleet Set number.

        Rule: Edge LAN 192.168.0.N uses camera NIC 192.168.2.(N+50) and
        camera sensor 192.168.2.(N+150). Example: N=51 -> 101/201.
        """
        octet = _fleet_octet(ip_octet)
        raw_ip = LaunchConfiguration('camera_ip').perform(context).strip()
        raw_gateway = LaunchConfiguration('camera_gateway').perform(context).strip()

        if raw_ip.lower() == 'auto':
            if octet is not None:
                camera_ip = f'192.168.2.{octet + CAMERA_SENSOR_OFFSET}'
                print(f'[camera] camera_ip: {camera_ip}  (auto from Set {octet})')
            else:
                camera_ip = DEFAULT_CAMERA_IP
                print(f'[camera] camera_ip:=auto but Set number is not {FLEET_FIRST_OCTET}-{FLEET_LAST_OCTET}; using {camera_ip}')
        else:
            camera_ip = raw_ip

        camera_gateway = DEFAULT_CAMERA_GATEWAY if raw_gateway.lower() == 'auto' else raw_gateway

        return camera_ip, camera_gateway

    def _refresh_fastdds_profile(ip_octet):
        if not ip_octet:
            return
        lan_ip = f'192.168.0.{ip_octet}'
        ros_dir = os.path.join(os.path.expanduser('~'), '.ros')
        profile = os.path.join(ros_dir, 'fastdds_nsl.xml')
        os.makedirs(ros_dir, exist_ok=True)
        with open(profile, 'w') as f:
            f.write(f'''<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <profiles>
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>nsl_lan</transport_id>
                <type>UDPv4</type>
                <interfaceWhiteList>
                    <address>{lan_ip}</address>
                </interfaceWhiteList>
            </transport_descriptor>
        </transport_descriptors>
        <participant profile_name="nsl_lan_only" is_default_profile="true">
            <rtps>
                <userTransports>
                    <transport_id>nsl_lan</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
''')
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = profile
        print(f'[camera] FastDDS whitelist: {lan_ip}  ({profile})')

    def _resolve_params_file(context, serial):
        """Pick the sensor-params file for the driver.

        Priority:
          calibration:=true  → shared calibration profile
          else, USB serial detected:
              {calib_dir}/{serial}/params.yaml  (created from the general default
              on first run, so each camera keeps & persists its own tuning)
          else (no serial)   → general default (zzapzzap baseline)
        """
        calibration = (LaunchConfiguration('calibration')
                       .perform(context).strip().lower() in ('true', '1', 'yes'))
        if calibration:
            print(f'[camera] sensor params: {calib_params}  [calibration profile]')
            return calib_params

        if not serial:
            print(f'[camera] sensor params: {general_params}  '
                  f'[general default — no camera id detected]')
            return general_params

        dev_dir = os.path.join(calib_dir, serial)
        dev_file = os.path.join(dev_dir, 'params.yaml')
        if not os.path.exists(dev_file):
            try:
                os.makedirs(dev_dir, exist_ok=True)
                shutil.copy(general_params, dev_file)
                print(f'[camera] seeded per-device params from general default → {dev_file}')
            except OSError as e:
                print(f'[camera] could not seed {dev_file} ({e}); using general default')
                return general_params
        print(f'[camera] sensor params: {dev_file}  [per-device: {serial}]')
        return dev_file

    def _abs_topic(ns, topic):
        """Resolve a (possibly relative) topic into the absolute name the node
        will publish on, accounting for the namespace. Used for the standalone
        extrinsic_tf process which is not itself inside the namespace."""
        rel = topic.lstrip('/')
        return f'/{ns}/{rel}' if ns else f'/{rel}'

    def _rviz_config(ns, serial):
        """Rewrite the bundled rviz config so topics AND frames match the live
        namespace. Topics: /camera/... → /{ns}/camera/...  Frames: the driver
        publishes the cloud in {ns}_lidar_frame (or {serial}_lidar_frame when no
        namespace), and extrinsic_tf_node hangs {ns}_camera_frame under it. The
        bundled config's Fixed Frame is one of these, so the view anchors on the
        live cloud frame (the old reference_lidar_frame bridge was removed)."""
        cam_prefix = f'/{ns}/camera/' if ns else '/camera/'
        lidar_frame  = (f'{ns}_lidar_frame'  if ns
                        else (f'{serial}_lidar_frame'  if serial else 'lidar_frame'))
        camera_frame = (f'{ns}_camera_frame' if ns
                        else (f'{serial}_camera_frame' if serial else 'camera_frame'))
        try:
            with open(rviz_config) as f:
                content = f.read()
            content = content.replace('/camera/', cam_prefix)
            # Frame tokens first (compound), then any remaining bare serial token.
            content = content.replace('N00A5060D_lidar_frame', lidar_frame)
            content = content.replace('N00A5060D_camera_frame', camera_frame)
            if serial:
                content = content.replace('N00A5060D', serial)
            out = os.path.join('/tmp', f'roboscan_{ns or "nons"}.rviz')
            with open(out, 'w') as f:
                f.write(content)
            return out
        except OSError:
            return rviz_config

    def _is_true(context, arg):
        return LaunchConfiguration(arg).perform(context).strip().lower() in ('true', '1', 'yes')

    def _fleet_setup(context):
        serial = _detect_serial()
        ip_octet = _detect_ip_octet()
        _refresh_fastdds_profile(ip_octet)
        ns = _resolve_namespace(context, ip_octet)
        camera_ip, camera_gateway = _resolve_camera_network(context, ip_octet)
        params_file = _resolve_params_file(context, serial)

        frame_id = f'{ns}_lidar_frame' if ns else ''

        actions = [Node(
            package='roboscan_nsl3130',
            executable='roboscan_publish_node',
            namespace=ns or None,            # '' → no namespace
            output='screen',
            parameters=params if params else None,
            additional_env={'NSL_CALIB_DIR': calib_dir,
                            'NSL_PARAMS_FILE': params_file,
                            'NSL_CAMERA_ID': serial,
                            'NSL_CAMERA_IP': camera_ip,
                            'NSL_CAMERA_NETMASK': LaunchConfiguration('camera_netmask').perform(context),
                            'NSL_CAMERA_GATEWAY': camera_gateway,
                            'NSL_NET_PREFLIGHT': LaunchConfiguration('net_preflight').perform(context),
                            'NSL_USB_ID': LaunchConfiguration('usb_id').perform(context),
                            'NSL_FRAME_ID': frame_id,
                            'NSL_CONNECTION': LaunchConfiguration('connection').perform(context)},
            remappings=[
                ('roboscanImage',         LaunchConfiguration('rgb_topic').perform(context)),
                ('roboscanDistance',      LaunchConfiguration('depth_topic').perform(context)),
                ('roboscanAmpl',          LaunchConfiguration('ampl_topic').perform(context)),
                ('roboscanGray',          LaunchConfiguration('gray_topic').perform(context)),
                ('roboscanPointCloud',    LaunchConfiguration('point_cloud_topic').perform(context)),
                ('roboscanPointCloudRgb', LaunchConfiguration('point_cloud_rgb_topic').perform(context)),
            ])]

        # Extrinsic → TF. Runs outside the namespace (publishes serial-prefixed
        # frames to the global /tf), so it needs the absolute point-cloud topic.
        if _is_true(context, 'use_extrinsic_tf'):
            lidar_topic = _abs_topic(ns, LaunchConfiguration('point_cloud_topic').perform(context))
            cmd = ['python3', extrinsic_tf_script,
                   '--calib-dir', calib_dir,
                   '--lidar-topic', lidar_topic]
            if serial:
                cmd += ['--camera-id', serial]
            if ns:
                cmd += ['--frame-prefix', ns]
            actions.append(ExecuteProcess(cmd=cmd, output='screen'))

        # STag multiview → TF. Anchors this camera under the shared `stag_marker`
        # reference frame from calib_output/{serial}/multiview.yml (composed with the
        # extrinsic for the lidar frame). Same role as multiview.launch.py but
        # published per-edge; warns and skips if multiview is not calibrated yet.
        if _is_true(context, 'use_multiview_tf'):
            cmd = ['python3', multiview_tf_script, '--calib-dir', calib_dir]
            if serial:
                cmd += ['--camera-id', serial]
            if ns:
                cmd += ['--frame-prefix', ns]
            actions.append(ExecuteProcess(cmd=cmd, output='screen'))

        # rviz2 (config rewritten for the namespace so the bundled view still works)
        if _is_true(context, 'use_rviz'):
            actions.append(Node(
                package='rviz2', executable='rviz2', name='rviz2',
                arguments=['-d', _rviz_config(ns, serial)], output='screen'))
        return actions

    return LaunchDescription([
        # ── General ──────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'namespace', default_value='auto',
            description="Per-device topic namespace. 'auto'=machine 192.168.0.x octet → "
                        "/cam_<octet>/camera/point_cloud, ... (DEFAULT); "
                        "''=none; or an explicit name."),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch rviz2 with the default config'),
        DeclareLaunchArgument(
            'use_rqt', default_value='true',
            description='Launch rqt_reconfigure for parameter tuning'),
        DeclareLaunchArgument(
            'use_extrinsic_tf', default_value='true',
            description='Publish {lidar_frame}→{id}_camera_frame TF from the saved extrinsic'),
        DeclareLaunchArgument(
            'use_multiview_tf', default_value='true',
            description='Publish stag_marker→{lidar_frame} TF from the saved multiview.yml '
                        '(STag shared reference; warns and skips if not yet calibrated)'),
        DeclareLaunchArgument(
            'connection', default_value='ethernet',
            description="'ethernet'=strict fleet runtime path with no USB fallback (DEFAULT); "
                        "'auto'=Ethernet first, then USB IP refresh/fallback if needed; "
                        "'usb'=USB only."),
        DeclareLaunchArgument(
            'camera_ip', default_value='auto',
            description='auto=192.168.2.(Set+150), e.g. Set 51 -> 192.168.2.201'),
        DeclareLaunchArgument(
            'camera_netmask', default_value='255.255.255.0',
            description='Camera netmask on the 192.168.2.x camera link'),
        DeclareLaunchArgument(
            'camera_gateway', default_value='auto',
            description='auto=192.168.2.1, matching the camera NIC GUI setup'),
        DeclareLaunchArgument(
            'net_preflight', default_value='true',
            description='true=check ping reachability before Ethernet SDK open'),
        DeclareLaunchArgument(
            'usb_id', default_value='',
            description='Optional USB path or serial for connection:=usb/auto; empty uses detected serial'),
        DeclareLaunchArgument(
            'calibration', default_value='false',
            description='true → shared calibration sensor profile (board-tuned); '
                        'false → per-camera profile (calib_output/{id}/params.yaml), '
                        'falling back to the general default'),
        # ── Topic remap targets (relative, nested under namespace)
        # With namespace=cam_51 these become /cam_51/camera/rgb/image_raw etc. ──
        DeclareLaunchArgument('rgb_topic',             default_value='camera/rgb/image_raw'),
        DeclareLaunchArgument('depth_topic',           default_value='camera/depth/image_raw'),
        DeclareLaunchArgument('ampl_topic',            default_value='camera/ampl'),
        DeclareLaunchArgument('gray_topic',            default_value='camera/gray'),
        DeclareLaunchArgument('point_cloud_topic',     default_value='camera/point_cloud'),
        DeclareLaunchArgument('point_cloud_rgb_topic', default_value='camera/point_cloud_rgb'),
        # ── Driver node + extrinsic TF + rviz (namespace & profile resolved at launch) ──
        OpaqueFunction(function=_fleet_setup),
        # ── rqt parameter reconfigure (delayed to let the node spin first) ──────
        TimerAction(
            period=12.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'rqt_gui', 'rqt_gui',
                         '--force-discover', '-s', 'rqt_reconfigure_combo'],
                    output='screen')
            ],
            condition=conditions.IfCondition(LaunchConfiguration('use_rqt'))),
    ])
