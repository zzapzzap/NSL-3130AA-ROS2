"""Idle calibration must not request organized raw data; active refine must."""
import ast
from pathlib import Path
from types import SimpleNamespace


def methods():
    path = Path(__file__).parents[1]/'roboscan_nsl3130/scripts/multiview_calib_node.py'
    tree = ast.parse(path.read_text())
    selected = [n for node in tree.body if isinstance(node, ast.ClassDef)
                for n in node.body if isinstance(n, ast.FunctionDef)
                and n.name in ('_set_raw_cloud_subscription', '_cloud_cb')]
    namespace = {'PointCloud2': object, '_sensor_qos': lambda depth: depth,
                 '_cloud_to_xyz': lambda msg: ('full-raw-xyz',)}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), 'exec'), namespace)
    return namespace


def test_raw_subscription_only_while_refining():
    ns = methods()
    created, destroyed = [], []
    def create(*args):
        created.append(args)
        return 'raw-subscription'
    node = SimpleNamespace(a=SimpleNamespace(depth_refine=True), _ext=True,
                           _raw_cloud_sub=None, _raw_cloud_topic='/cam_161/camera/point_cloud',
                           _cloud_cb=lambda msg: None, create_subscription=create,
                           destroy_subscription=destroyed.append)
    update = ns['_set_raw_cloud_subscription']
    update(node, False)
    assert not created
    update(node, True)
    update(node, True)
    assert len(created) == 1
    assert created[0][1] == '/cam_161/camera/point_cloud'
    update(node, False)
    assert destroyed == ['raw-subscription']
    assert node._raw_cloud_sub is None


def test_rgb_frame_metadata_does_not_replace_raw_depth_refinement():
    callback = methods()['_cloud_cb']
    node = SimpleNamespace(live_lidar_frame='cam_161_lidar_frame',
                           a=SimpleNamespace(depth_refine=True), _ext=True,
                           saved=False, _clouds=[])
    msg = SimpleNamespace(header=SimpleNamespace(frame_id='cam_161_lidar_frame'))
    callback(node, msg, frame_only=True)
    assert not node._clouds
    callback(node, msg)
    assert node._clouds == [('full-raw-xyz',)]


def test_frame_metadata_subscription_released_after_first_message():
    callback = methods()['_cloud_cb']
    destroyed = []
    node = SimpleNamespace(live_lidar_frame='cam_161_lidar_frame',
                           a=SimpleNamespace(depth_refine=True), _ext=True,
                           saved=False, _clouds=[], _frame_sub='metadata',
                           destroy_subscription=destroyed.append)
    msg = SimpleNamespace(header=SimpleNamespace(frame_id='cam_161_lidar_frame'))
    callback(node, msg, frame_only=True)
    assert destroyed == ['metadata']
    assert node._frame_sub is None
    assert not node._clouds
