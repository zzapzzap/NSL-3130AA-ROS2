#!/usr/bin/env python3
"""Edge-side WRITEBACK receiver for the multiview bundle solver.

The host `multiview_bundle_node` solves every camera's pose globally, then pushes each edge its
solved `multiview.yml` here over the proven chunked + sha256 transfer (the same PutWeight service
the weight server uses). The file is staged and installed **atomically** at
`calib_output/{serial}/multiview.yml` only after the digest matches, so a crashed transfer can
never corrupt the calibration. The local `multiview_tf_node` then re-publishes `/tf_static` on its
1 s mtime poll — no SSH, no restart.

Per-edge service name (PutWeight carries no edge-id field, so a shared endpoint would let one
host push collide across edges):  `/<namespace>/multiview/put`  e.g. `/cam_52/multiview/put`.

    ros2 run roboscan_nsl3130 multiview_put_server.py \
        --camera-id N00A5060D --calib-dir /path/to/calib_output --namespace cam_52
"""

import argparse
import hashlib
import os
import sys

import rclpy
from rclpy.node import Node

from ros_humanpose.srv import PutWeight    # reuse the proven chunked transfer contract

ALLOWED = 'multiview.yml'


def _sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


class MultiviewPutServer(Node):
    def __init__(self, serial, calib_dir, namespace):
        super().__init__(f'multiview_put_server_{serial}' if serial else 'multiview_put_server')
        self.dev_dir = os.path.join(os.path.expanduser(calib_dir), serial)
        os.makedirs(self.dev_dir, exist_ok=True)
        svc = f'/{namespace}/multiview/put' if namespace else '/multiview/put'
        self.create_service(PutWeight, svc, self._on_put)
        self.get_logger().info(
            f'[multiview_put] writeback ready: {svc} -> {os.path.join(self.dev_dir, ALLOWED)}')

    def _on_put(self, req, resp):
        # Chunked, ordered, sha256-verified, atomic install — cloned from weight_server._on_put,
        # rooted at THIS edge's calib_output/{serial}/ and locked to multiview.yml.
        resp.complete = False
        if os.path.basename(req.name.strip()) != ALLOWED:
            resp.success = False
            resp.message = f'only {ALLOWED} accepted (got {req.name!r})'
            return resp
        final = os.path.join(self.dev_dir, ALLOWED)
        staging = os.path.join(self.dev_dir, f'.staging.{ALLOWED}')
        try:
            staged = os.path.getsize(staging) if os.path.exists(staging) else 0
            if req.offset == 0:
                staged = 0
                open(staging, 'wb').close()
                self.get_logger().info(f'[multiview_put] starting ({req.total_size} bytes)')
            if req.offset != staged:
                resp.success = False
                resp.message = f'out-of-order chunk: expected offset {staged}, got {req.offset}'
                return resp
            with open(staging, 'ab') as f:
                f.write(bytes(req.data))
            staged += len(req.data)
            if staged > req.total_size:
                os.remove(staging)
                resp.success = False
                resp.message = f'overflow: staged {staged} > total {req.total_size}; reset'
                return resp
            if staged == req.total_size:
                digest = _sha256(staging)
                if digest != req.sha256:
                    os.remove(staging)
                    resp.success = False
                    resp.message = f'sha256 mismatch: got {digest[:12]}.., want {req.sha256[:12]}..'
                    return resp
                os.replace(staging, final)
                resp.complete = True
                self.get_logger().info(f'[multiview_put] complete, verified, installed -> {final}')
            resp.success = True
            resp.message = ''
        except OSError as exc:
            resp.success = False
            resp.message = f'write failed: {exc}'
        return resp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera-id', required=True, help='Camera USB serial (calib_output subfolder key)')
    ap.add_argument('--calib-dir', required=True, help='calib_output directory')
    ap.add_argument('--namespace', default='', help='cam_NN — service is /<namespace>/multiview/put')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args or None)
    node = MultiviewPutServer(args.camera_id, args.calib_dir, args.namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
