"""Saved map poses and each marker-relative camera pose retain their frame convention."""
from pathlib import Path
import unittest

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[4]


class GlobalFrameTests(unittest.TestCase):
    def test_saved_marker_relative_poses_compose_to_the_map_pose(self):
        frames = yaml.safe_load((ROOT/'controller/src/NSL-3130AA-ROS2/calib_output/global_frames.yml').read_text())
        identity = frames['global']['anchor']['id_0']
        if identity['verification'] == 'unverified_identity_seed':
            np.testing.assert_array_equal(np.array(identity['tf']).reshape(4, 4), np.eye(4))
        for name, camera in frames['global']['cameras'].items():
            for marker, anchor in frames['global']['anchor'].items():
                relation = frames['relative'][marker][name]
                self.assertEqual(relation['parent'], marker)
                self.assertEqual(relation['child'], f'{name}_lidar_frame')
                if camera['tf'] is None:
                    self.assertIsNone(relation['tf'])
                else:
                    map_anchor = np.array(anchor['tf']).reshape(4, 4)
                    anchor_camera = np.array(relation['tf']).reshape(4, 4)
                    np.testing.assert_allclose(map_anchor @ anchor_camera,
                                               np.array(camera['tf']).reshape(4, 4), atol=1e-12)


if __name__ == '__main__':
    unittest.main()
