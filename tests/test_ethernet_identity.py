"""Verify chip/wafer serial composition and reject uninitialized SDK observations."""
import importlib.util
from pathlib import Path
import unittest

SCRIPT=Path(__file__).resolve().parents[1]/'roboscan_nsl3130/scripts/detect_camera_id.py'
spec=importlib.util.spec_from_file_location('detect_identity',SCRIPT)
identity=importlib.util.module_from_spec(spec);spec.loader.exec_module(identity)

class EthernetIdentityTests(unittest.TestCase):
    def test_known_device_ids_preserve_padding_and_chip_wafer_order(self):
        for chip,wafer,serial in [(0x100,0x5c4,'N010005C4'),(0x8b,0x60d,'N008B060D'),(0xc7,0x5c3,'N00C705C3')]:
            self.assertEqual(identity.compose_camera_serial(chip,wafer),serial)

    def test_invalid_and_uninitialized_fields_are_not_serials(self):
        for chip,wafer in [(-1,1476),(256,-1),(65536,1),(True,1476),('256',1476),(0,0),(65535,65535)]:
            with self.assertRaises(ValueError):identity.compose_camera_serial(chip,wafer)

    def test_cached_serial_is_not_a_live_sdk_observation(self):
        with self.assertRaises(ValueError):
            identity.validate_ethernet_identity({'serial':'N010005C4','source':'cache'})
