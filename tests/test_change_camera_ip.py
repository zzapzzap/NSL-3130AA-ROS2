"""Exercise address-change socket loss against a compiled fake vendor SDK."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
LIB = ROOT / 'roboscan_nsl3130/nsl_lib'

FAKE_SDK = r'''
#include "nanolib.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
using namespace NslOption;
static int opens = 0;
static bool changed = false;
static std::chrono::steady_clock::time_point opened;
static const char *scenario() { return std::getenv("TEST_SCENARIO"); }
static bool is(const char *value) { return std::strcmp(scenario(), value) == 0; }
static void log(const char *operation, int handle) {
    FILE *file = std::fopen(std::getenv("TEST_CALLS"), "a");
    std::fprintf(file, "%s %d\n", operation, handle);
    std::fclose(file);
}
int nsl_open(const char *, NslConfig *config, FUNCTION_OPTIONS) {
    const int handle = opens++;
    log("open", handle);
    if (is("reboot_delay") && handle > 0 && handle < 4) return -1;
    opened = std::chrono::steady_clock::now();
    config->chipID = is("wrong_device") && handle > 0 ? 257 : 256;
    config->waferID = 1476;
    return handle;
}
NSL_ERROR_TYPE nsl_closeHandle(int handle) {
    log("close", handle);
    return NSL_ERROR_TYPE::NSL_SUCCESS;
}
NSL_ERROR_TYPE nsl_getIpAddress(int handle, char *ip, char *mask, char *gw) {
    log("get", handle);
    if (is("needs_settle") && std::chrono::steady_clock::now() - opened < std::chrono::milliseconds(15))
        return NSL_ERROR_TYPE::NSL_ANSWER_ERROR;
    std::strcpy(ip, (changed || is("already_configured") || is("already_retry")) && !is("wrong_ip") ? "192.168.2.201" : "192.168.2.202");
    std::strcpy(mask, "255.255.255.0");
    std::strcpy(gw, "192.168.2.1");
    return NSL_ERROR_TYPE::NSL_SUCCESS;
}
NSL_ERROR_TYPE nsl_setIpAddress(int handle, const char *, const char *, const char *) {
    log("set", handle);
    changed = true;
    return is("normal") ? NSL_ERROR_TYPE::NSL_SUCCESS : NSL_ERROR_TYPE::NSL_ANSWER_ERROR;
}
NSL_ERROR_TYPE nsl_saveConfiguration(int handle) {
    log("save", handle);
    if (is("needs_settle") && std::chrono::steady_clock::now() - opened < std::chrono::milliseconds(15))
        return NSL_ERROR_TYPE::NSL_ANSWER_ERROR;
    if (is("normal") || is("ack_warning") || is("already_configured") ||
        (handle > 0 && !is("save_fail"))) return NSL_ERROR_TYPE::NSL_SUCCESS;
    return NSL_ERROR_TYPE::NSL_ANSWER_ERROR;
}
'''


@unittest.skipUnless(shutil.which('g++'), 'C++ compiler required')
class ChangeCameraIpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.directory = Path(cls.temp.name)
        stub = cls.directory / 'fake_sdk.cpp'
        stub.write_text(FAKE_SDK)
        cls.binary = cls.directory / 'change_camera_ip'
        subprocess.run([
            'g++', '-std=c++17', '-pthread', '-I', str(LIB / 'include'),
            '-DNSL_CAMERA_SETTLE_MS=20', '-DNSL_CAMERA_RECONNECT_WINDOW_MS=500',
            '-DNSL_CAMERA_RECONNECT_RETRY_MS=1', '-DNSL_CAMERA_RECONNECT_ATTEMPTS=4',
            str(LIB / 'script/change_camera_ip.cpp'), str(stub), '-o', str(cls.binary)
        ], check=True, capture_output=True, text=True)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def run_writer(self, scenario, endpoint='192.168.2.202', new_ip='192.168.2.201'):
        calls_file = self.directory / 'calls'
        calls_file.unlink(missing_ok=True)
        result = subprocess.run([
            str(self.binary), new_ip, '255.255.255.0', '192.168.2.1', endpoint
        ], env=dict(os.environ, TEST_SCENARIO=scenario, TEST_CALLS=str(calls_file)),
            text=True, capture_output=True, timeout=8)
        calls = calls_file.read_text().splitlines() if calls_file.exists() else []
        return result, calls

    def test_reconnects_and_saves_once_at_new_verified_address(self):
        result, calls = self.run_writer('lost_socket')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(calls, ['open 0', 'get 0', 'set 0', 'save 0', 'close 0',
                                 'open 1', 'get 1', 'save 1', 'close 1'])
        self.assertIn('saved to flash', result.stdout)
        self.assertIn('via Ethernet', result.stdout)

    def test_never_saves_unverified_target_or_different_camera(self):
        for scenario in ('wrong_ip', 'wrong_device'):
            with self.subTest(scenario=scenario):
                result, calls = self.run_writer(scenario)
                self.assertEqual(result.returncode, 3, result.stdout)
                self.assertNotIn('save 1', calls)
                self.assertIn('close 1', calls)
                self.assertNotIn('[DONE]', result.stdout)

    def test_already_changed_address_can_retry_an_unconfirmed_save(self):
        result, calls = self.run_writer('already_retry', endpoint='192.168.2.201')
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('save 1', calls)
        self.assertEqual(sum(call.startswith('set ') for call in calls), 0)

    def test_already_correct_settings_are_saved_without_rebooting(self):
        result, calls = self.run_writer('already_configured', endpoint='192.168.2.201')
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(calls, ['open 0', 'get 0', 'save 0', 'close 0'])

    def test_waits_for_sdk_calibration_reply_after_every_open(self):
        result, calls = self.run_writer('needs_settle')
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('get 1', calls)
        self.assertIn('save 1', calls)

    def test_waits_for_sdk_control_port_after_camera_reboot(self):
        result, calls = self.run_writer('reboot_delay')
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn('save 4', calls)
        self.assertEqual(sum(call.startswith('set ') for call in calls), 1)
        self.assertNotIn('close 1', calls)
        self.assertNotIn('close 2', calls)
        self.assertNotIn('close 3', calls)

    def test_save_failure_is_nonzero_and_does_not_repeat_address_writes(self):
        result, calls = self.run_writer('save_fail')
        self.assertEqual(result.returncode, 3, result.stdout)
        self.assertEqual(sum(call.startswith('set ') for call in calls), 1)
        self.assertEqual(sum(call.startswith('open ') for call in calls), 5)
        self.assertEqual(sum(call.startswith('close ') for call in calls), 5)
        self.assertNotIn('[DONE]', result.stdout)

    def test_usb_success_does_not_switch_transport(self):
        result, calls = self.run_writer('normal', endpoint='N010005C4')
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(sum(call.startswith('open ') for call in calls), 1)
        self.assertIn('via USB', result.stdout)

    def test_usb_save_failure_does_not_open_ethernet(self):
        result, calls = self.run_writer('save_fail', endpoint='N010005C4')
        self.assertEqual(result.returncode, 3, result.stdout)
        self.assertEqual(sum(call.startswith('open ') for call in calls), 1)

    def test_saved_but_missing_set_ack_preserves_separate_status(self):
        result, calls = self.run_writer('ack_warning', endpoint='N010005C4')
        self.assertEqual(result.returncode, 2, result.stdout)
        self.assertIn('Configuration saved to flash', result.stdout)

    def test_invalid_address_is_rejected_before_sdk_access(self):
        result, calls = self.run_writer('normal', new_ip='192.168.2.999')
        self.assertEqual(result.returncode, 1)
        self.assertEqual(calls, [])
