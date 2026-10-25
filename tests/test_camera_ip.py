"""Camera IP provisioning and SDK reconnect/save behaviour without real hardware."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


MODULE = Path(__file__).resolve().parents[1]
SCRIPT = MODULE/'setup/setup_fleet_edge.bash'
LIB = MODULE/'roboscan_nsl3130/nsl_lib'


COMMAND_ROUTER = r'''#!/usr/bin/python3
import json, os, subprocess, sys
from pathlib import Path
name = Path(sys.argv[0]).name
args = sys.argv[1:]
if name == 'python3':
    if args and args[0].endswith('detect_camera_id.py'):
        print(os.environ.get('TEST_USB_SERIAL', ''))
        sys.exit(0)
    os.execv('/usr/bin/python3', ['/usr/bin/python3', *args])
state_path = Path(os.environ['TEST_NETWORK_STATE'])
state = json.loads(state_path.read_text())
state['calls'].append([name, *args])
def finish(status=0, output=''):
    state_path.write_text(json.dumps(state))
    if output: print(output)
    sys.exit(status)
if name == 'sudo':
    if args[:1] == ['-n']: args = args[1:]
    if args[:1] == ['ip']:
        state_path.write_text(json.dumps(state))
        os.execvp('ip', args)
    finish()
if name == 'sleep': finish()
if name == 'ping':
    address = args[-1]
    if address == os.environ['NSL_CAMERA_IP']:
        finish(0 if state['target_reachable'] else 1)
    if address == os.environ['NSL_FACTORY_CAMERA_IP']:
        reachable = os.environ.get('TEST_CURRENT_REACHABLE', '1') == '1' or state['added_route']
        finish(0 if reachable else 1)
    finish(1)
if name == 'ros2':
    state['writes'] += 1
    state['target_reachable'] = os.environ.get('TEST_TARGET_AFTER_WRITE', '1') == '1'
    status = int(os.environ.get('TEST_WRITE_STATUS', '0'))
    output = '[INFO] Camera current IP : '+os.environ['NSL_FACTORY_CAMERA_IP']+' mask=255.255.255.0 gw=192.168.2.1\n'
    output += '[OK] Configuration saved to flash.' if status in (0, 2) else '[ERROR] nsl_saveConfiguration returned error.'
    finish(status, output)
if name == 'ip':
    if args[:5] == ['-4', '-o', 'addr', 'show', 'dev']:
        finish(output='\n'.join('3: ethcam inet '+a+'/24' for a in state['addresses']))
    if args == ['-4', '-o', 'addr', 'show']:
        finish(output='2: ethfleet inet 192.168.3.161/24\n'+'\n'.join('3: ethcam inet '+a+'/24' for a in state['addresses']))
    if args[:5] == ['-4', 'route', 'show', 'exact', os.environ['NSL_FACTORY_CAMERA_IP']+'/32']:
        finish(output=state['existing_route'])
    if args[:2] == ['addr', 'add']:
        address = args[2].split('/')[0]
        if address in state['addresses']: finish(2)
        state['addresses'].append(address)
        finish()
    if args[:2] == ['addr', 'del']:
        address = args[2].split('/')[0]
        if address in state['addresses']: state['addresses'].remove(address)
        finish()
    if args[:2] == ['route', 'add']:
        if os.environ.get('TEST_ROUTE_ADD_FAIL') == '1': finish(2)
        state['added_route'] = True
        finish()
    if args[:2] == ['route', 'del']:
        state['added_route'] = False
        finish()
    finish(4, 'Unexpected fake ip arguments: '+repr(args))
finish(4, 'Unexpected command '+name)
'''


class CameraIpSetupTests(unittest.TestCase):
    def run_recovery(self, *, factory_host='192.168.2.191', addresses=None,
                     existing_route='', mode='--set-camera-ip', **options):
        with tempfile.TemporaryDirectory() as directory:
            temporary = Path(directory)
            command_dir = temporary / 'bin'
            command_dir.mkdir()
            router = command_dir / 'router'
            router.write_text(COMMAND_ROUTER)
            router.chmod(0o755)
            for name in ('sudo', 'ip', 'ping', 'ros2', 'python3', 'sleep'):
                (command_dir / name).symlink_to(router)
            state_file = temporary / 'network.json'
            state_file.write_text(json.dumps({
                'addresses': addresses or ['192.168.2.100'],
                'existing_route': existing_route, 'added_route': False,
                'target_reachable': False, 'writes': 0, 'calls': []}))
            # Environment setup is stubbed too, so neither real ROS executable
            # paths nor the user's DDS configuration are loaded by these tests.
            startup = temporary / 'bash_env'
            startup.write_text('source() { :; }\n')
            environment = os.environ.copy()
            environment.update(
                BASH_ENV=str(startup), PATH=str(command_dir)+os.pathsep+environment['PATH'],
                TEST_NETWORK_STATE=str(state_file), NSL_SETUP_IP='192.168.3.161',
                NSL_FLEET_ROLE='edge', NSL_CAMERA_IP='192.168.2.101',
                NSL_FACTORY_CAMERA_IP='192.168.2.202', NSL_FACTORY_HOST_IP=factory_host,
                NSL_CAMERA_APPLY_WAIT_SEC='1')
            environment.update({key: str(value) for key, value in options.items()})
            result = subprocess.run(['bash', str(SCRIPT), mode],
                                    env=environment, text=True, capture_output=True, timeout=8)
            return result, json.loads(state_file.read_text())

    def test_all_edges_share_camera_addresses_and_keep_distinct_namespaces(self):
        for octet in range(161, 166):
            with self.subTest(octet=octet):
                result, state = self.run_recovery(
                    mode='--check', NSL_SETUP_IP=f'192.168.3.{octet}', NSL_CAMERA_IP='auto')
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn(f'namespace /cam_{octet}', result.stdout)
                self.assertIn('camera NIC 192.168.2.100, camera 192.168.2.101', result.stdout)
                self.assertEqual(self.mutations(state), [])
                self.assertEqual(state['writes'], 0)

    def test_camera_cannot_use_the_nic_address(self):
        result, state = self.run_recovery(NSL_CAMERA_IP='192.168.2.100')
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn('must differ from the camera NIC IP', result.stderr)
        self.assertEqual(self.mutations(state), [])
        self.assertEqual(state['writes'], 0)

    def test_migration_rejects_target_still_owned_by_the_old_nic(self):
        result, state = self.run_recovery(addresses=['192.168.2.101'])
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn('must differ from the camera NIC IP', result.stderr)
        self.assertEqual(self.mutations(state), [])
        self.assertEqual(state['writes'], 0)

    def test_migration_rejects_target_left_as_a_secondary_nic_address(self):
        result, state = self.run_recovery(addresses=['192.168.2.100', '192.168.2.101'])
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn('must differ from the camera NIC IP', result.stderr)
        self.assertEqual(self.mutations(state), [])
        self.assertEqual(state['writes'], 0)

    def test_explicit_alternative_camera_address_is_still_supported(self):
        result, state = self.run_recovery(NSL_CAMERA_IP='192.168.2.203')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('target camera IP 192.168.2.203', result.stdout)
        self.assertEqual(state['writes'], 1)
        self.assertEqual(self.mutations(state), [])

    @staticmethod
    def mutations(state):
        return [call for call in state['calls'] if call[0] == 'ip'
                and len(call) > 2 and call[1:3] in
                (['addr', 'add'], ['addr', 'del'], ['route', 'add'], ['route', 'del'])]

    def test_reachable_current_ip_preserves_static_nic_even_if_factory_host_matches(self):
        result, state = self.run_recovery(factory_host='192.168.2.100')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(state['addresses'], ['192.168.2.100'])
        self.assertEqual(self.mutations(state), [])
        self.assertEqual(state['writes'], 1)

    def test_factory_route_reuses_preexisting_address_without_deleting_it(self):
        result, state = self.run_recovery(factory_host='192.168.2.100', TEST_CURRENT_REACHABLE=0)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(state['addresses'], ['192.168.2.100'])
        self.assertEqual([call[1:3] for call in self.mutations(state)],
                         [['route', 'add'], ['route', 'del']])
        self.assertFalse(state['added_route'])

    def test_factory_cleanup_removes_only_the_alias_and_route_it_added(self):
        result, state = self.run_recovery(TEST_CURRENT_REACHABLE=0)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(state['addresses'], ['192.168.2.100'])
        self.assertEqual([call[1:3] for call in self.mutations(state)],
                         [['addr', 'add'], ['route', 'add'], ['route', 'del'], ['addr', 'del']])
        self.assertFalse(state['added_route'])

    def test_existing_host_route_is_preserved_without_a_write(self):
        route = '192.168.2.202 dev ethother src 192.168.2.111'
        result, state = self.run_recovery(existing_route=route, TEST_CURRENT_REACHABLE=0)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertIn('existing host route', result.stderr)
        self.assertEqual(state['existing_route'], route)
        self.assertEqual(self.mutations(state), [])
        self.assertEqual(state['writes'], 0)

    def test_ping_success_does_not_hide_failed_flash_save(self):
        result, state = self.run_recovery(TEST_WRITE_STATUS=3)
        self.assertEqual(result.returncode, 3, result.stderr)
        self.assertTrue(state['target_reachable'])
        self.assertIn('flash save was not confirmed', result.stderr)
        self.assertNotIn('IP write via 192.168.2.202 saved', result.stdout)
        self.assertEqual(state['writes'], 1)

    def test_saved_but_offline_camera_requires_restart_and_nonzero_strict_status(self):
        result, state = self.run_recovery(TEST_TARGET_AFTER_WRITE=0)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn('Power-cycle the camera', result.stdout)
        self.assertEqual(state['writes'], 1)

    def test_legacy_missing_set_ack_is_accepted_when_save_and_ping_succeed(self):
        result, state = self.run_recovery(TEST_WRITE_STATUS=2)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(state['target_reachable'])
        self.assertEqual(state['writes'], 1)

    def test_failed_route_add_cleans_own_alias_and_keeps_primary(self):
        result, state = self.run_recovery(TEST_CURRENT_REACHABLE=0, TEST_ROUTE_ADD_FAIL=1)
        self.assertEqual(result.returncode, 1, result.stderr)
        self.assertEqual(state['addresses'], ['192.168.2.100'])
        self.assertEqual([call[1:3] for call in self.mutations(state)],
                         [['addr', 'add'], ['route', 'add'], ['addr', 'del']])
        self.assertEqual(state['writes'], 0)

    def test_usb_saved_but_offline_does_not_repeat_ip_write_over_ethernet(self):
        result, state = self.run_recovery(TEST_USB_SERIAL='N008B060D', TEST_TARGET_AFTER_WRITE=0)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(state['writes'], 1)
        self.assertEqual(self.mutations(state), [])


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
