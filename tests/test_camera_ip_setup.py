"""Exercise IP recovery with fake network/SDK commands; never change the host."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'setup/setup_fleet_edge.bash'

# The command router keeps an independent fake NIC, route and camera state.
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
                     existing_route='', **options):
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
                'addresses': addresses or ['192.168.2.101'],
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
                NSL_FLEET_ROLE='edge', NSL_CAMERA_IP='192.168.2.201',
                NSL_FACTORY_CAMERA_IP='192.168.2.202', NSL_FACTORY_HOST_IP=factory_host,
                NSL_CAMERA_APPLY_WAIT_SEC='1')
            environment.update({key: str(value) for key, value in options.items()})
            result = subprocess.run(['bash', str(SCRIPT), '--set-camera-ip'],
                                    env=environment, text=True, capture_output=True, timeout=8)
            return result, json.loads(state_file.read_text())

    @staticmethod
    def mutations(state):
        return [call for call in state['calls'] if call[0] == 'ip'
                and len(call) > 2 and call[1:3] in
                (['addr', 'add'], ['addr', 'del'], ['route', 'add'], ['route', 'del'])]

    def test_reachable_current_ip_preserves_static_nic_even_if_factory_host_matches(self):
        result, state = self.run_recovery(factory_host='192.168.2.101')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(state['addresses'], ['192.168.2.101'])
        self.assertEqual(self.mutations(state), [])
        self.assertEqual(state['writes'], 1)

    def test_factory_route_reuses_preexisting_address_without_deleting_it(self):
        result, state = self.run_recovery(factory_host='192.168.2.101', TEST_CURRENT_REACHABLE=0)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(state['addresses'], ['192.168.2.101'])
        self.assertEqual([call[1:3] for call in self.mutations(state)],
                         [['route', 'add'], ['route', 'del']])
        self.assertFalse(state['added_route'])

    def test_factory_cleanup_removes_only_the_alias_and_route_it_added(self):
        result, state = self.run_recovery(TEST_CURRENT_REACHABLE=0)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(state['addresses'], ['192.168.2.101'])
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
        self.assertEqual(state['addresses'], ['192.168.2.101'])
        self.assertEqual([call[1:3] for call in self.mutations(state)],
                         [['addr', 'add'], ['route', 'add'], ['addr', 'del']])
        self.assertEqual(state['writes'], 0)

    def test_usb_saved_but_offline_does_not_repeat_ip_write_over_ethernet(self):
        result, state = self.run_recovery(TEST_USB_SERIAL='N008B060D', TEST_TARGET_AFTER_WRITE=0)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(state['writes'], 1)
        self.assertEqual(self.mutations(state), [])


if __name__ == '__main__':
    unittest.main()
