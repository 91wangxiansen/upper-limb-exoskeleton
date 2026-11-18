"""Offline lifecycle checks; never open a serial port or target the live service."""
import copy
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import unittest
from contextlib import ExitStack, nullcontext
from pathlib import Path
from unittest.mock import Mock, patch
from urllib.error import HTTPError
from http.server import ThreadingHTTPServer

import backend_lifecycle as lifecycle
import digital_twin_server as server_module
from classic_dual_controller import DEFAULT_CONFIG, ClassicConfigStore


class RegistrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        ClassicConfigStore(self.root / 'classic_dual_config.json').save(copy.deepcopy(DEFAULT_CONFIG))

    def state(self):
        return {'version': '2.3.0', 'connected': False, 'motion_active': False,
                'joints': copy.deepcopy(DEFAULT_CONFIG['joints'])}

    def test_registration_has_private_token_and_cleanup_does_not_remove_successor(self):
        registration = lifecycle.BackendRegistration(self.root)
        registration.publish()
        record = json.loads(registration.path.read_text())
        self.assertNotIn('token', registration.identity)
        self.assertEqual(len(record['token']), 64)
        successor = lifecycle.BackendRegistration(self.root)
        successor.publish()
        registration.cleanup()
        self.assertTrue(successor.path.exists())
        successor.cleanup()
        self.assertFalse(successor.path.exists())

    def test_new_instance_cooperatively_retires_only_matching_registration(self):
        registration = lifecycle.BackendRegistration(self.root)
        registration.publish()
        with patch.object(lifecycle, 'request_json', side_effect=[self.state(), registration.identity, {'ok': True}]) as request, \
                patch.object(lifecycle, 'terminate_verified_legacy') as kill:
            lifecycle.retire_existing_backend(self.root, 'http://127.0.0.1:12345', 12345)
        self.assertEqual(request.call_args.args[1:], ('/api/backend/shutdown', {'token': registration.token}))
        kill.assert_not_called()

    def test_holding_and_registration_mismatch_never_send_shutdown(self):
        registration = lifecycle.BackendRegistration(self.root)
        registration.publish()
        for replies in ([{**self.state(), 'motion_active': True}],
                        [self.state(), {**registration.identity, 'instance_id': 'wrong'}],
                        [self.state(), {**registration.identity, 'app_root': 'another project'}]):
            with self.subTest(replies=replies), patch.object(lifecycle, 'request_json', side_effect=replies) as request:
                with self.assertRaises(RuntimeError):
                    lifecycle.retire_existing_backend(self.root, 'http://127.0.0.1:12345', 12345)
                self.assertFalse(any(call.args[1].endswith('/shutdown') for call in request.call_args_list))

    def test_legacy_owner_requires_exact_script_and_motor_identities(self):
        owner = {'executable': 'C:/Python311/python.exe'}
        lifecycle.verify_legacy_owner(owner, self.state(), self.root, ['python.exe', '-u', 'digital_twin_server.py'])
        lifecycle.verify_legacy_owner(owner, self.state(), self.root,
                                      ['python.exe', str(self.root / 'digital_twin_server.py')])
        variants = [(['python.exe', 'unrelated.py'], self.state()),
                    (['python.exe', '-c', 'digital_twin_server.py'], self.state()),
                    (['python.exe', str(self.root.parent / 'other' / 'digital_twin_server.py')], self.state())]
        mismatch = self.state()
        mismatch['joints']['elbow']['expected_uid'] = 'WRONG'
        variants.append((['python.exe', 'digital_twin_server.py'], mismatch))
        for argv, state in variants:
            with self.subTest(argv=argv), self.assertRaises(RuntimeError):
                lifecycle.verify_legacy_owner(owner, state, self.root, argv)

    def legacy_run(self, *, disconnect_ok=True, connected=False, became_active=False):
        missing = HTTPError('local', 404, 'legacy route absent', {}, None)
        owner = {'pid': 123, 'command': 'python.exe digital_twin_server.py',
                 'executable': 'C:/Python311/python.exe', 'created': 'timestamp'}
        replies = [self.state(), missing, {**self.state(), 'motion_active': became_active},
                   {'ok': disconnect_ok}, {**self.state(), 'connected': connected}]
        with patch.object(lifecycle, 'request_json', side_effect=replies), \
                patch.object(lifecycle, 'legacy_port_owner', return_value=owner), \
                patch.object(lifecycle, '_windows_argv', return_value=['python.exe', 'digital_twin_server.py']), \
                patch.object(lifecycle, 'terminate_verified_legacy') as kill:
            try:
                lifecycle.retire_existing_backend(self.root, 'http://127.0.0.1:12345', 12345)
            except RuntimeError:
                if disconnect_ok and not connected and not became_active:
                    raise
            return kill.call_args_list

    def test_legacy_migration_requires_successful_disconnect_and_no_new_motion(self):
        self.assertEqual(len(self.legacy_run()), 1)
        self.assertEqual(self.legacy_run(disconnect_ok=False), [])
        self.assertEqual(self.legacy_run(connected=True), [])
        self.assertEqual(self.legacy_run(became_active=True), [])


class LauncherTests(unittest.TestCase):
    def launch(self, mutex_results, retirement_error=None, cleanup_error=None):
        with ExitStack() as stack:
            mocks = {}
            targets = ['startup_lock', 'acquire_single_instance', 'retire_existing_backend',
                       'ThreadingHTTPServer', 'BackendRegistration', 'CyberArmTwin',
                       'install_console_cleanup', 'remove_console_cleanup',
                       'release_single_instance', 'startup_log', 'threading.Timer', 'webbrowser.open']
            for name in targets:
                mocks[name] = stack.enter_context(patch('digital_twin_server.' + name))
            stack.enter_context(patch.object(server_module, 'TWIN', Mock()))
            mocks['startup_lock'].return_value = nullcontext()
            mocks['acquire_single_instance'].side_effect = mutex_results
            mocks['retire_existing_backend'].side_effect = retirement_error
            mocks['ThreadingHTTPServer'].return_value.serve_forever.side_effect = KeyboardInterrupt
            mocks['CyberArmTwin'].return_value.shutdown.side_effect = cleanup_error
            mocks['BackendRegistration'].return_value.identity = {'source_revision': 'current-source'}
            server_module.main()
            return mocks

    def test_launch_replaces_old_instance_and_starts_fresh_process_code_path(self):
        mocks = self.launch([False, True])
        mocks['retire_existing_backend'].assert_called_once()
        mocks['CyberArmTwin'].assert_called_once_with()
        mocks['ThreadingHTTPServer'].return_value.serve_forever.assert_called_once()
        mocks['BackendRegistration'].return_value.publish.assert_called_once()
        mocks['release_single_instance'].assert_called_once()
        mocks['webbrowser.open'].assert_not_called()  # Never reopen the stale backend.

    def test_takeover_failure_does_not_open_old_page_or_start_duplicate(self):
        mocks = self.launch([False], RuntimeError('holding'))
        mocks['CyberArmTwin'].assert_not_called()
        mocks['ThreadingHTTPServer'].assert_not_called()
        mocks['threading.Timer'].assert_not_called()
        mocks['webbrowser.open'].assert_not_called()
        mocks['startup_log'].assert_called_once()

    def test_first_start_releases_registration_and_mutex(self):
        mocks = self.launch([True])
        mocks['retire_existing_backend'].assert_not_called()
        mocks['CyberArmTwin'].return_value.shutdown.assert_called_once()
        mocks['ThreadingHTTPServer'].return_value.server_close.assert_called_once()
        mocks['BackendRegistration'].return_value.cleanup.assert_called_once()

    def test_cleanup_failure_does_not_skip_port_record_or_mutex_release(self):
        mocks = self.launch([True], cleanup_error=OSError('simulated close failure'))
        mocks['ThreadingHTTPServer'].return_value.server_close.assert_called_once()
        mocks['BackendRegistration'].return_value.cleanup.assert_called_once()
        mocks['release_single_instance'].assert_called_once()


class LegacyTerminationTests(unittest.TestCase):
    def test_noninteractive_exit_flags_do_not_remove_identity_checks(self):
        owner = {'pid': 123, 'command': 'python.exe digital_twin_server.py',
                 'executable': 'C:/Python311/python.exe', 'created': 'timestamp'}
        with patch.object(lifecycle, '_powershell', return_value='') as execute:
            lifecycle.terminate_verified_legacy(owner, 12345)
        script, values = execute.call_args.args
        command = 'Stop-Process -Id $targetProcessId -Force -Confirm:$false -ErrorAction Stop'
        self.assertIn(command, script)
        for check in ('CreationDate', 'CommandLine -cne', 'ExecutablePath -ine', 'Port owner changed'):
            self.assertLess(script.index(check), script.index(command))
        self.assertEqual(values['CYBERARM_LAUNCH_PID'], '123')
        self.assertEqual(values['CYBERARM_LAUNCH_PORT'], '12345')

    @unittest.skipUnless(os.name == 'nt' and shutil.which('powershell.exe'), 'Windows PowerShell integration')
    def test_real_noninteractive_stop_exits_only_owned_test_child(self):
        # Metadata providers are controlled because this sandbox may forbid
        # CIM/network inventory. Stop-Process itself is REAL Windows PowerShell
        # and targets ONLY the inert child created here, never a real backend.
        child = subprocess.Popen([sys.executable, '-u', '-c',
            'import sys; print("READY", flush=True); sys.stdin.read()'],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        try:
            self.assertEqual(child.stdout.readline().strip(), 'READY')
            execute_real = lifecycle._powershell
            created = execute_real("(Get-Process -Id ([int]$env:CYBERARM_TEST_PID)).StartTime.ToUniversalTime().ToString('o')",
                                   {'CYBERARM_TEST_PID': str(child.pid)})
            owner = {'pid': child.pid, 'created': created,
                     'command': 'inert regression child', 'executable': sys.executable}
            prelude = r'''
$ConfirmPreference = 'High'
function Get-CimInstance {
    param($ClassName, $Filter)
    $owned = Get-Process -Id ([int]$env:CYBERARM_LAUNCH_PID) -ErrorAction Stop
    [pscustomobject]@{ProcessId=$owned.Id; CreationDate=$owned.StartTime; CommandLine=$env:CYBERARM_LAUNCH_COMMAND; ExecutablePath=$env:CYBERARM_LAUNCH_EXE}
}
function Get-NetTCPConnection {
    param($LocalPort, $State)
    [pscustomobject]@{OwningProcess=[int]$env:CYBERARM_LAUNCH_PID}
}
'''
            with patch.object(lifecycle, '_powershell', side_effect=lambda script, values: execute_real(prelude + script, values)):
                # Wrong creation time still blocks termination, even with Force.
                with self.assertRaisesRegex(RuntimeError, 'identity changed'):
                    lifecycle.terminate_verified_legacy({**owner, 'created': 'wrong'}, 12345)
                self.assertIsNone(child.poll())
                lifecycle.terminate_verified_legacy(owner, 12345)
            child.wait(timeout=5)
            self.assertIsNotNone(child.returncode)
        finally:
            if child.poll() is None:
                child.terminate()
                child.wait(timeout=5)
            for pipe in (child.stdin, child.stdout, child.stderr):
                pipe.close()


class ShutdownHttpTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        self.twin = server_module.CyberArmTwin(root / 'config.json')
        self.registration = lifecycle.BackendRegistration(root)
        self.http = ThreadingHTTPServer(('127.0.0.1', 0), server_module.DashboardHandler)
        self.http.backend_registration = self.registration
        port = self.http.server_address[1]
        self.base = f'http://127.0.0.1:{port}'
        patches = [patch.object(server_module, 'TWIN', self.twin), patch.object(server_module, 'PORT', port)]
        for item in patches:
            item.start()
            self.addCleanup(item.stop)
        self.thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.close)

    def close(self):
        self.http.shutdown()
        self.http.server_close()
        self.thread.join(timeout=2)

    def test_authenticated_shutdown_exits_without_connecting_serial(self):
        identity = lifecycle.request_json(self.base, '/api/backend')
        self.assertEqual(identity, self.registration.identity)
        self.assertNotIn('token', identity)
        with patch.object(server_module.serial, 'Serial') as serial:
            result = lifecycle.request_json(self.base, '/api/backend/shutdown', {'token': self.registration.token})
        self.assertTrue(result['ok'])
        self.thread.join(timeout=2)
        self.assertFalse(self.thread.is_alive())
        serial.assert_not_called()
        self.assertTrue(self.twin.shutting_down)

    def test_wrong_token_or_foreign_origin_never_disconnects(self):
        for body in ({'token': 'wrong'}, {}, {'token': self.registration.token, 'extra': True}):
            with self.assertRaises(HTTPError) as caught:
                lifecycle.request_json(self.base, '/api/backend/shutdown', body)
            self.assertEqual(caught.exception.code, 403)
        self.assertFalse(self.twin.shutting_down)
        self.assertTrue(self.thread.is_alive())

    def test_motion_holding_blocks_cooperative_exit(self):
        controller = Mock()
        controller.state_lock = threading.RLock()
        controller.motion_plan = object()
        self.twin.controller = controller
        with self.assertRaises(HTTPError) as caught:
            lifecycle.request_json(self.base, '/api/backend/shutdown', {'token': self.registration.token})
        self.assertEqual(caught.exception.code, 409)
        controller.shutdown.assert_not_called()
        self.assertFalse(self.twin.shutting_down)

    def test_shutting_down_instance_rejects_new_connect_or_move(self):
        self.twin.shutting_down = True
        for path in ('/api/connect', '/api/move'):
            with self.assertRaises(HTTPError) as caught:
                lifecycle.request_json(self.base, path, {})
            self.assertEqual(caught.exception.code, 409)


if __name__ == '__main__':
    unittest.main()
