"""Local backend replacement. No serial access and no automatic motor enable.

New instances exit cooperatively; legacy migration may terminate only the
verified port owner, after its API confirms that CAN has been disconnected.
"""
from __future__ import annotations

import ctypes
import hashlib
import json
import os
import secrets
import subprocess
import time
from contextlib import contextmanager
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener


def request_json(base: str, path: str, body: dict | None = None) -> dict:
    data = None if body is None else json.dumps(body).encode('utf-8')
    request = Request(base + path, data=data, headers={'Content-Type': 'application/json'})
    # Never send the local shutdown token through a configured HTTP proxy.
    with build_opener(ProxyHandler({})).open(request, timeout=5) as response:
        return json.load(response)


def kernel_api():
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.CreateMutexW.argtypes = (ctypes.c_void_p, ctypes.c_bool, ctypes.c_wchar_p)
    kernel.CreateMutexW.restype = ctypes.c_void_p
    kernel.WaitForSingleObject.argtypes = (ctypes.c_void_p, ctypes.c_uint32)
    kernel.WaitForSingleObject.restype = ctypes.c_uint32
    kernel.ReleaseMutex.argtypes = (ctypes.c_void_p,)
    kernel.CloseHandle.argtypes = (ctypes.c_void_p,)
    return kernel


@contextmanager
def startup_lock():
    """Serialize double-clicks until the replacement owns the service mutex."""
    if os.name != 'nt':
        # Portable offline test hosts; production launcher runs on Windows.
        yield
        return
    kernel = kernel_api()
    handle = kernel.CreateMutexW(None, False, 'Local\\CyberArmDigitalTwin-Startup-8765')
    if not handle:
        raise ctypes.WinError(ctypes.get_last_error())
    acquired = False
    try:
        acquired = kernel.WaitForSingleObject(handle, 15000) in (0, 0x80)
        if not acquired:
            raise RuntimeError('Backend launch is already in progress')
        yield
    finally:
        if acquired:
            kernel.ReleaseMutex(handle)
        kernel.CloseHandle(handle)


class BackendRegistration:
    def __init__(self, root: Path):
        self.root = root.resolve()
        self.path = self.root / 'tmp' / 'twin_backend_instance.json'
        self.token = secrets.token_hex(32)
        digest = hashlib.sha256()
        for path in sorted([*self.root.glob('*.py'), *self.root.glob('web/*.*')]):
            digest.update(path.name.encode())
            digest.update(path.read_bytes())
        self.identity = {'pid': os.getpid(), 'app_root': str(self.root),
                         'instance_id': secrets.token_hex(16), 'source_revision': digest.hexdigest()}

    def publish(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(f'.{self.identity["instance_id"]}.tmp')
        temporary.write_text(json.dumps({**self.identity, 'token': self.token}), encoding='utf-8')
        temporary.replace(self.path)

    def cleanup(self):
        try:
            record = json.loads(self.path.read_text(encoding='utf-8'))
            if record.get('instance_id') == self.identity['instance_id']:
                self.path.unlink()
        except (OSError, ValueError):
            pass


def _powershell(script: str, values: dict | None = None) -> str:
    env = {**os.environ, **(values or {})}
    result = subprocess.run(['powershell.exe', '-NoProfile', '-NonInteractive', '-Command', script],
                            capture_output=True, text=True, encoding='utf-8', errors='replace',
                            timeout=10, env=env, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    if result.returncode:
        raise RuntimeError('Cannot verify or retire the legacy backend: ' + result.stderr.strip())
    return result.stdout.strip().lstrip('\ufeff')


def legacy_port_owner(port: int) -> dict:
    if os.name != 'nt':
        raise RuntimeError('Legacy replacement is Windows-only')
    return json.loads(_powershell(r'''
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$owners = @(Get-NetTCPConnection -LocalPort ([int]$env:CYBERARM_LAUNCH_PORT) -State Listen | Select-Object -ExpandProperty OwningProcess -Unique)
if ($owners.Count -ne 1) { throw 'Port owner is not unique' }
$p = Get-CimInstance Win32_Process -Filter ("ProcessId=" + $owners[0])
@{pid=$p.ProcessId; command=$p.CommandLine; executable=$p.ExecutablePath; created=$p.CreationDate.ToUniversalTime().ToString('o')} | ConvertTo-Json -Compress
''', {'CYBERARM_LAUNCH_PORT': str(port)}))


def _windows_argv(command: str) -> list[str]:
    shell = ctypes.WinDLL('shell32', use_last_error=True)
    shell.CommandLineToArgvW.argtypes = (ctypes.c_wchar_p, ctypes.POINTER(ctypes.c_int))
    shell.CommandLineToArgvW.restype = ctypes.POINTER(ctypes.c_wchar_p)
    count = ctypes.c_int()
    argv = shell.CommandLineToArgvW(command, ctypes.byref(count))
    if not argv:
        raise ValueError('Cannot parse legacy command line')
    try:
        return [argv[i] for i in range(count.value)]
    finally:
        kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        kernel.LocalFree.argtypes = (ctypes.c_void_p,)
        kernel.LocalFree(argv)


def verify_legacy_owner(owner: dict, state: dict, root: Path, argv: list[str]) -> None:
    """Relative old launchers additionally require this project's exact motor identities."""
    executable = Path(owner['executable']).name.lower()
    args = argv[1:]
    while args and args[0] in ('-u', '-B'):
        args = args[1:]
    if executable not in ('python.exe', 'pythonw.exe') or len(args) != 1:
        raise RuntimeError('Port belongs to a different application')
    script = Path(args[0])
    expected = root.resolve() / 'digital_twin_server.py'
    if script.is_absolute():
        if script.resolve() != expected:
            raise RuntimeError('Port belongs to a different checkout')
    elif args[0] not in ('digital_twin_server.py', '.\\digital_twin_server.py', './digital_twin_server.py'):
        raise RuntimeError('Unrecognized legacy launcher')
    config = json.loads((root / 'classic_dual_config.json').read_text(encoding='utf-8'))
    if not isinstance(state.get('version'), str) or not state['version'].startswith('2.'):
        raise RuntimeError('Port does not identify as a CyberArm backend')
    for key in ('shoulder', 'elbow'):
        actual, wanted = state.get('joints', {}).get(key, {}), config['joints'][key]
        if (actual.get('expected_uid'), actual.get('motor_can_id')) != (wanted['expected_uid'], wanted['motor_can_id']):
            raise RuntimeError('Legacy backend motor identities do not match this project')


def terminate_verified_legacy(owner: dict, port: int):
    # Recheck PID creation time, command line, executable and listener at the
    # point of termination. Never kill by filename or sweep all Python tasks.
    _powershell(r'''
$ErrorActionPreference = 'Stop'
$targetProcessId = [int]$env:CYBERARM_LAUNCH_PID
$p = Get-CimInstance Win32_Process -Filter ("ProcessId=" + $targetProcessId)
if ($null -eq $p) { exit 0 }
if ($p.CreationDate.ToUniversalTime().ToString('o') -ne $env:CYBERARM_LAUNCH_CREATED -or $p.CommandLine -cne $env:CYBERARM_LAUNCH_COMMAND -or $p.ExecutablePath -ine $env:CYBERARM_LAUNCH_EXE) { throw 'Legacy process identity changed' }
$owners = @(Get-NetTCPConnection -LocalPort ([int]$env:CYBERARM_LAUNCH_PORT) -State Listen | Select-Object -ExpandProperty OwningProcess -Unique)
if ($owners.Count -ne 1 -or $owners[0] -ne $targetProcessId) { throw 'Port owner changed' }
# -Confirm:$false handles confirmation preferences; -Force also suppresses
# Stop-Process's separate ownership prompt in a non-interactive host.
# All identity/disconnection checks above remain mandatory.
Stop-Process -Id $targetProcessId -Force -Confirm:$false -ErrorAction Stop
''', {'CYBERARM_LAUNCH_PID': str(owner['pid']), 'CYBERARM_LAUNCH_CREATED': owner['created'],
      'CYBERARM_LAUNCH_COMMAND': owner['command'], 'CYBERARM_LAUNCH_EXE': owner['executable'],
      'CYBERARM_LAUNCH_PORT': str(port)})


def retire_existing_backend(root: Path, base: str, port: int) -> None:
    """Do not take over motion/holding or an unresponsive, unidentified process."""
    state = request_json(base, '/api/state')
    if state.get('motion_active') is not False:
        raise RuntimeError('Restart deferred: motor motion/holding is active')
    try:
        identity = request_json(base, '/api/backend')
    except HTTPError as exc:
        if exc.code != 404:
            raise
    else:
        record = json.loads((root / 'tmp' / 'twin_backend_instance.json').read_text(encoding='utf-8'))
        if (identity.get('app_root') != str(root.resolve()) or
                any(record.get(key) != identity.get(key) for key in ('pid', 'instance_id', 'source_revision', 'app_root'))):
            raise RuntimeError('Backend registration identity mismatch')
        result = request_json(base, '/api/backend/shutdown', {'token': record['token']})
        if result.get('ok') is not True:
            raise RuntimeError('Backend did not accept shutdown')
        return
    # One-time migration for pre-lifecycle versions. A legacy process may be
    # terminated only AFTER its own control API reports serial disconnected.
    owner = legacy_port_owner(port)
    verify_legacy_owner(owner, state, root, _windows_argv(owner['command']))
    if request_json(base, '/api/state').get('motion_active') is not False:
        raise RuntimeError('Restart deferred: legacy motion/holding started')
    result = request_json(base, '/api/disconnect', {})
    if result.get('ok') is not True:
        raise RuntimeError('Legacy disconnect failed; process left untouched')
    final_state = request_json(base, '/api/state')
    if final_state.get('connected') is not False or final_state.get('motion_active') is not False:
        raise RuntimeError('Legacy backend is not disconnected; process left untouched')
    terminate_verified_legacy(owner, port)


def startup_log(root: Path, message: str):
    try:
        folder = root / 'tmp'
        folder.mkdir(parents=True, exist_ok=True)
        with (folder / 'backend_startup.log').open('a', encoding='utf-8') as stream:
            stream.write(time.strftime('%Y-%m-%d %H:%M:%S ') + message + '\n')
    except OSError:
        pass  # Logging failure must not create a user-facing traceback.
