"""Explicitly authorized, bounded live test via the existing dashboard only.

No auto-connect, calibration, safety reset, tuning, or protection bypass.
Stops both joints in finally. Never invoke without current physical safety consent.
"""
from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

BASE = 'http://127.0.0.1:8765'
KEYS = ('shoulder', 'elbow')


def stage_envelopes(starts, targets):
    return {k: (max(-3., min(starts[k], targets[k])-3.),
                min(33., max(starts[k], targets[k])+3.)) for k in KEYS}


def api(path, body=None):
    data = None if body is None else json.dumps(body).encode()
    req = Request(BASE + path, data=data, headers={'Content-Type': 'application/json'})
    try:
        with urlopen(req, timeout=10) as response:
            result = json.load(response)
    except HTTPError as exc:
        result = json.load(exc)
        raise RuntimeError(result.get('error', str(exc))) from exc
    if body is not None and not result.get('ok'):
        raise RuntimeError(result.get('error', 'Request rejected'))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--confirmed-safe', action='store_true', required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--expect-gravity', choices=('off', 'on'), default='off')
    parser.add_argument('--expect-load-compensation', choices=('off', 'on'), default='off')
    args = parser.parse_args()
    result = {'started': datetime.now().isoformat(), 'test': 'relative +5 deg then absolute 30 deg; 0.05 rad/s; 3.5 Nm; hold 5s',
              'stages': [], 'samples': [], 'success': False}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    path = args.output_dir / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_hold_test.json')
    try:
        stopped = api('/api/stop', {})
        if any(stopped['result'].get(k) != 'sent' for k in KEYS):
            raise RuntimeError('Initial stop was not sent to both motors')
        time.sleep(.1)
        state = api('/api/state')
        result['initial'] = state
        if state['version'] != '2.3.3' or not state['connected'] or state['motion_active'] or state['safety_latched']:
            raise RuntimeError('Version/connection/stopped/safety precondition failed')
        if state['gravity']['enabled'] != (args.expect_gravity == 'on'):
            raise RuntimeError('Gravity configuration differs from explicitly requested test; no automatic change')
        if args.expect_gravity == 'on' and not state['gravity']['calibration_valid']:
            raise RuntimeError('Gravity calibration must be valid before this test')
        if state['load_compensation']['enabled'] != (args.expect_load_compensation == 'on'):
            raise RuntimeError('Residual compensation differs from explicitly requested test; no automatic change')
        if not all(state['joints'][k]['verified'] for k in KEYS):
            raise RuntimeError('Both motor UIDs must already be independently verified')
        # First test is small relative to the stopped starting pose.
        if any(state['joints'][k]['actual_deg'] is None or not -3 <= state['joints'][k]['actual_deg'] <= 30
               or state['joints'][k]['feedback_age_s'] is None or state['joints'][k]['feedback_age_s'] > .3 for k in KEYS):
            raise RuntimeError('Starting pose/freshness outside authorized small-move envelope')
        first_targets = {k: min(30., state['joints'][k]['actual_deg']+5.) for k in KEYS}
        for targets in (first_targets, dict.fromkeys(KEYS, 30.)):
            stage = {'targets_deg': targets, 'hold_success': False}
            bounds = stage_envelopes({k: state['joints'][k]['actual_deg'] for k in KEYS}, targets)
            stage['angle_envelopes_deg'] = bounds
            result['stages'].append(stage)
            print(f'BEGIN targets={targets} deg, speed=0.05 rad/s, limit=3.5 Nm', flush=True)
            reply = api('/api/move', {'targets_deg': targets,
                                     'speeds_rad_s': {k: .05 for k in KEYS},
                                     'torque_limits_nm': {k: 3.5 for k in KEYS}})
            stage['planned_duration_s'] = reply['result']
            deadline = time.monotonic() + float(reply['result']) + 15
            holding_since = None
            position_window = []
            last_print = 0.0
            while time.monotonic() < deadline:
                state = api('/api/state')
                now = time.monotonic()
                sample = {'time': datetime.now().isoformat(), 'targets_deg': targets,
                          'phase': state['control_phase'], 'safety_reason': state['safety_reason'],
                          'joints': state['joints']}
                result['samples'].append(sample)
                if state['safety_latched'] or not state['motion_active']:
                    raise RuntimeError('Controller stopped: ' + state['safety_reason'])
                for key in KEYS:
                    joint = state['joints'][key]
                    if not joint['verified'] or joint['feedback_age_s'] is None or joint['feedback_age_s'] > .3:
                        raise RuntimeError(key + ': identity or fresh feedback lost')
                    if joint['fault_bits'] or joint['temperature_c'] > 60 or joint['mode_status'] != 2:
                        raise RuntimeError(key + ': fault, temperature or mode changed')
                    if not bounds[key][0] <= joint['actual_deg'] <= bounds[key][1]:
                        raise RuntimeError(key + ': exceeded authorized test angle envelope')
                # Independently verify encoder stability instead of demanding
                # five seconds with not a single noisy velocity sample.
                position_window.append((now, {k: state['joints'][k]['actual_deg'] for k in KEYS}))
                position_window = [s for s in position_window if now-s[0] <= .65]
                good_hold = (state['control_phase'] == 'controlling'
                    and all(state['joints'][k]['on_target'] for k in KEYS)
                    and len(position_window) >= 4
                    and now-position_window[0][0] >= .4
                    and all(b[0]-a[0] <= .3 for a,b in zip(position_window, position_window[1:]))
                    and all(abs(state['joints'][k]['actual_deg'] - targets[k]) <= 1.5
                        and max(s[1][k] for s in position_window)-min(s[1][k] for s in position_window) <= .1 for k in KEYS))
                if good_hold:
                    holding_since = now if holding_since is None else holding_since
                    if now - holding_since >= 5:
                        stage['hold_success'] = True
                        stage['final'] = sample
                        print(f'PASS targets={targets}: actual arrival and 5s continuous hold', flush=True)
                        break
                else:
                    holding_since = None
                if now - last_print >= 1:
                    print(json.dumps({'phase': state['control_phase'],
                        **{k: round(state['joints'][k]['actual_deg'], 2) for k in KEYS}}, ensure_ascii=False), flush=True)
                    last_print = now
                time.sleep(.1)
            if not stage['hold_success']:
                raise RuntimeError('Did not establish 5s continuous hold before test deadline')
        result['success'] = True
    except Exception as exc:
        result['error'] = str(exc)
        print('FAILED: ' + str(exc), flush=True)
    finally:
        try:
            result['stop'] = api('/api/stop', {})
            if any(result['stop']['result'].get(k) != 'sent' for k in KEYS):
                result['stop_error'] = 'One or more stop sends failed; physical emergency stop required'
        except Exception as exc:
            result['stop_error'] = str(exc)
            print('STOP FAILED: use physical emergency stop: ' + str(exc), flush=True)
        result['finished'] = datetime.now().isoformat()
        try:
            result['diagnostics'] = api('/api/diagnostics')
        except Exception as exc:
            result['diagnostics_error'] = str(exc)
        path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        print('REPORT ' + str(path), flush=True)
        print('STOP ' + json.dumps(result.get('stop', {}).get('result', result.get('stop_error')), ensure_ascii=False), flush=True)
    return 0 if result['success'] and not result.get('stop_error') else 1


if __name__ == '__main__':
    raise SystemExit(main())
