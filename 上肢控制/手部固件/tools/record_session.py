"""Read-only state recorder. Never writes commands or saves access credentials."""
import argparse
import csv
import json
import time
import urllib.request
from datetime import datetime, timezone, timedelta
from pathlib import Path

STATE_KEYS = ('epoch', 'armed', 'ready', 'gripping', 'fault', 'leaseMs', 'heartbeatAgeMs', 'contactFilter')
FINGER_KEYS = ('id', 'online', 'p', 'load', 'speed', 'speedSet', 'temp', 'v', 'phase',
               'alarm', 'readError', 'failures', 'mode', 'open', 'close', 'limit', 'q',
               'contactRawLoad', 'contactFilteredLoad', 'contactVotes', 'contactNoProgressMs',
               'contactPositionError', 'commandedP', 'contactSampleMs')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://192.168.4.1/api/state')
    parser.add_argument('--seconds', type=float, default=600)
    parser.add_argument('--interval', type=float, default=0.2)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    if args.seconds <= 0 or args.interval < 0.1:
        parser.error('seconds must be positive; interval must be at least 0.1')
    folder = Path(args.out)
    folder.mkdir(parents=True, exist_ok=True)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    start = time.monotonic()
    stats = {'samples': 0, 'errors': 0, 'fingers': {}, 'running': True}
    fields = ['time', 'elapsed_s', 'latency_ms', *STATE_KEYS, 'index', *FINGER_KEYS]
    with (folder / 'states.jsonl').open('w', encoding='utf-8') as raw, \
            (folder / 'fingers.csv').open('w', encoding='utf-8-sig', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fields)
        writer.writeheader()
        while time.monotonic() - start < args.seconds and not (folder / 'STOP').exists():
            began = time.monotonic()
            stamp = datetime.now(timezone(timedelta(hours=8))).isoformat(timespec='milliseconds')
            base = {'time': stamp, 'elapsed_s': round(began-start, 3)}
            try:
                with opener.open(args.url, timeout=1.8) as response:
                    state = json.load(response)
                record = {**base, 'latency_ms': round((time.monotonic()-began)*1000, 1),
                          **{k: state.get(k) for k in STATE_KEYS},
                          'fingers': [{k: f.get(k) for k in FINGER_KEYS} for f in state['fingers']]}
                for i, f in enumerate(record['fingers']):
                    writer.writerow({**{k: v for k, v in record.items() if k != 'fingers'}, 'index': i, **f})
                    if f['online'] and f['readError'] == 0 and isinstance(f['load'], (int, float)):
                        st = stats['fingers'].setdefault(str(f['id']), {'valid': 0, 'min': f['load'], 'max': f['load'], 'negative': 0, 'phases': {}})
                        st['valid'] += 1
                        st['min'] = min(st['min'], f['load'])
                        st['max'] = max(st['max'], f['load'])
                        st['negative'] += int(f['load'] < 0)
                        st['phases'][f['phase']] = st['phases'].get(f['phase'], 0) + 1
                stats['samples'] += 1
                if stats['samples'] == 1:
                    print('Recording started: ' + str(folder.resolve()), flush=True)
            except Exception as error:
                # Save only error type; arbitrary server responses may contain credentials.
                record = {**base, 'error': type(error).__name__}
                stats['errors'] += 1
            raw.write(json.dumps(record, ensure_ascii=False) + '\n')
            raw.flush()
            csvfile.flush()
            stats['last_time'] = stamp
            (folder / 'summary.json').write_text(json.dumps(stats, ensure_ascii=False, indent=2), encoding='utf-8')
            time.sleep(max(0, args.interval-(time.monotonic()-began)))
    stats['running'] = False
    (folder / 'summary.json').write_text(json.dumps(stats, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(stats, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
