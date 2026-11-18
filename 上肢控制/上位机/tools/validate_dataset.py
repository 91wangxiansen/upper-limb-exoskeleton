"""Validate a finalized CyberArm v1 dataset directory or ZIP; never access hardware."""
from __future__ import annotations
import argparse
import csv
import hashlib
import io
import json
import math
import sys
import zipfile
from contextlib import contextmanager
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dataset_recorder import COMMON, FIELDS, PACKAGE_NAMES, SCHEMA_VERSION, utc_timestamp


@contextmanager
def dataset_files(path: Path):
    if path.is_dir():
        for name in PACKAGE_NAMES:
            if (path/name).is_symlink():
                raise ValueError('Dataset contains a symbolic link')
        yield lambda name: (path/name).open('rb')
    else:
        with zipfile.ZipFile(path) as archive:
            names = archive.namelist()
            if len(names) != len(set(names)):
                raise ValueError('Duplicate ZIP entry')
            metadata_names = [name for name in names if name.endswith('/metadata.json')]
            if len(metadata_names) != 1:
                raise ValueError('Expected one dataset root')
            prefix = metadata_names[0].removesuffix('metadata.json')
            if set(names) != {prefix+name for name in PACKAGE_NAMES}:
                raise ValueError('Unexpected or missing package files')
            yield lambda name: archive.open(prefix+name)


def validate(path: Path) -> dict:
    counts = {}
    with dataset_files(path) as open_file:
        with open_file('metadata.json') as handle:
            metadata = json.load(handle)
        with open_file('schema.json') as handle:
            schema = json.load(handle)
        if metadata.get('schema_version') != SCHEMA_VERSION or schema.get('schema_version') != SCHEMA_VERSION:
            raise ValueError('Unsupported dataset schema')
        for name in PACKAGE_NAMES:
            if name == 'metadata.json':
                continue
            expected = metadata.get('sha256',{}).get(name)
            digest = hashlib.sha256()
            with open_file(name) as handle:
                for block in iter(lambda: handle.read(1024*1024), b''):
                    digest.update(block)
            if expected != digest.hexdigest():
                raise ValueError(f'Checksum mismatch: {name}')
        for kind, fields in FIELDS.items():
            columns = {**COMMON, **fields}
            definitions = schema['tables'][f'{kind}.csv']
            if [item['name'] for item in definitions] != list(columns):
                raise ValueError(f'Schema column mismatch: {kind}')
            for item in definitions:
                if (item['type'],item['unit']) != columns[item['name']][:2]:
                    raise ValueError(f'Schema unit/type mismatch: {kind}.{item["name"]}')
            count, previous_index = 0, 0
            with open_file(f'{kind}.csv') as handle, io.TextIOWrapper(handle, encoding='utf-8', newline='') as text:
                reader = csv.DictReader(text)
                if reader.fieldnames != list(columns):
                    raise ValueError(f'CSV header mismatch: {kind}')
                for row in reader:
                    if None in row or any(value is None for value in row.values()):
                        raise ValueError(f'CSV row width mismatch: {kind}')
                    if row['session_id'] != metadata['session_id']:
                        raise ValueError('Session identity mismatch')
                    index = int(row['sample_index'])
                    if index <= previous_index:
                        raise ValueError('Sample order mismatch within stream')
                    previous_index = index
                    if row['timestamp_utc'] != utc_timestamp(int(row['timestamp_unix_ns'])):
                        raise ValueError('UTC/unix timestamp mismatch')
                    if float(row['elapsed_s']) < 0:
                        raise ValueError('Negative elapsed time')
                    if 'started_monotonic_ns' in metadata:
                        expected = (int(row['monotonic_ns'])-int(metadata['started_monotonic_ns']))/1e9
                        if not math.isclose(float(row['elapsed_s']),expected,abs_tol=1e-9):
                            raise ValueError('Monotonic/elapsed time mismatch')
                    for key, (kind_type,_unit,_description) in columns.items():
                        value = row[key]
                        if not value:
                            if key in COMMON:
                                raise ValueError(f'Missing required field: {key}')
                            continue
                        if kind_type == 'integer':
                            int(value)
                        elif kind_type == 'number' and not math.isfinite(float(value)):
                            raise ValueError(f'Nonfinite numeric field: {key}')
                        elif kind_type == 'boolean' and value not in ('true','false'):
                            raise ValueError(f'Invalid boolean: {key}')
                    count += 1
            if count != metadata.get('row_counts',{}).get(kind):
                raise ValueError(f'Row count mismatch: {kind}')
            counts[kind] = count
    return {'schema_valid':True,'session_id':metadata['session_id'], 'complete':metadata.get('complete') is True,
            'data_origin':metadata.get('data_origin','unspecified'), 'row_counts':counts,
            'quality_flags':metadata.get('quality_flags',[]), 'dropped_rows':metadata.get('dropped_rows',0)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset',type=Path)
    args = parser.parse_args()
    try:
        result = validate(args.dataset)
    except (OSError,ValueError,KeyError,TypeError,zipfile.BadZipFile) as exc:
        print(json.dumps({'schema_valid':False,'error':str(exc)},ensure_ascii=True))
        return 1
    print(json.dumps(result,ensure_ascii=True,indent=2))
    return 0 if result['complete'] and not result['dropped_rows'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
