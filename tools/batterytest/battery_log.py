#!/usr/bin/env python3
"""Capture and analyze Unidex BATLOG recordings (pyserial needed only for USB)."""
import argparse
import csv
import io
import math
from pathlib import Path
import re
import statistics
import sys
import time

HEADER = ['source', 'elapsed_ms', 'millivolts', 'raw_mean', 'raw_span', 'usb_host', 'usb_active']
SOURCES = ('pc', 'wall', 'battery')
ID_RE = re.compile(r'^[A-Za-z0-9_-]{1,24}$', re.ASCII)


def valid_id(value):
    if not ID_RE.fullmatch(value):
        raise argparse.ArgumentTypeError('ID must be 1..24 ASCII letters, digits, underscores or hyphens')
    return value


def seconds_value(value):
    value = int(value)
    if not 5 <= value <= 1800:
        raise argparse.ArgumentTypeError('seconds must be 5..1800')
    return value


def connect(port):
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError('USB commands need pyserial: python -m pip install pyserial') from exc
    return serial.Serial(port, 115200, timeout=0.5, write_timeout=5)


def send(ser, command):
    ser.write((command + '\n').encode('ascii'))
    ser.flush()


def lines(ser, timeout=15):
    deadline = time.monotonic() + timeout
    pending = bytearray()
    while time.monotonic() < deadline:
        line = ser.readline()
        if not line:
            continue
        pending.extend(line)
        if not pending.endswith(b'\n'):
            continue
        text = pending.decode('utf-8', errors='replace').strip()
        pending.clear()
        if text.startswith('ERR BATLOG '):
            raise RuntimeError(text)
        yield text
    raise RuntimeError('Timed out waiting for BATLOG response')


def response(ser, prefix, timeout=15):
    for line in lines(ser, timeout):
        if line.startswith(prefix):
            return line


def parse_csv(data):
    text = data.decode('utf-8-sig') if isinstance(data, bytes) else data
    reader = csv.DictReader(io.StringIO(text, newline=''))
    if reader.fieldnames != HEADER:
        raise ValueError('Unexpected CSV header; expected ' + ','.join(HEADER))
    rows = {source: [] for source in SOURCES}
    for number, row in enumerate(reader, 2):
        if None in row or any(row.get(k) is None for k in HEADER):
            raise ValueError(f'Invalid CSV row {number}')
        source = row['source']
        if source not in rows:
            raise ValueError(f'Invalid source on row {number}: {source!r}')
        try:
            parsed = {k: float(row[k]) for k in HEADER[1:]}
        except ValueError as exc:
            raise ValueError(f'Invalid number on row {number}') from exc
        if any(not math.isfinite(v) for v in parsed.values()):
            raise ValueError(f'Non-finite number on row {number}')
        parsed['original'] = row
        rows[source].append(parsed)
    return rows


def split_csv(path, rows):
    for source in SOURCES:
        target = path.with_name(path.stem + '_' + source + path.suffix)
        with target.open('w', encoding='utf-8', newline='') as out:
            writer = csv.DictWriter(out, fieldnames=HEADER, lineterminator='\n')
            writer.writeheader()
            writer.writerows(row['original'] for row in rows[source])
        print(f'{source}: {target} ({len(rows[source])} rows)')


def command_start(args):
    with connect(args.port) as ser:
        command = args.command.upper()
        send(ser, f'BATLOG {command} {args.id} {args.seconds}')
        prefix = f'OK BATLOG {command} '
        line = response(ser, prefix)
        if line != prefix + args.id:
            raise RuntimeError('Unexpected response: ' + line)
        print(line)
        if not args.live:
            return
        count = 0
        with Path(args.live).open('w', encoding='utf-8', newline='') as out:
            out.write(','.join(HEADER) + '\n')
            # Recording lasts at most seconds; leave room for stage transition.
            for line in lines(ser, args.seconds + 60):
                if line.startswith('BATLOG ROW '):
                    row = line[len('BATLOG ROW '):]
                    if not row.startswith('pc,') or len(next(csv.reader([row]))) != len(HEADER):
                        raise RuntimeError('Invalid live CSV row')
                    out.write(row + '\n')
                    count += 1
                elif line == 'BATLOG STATE wait_wall':
                    print(f'PC capture: {count} rows in {args.live}; state wait_wall')
                    return
                elif line in ('BATLOG STATE stopped', 'BATLOG STATE idle', 'BATLOG STATE done'):
                    raise RuntimeError(f'Recording ended unexpectedly: {line}; saved {count} rows')


def command_simple(args):
    with connect(args.port) as ser:
        send(ser, 'BATLOG ' + args.command.upper())
        print(response(ser, 'OK BATLOG ' + args.command.upper()))


def command_download(args):
    with connect(args.port) as ser:
        send(ser, 'BATLOG READ ' + args.id)
        line = response(ser, 'OK BATLOG READ ')
        count_text = line[len('OK BATLOG READ '):]
        if not count_text.isascii() or not count_text.isdecimal():
            raise RuntimeError('Invalid byte count: ' + line)
        count = int(count_text)
        if count > 50_000_000:
            raise RuntimeError('CSV byte count exceeds 50 MB safety limit')
        chunks = []
        remaining = count
        deadline = time.monotonic() + max(30, count / 2000 + 15)
        while remaining:
            if time.monotonic() > deadline:
                raise RuntimeError(f'Incomplete CSV: received {count - remaining} of {count} bytes')
            chunk = ser.read(min(remaining, 65536))
            if chunk:
                chunks.append(chunk)
                remaining -= len(chunk)
        data = b''.join(chunks)
        footer = response(ser, 'OK BATLOG END', timeout=15)
        if footer != 'OK BATLOG END':
            raise RuntimeError('Invalid footer: ' + footer)
    rows = parse_csv(data)
    path = Path(args.output)
    path.write_bytes(data)
    print(f'Saved {len(data)} bytes to {path}')
    split_csv(path, rows)


def percentile(values, p):
    values = sorted(values)
    i = (len(values) - 1) * p
    lo = int(i)
    return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * (i - lo)


def describe(values):
    return 'min/p05/median/p95/max=' + '/'.join(f'{percentile(values, p):.1f}' for p in (0, .05, .5, .95, 1))


def analyze(args):
    rows = parse_csv(Path(args.file).read_bytes())
    for source in SOURCES:
        items = rows[source]
        if not items:
            print(f'{source}: MISSING')
            continue
        times = [r['elapsed_ms'] for r in items]
        gaps = [b - a for a, b in zip(times, times[1:])]
        missing = sum(max(0, round(g / 100) - 1) for g in gaps if g > 150)
        print(f'{source}: {len(items)} rows; time {min(times):.0f}..{max(times):.0f} ms; '
              f'expected ~{args.seconds * 10} rows for {args.seconds}s at 10 Hz; '
              f'estimated missing within gaps={missing}; non-increasing timestamps={sum(g <= 0 for g in gaps)}')
        print('  mV ' + describe([r['millivolts'] for r in items]))
        print('  raw_span ' + describe([r['raw_span'] for r in items]))
        print('  raw_mean ' + describe([r['raw_mean'] for r in items]))
        for flag in ('usb_host', 'usb_active'):
            counts = {v: sum(r[flag] == v for r in items) for v in sorted({r[flag] for r in items})}
            print(f'  {flag} counts: {counts}')
        # Compare medians of the first and last tenth of elapsed-time range.
        low, high = min(times), max(times)
        if high > low:
            width = (high - low) * .1
            first = [r['millivolts'] for r in items if r['elapsed_ms'] <= low + width]
            last = [r['millivolts'] for r in items if r['elapsed_ms'] >= high - width]
            first_t = statistics.median(r['elapsed_ms'] for r in items if r['elapsed_ms'] <= low + width)
            last_t = statistics.median(r['elapsed_ms'] for r in items if r['elapsed_ms'] >= high - width)
            delta = statistics.median(last) - statistics.median(first)
            if last_t > first_t:
                print(f'  first-to-last 10% median change {delta:+.1f} mV, ~{delta * 3600000 / (last_t - first_t):+.1f} mV/hour')
    wall = sorted(r['millivolts'] for r in rows['wall'])
    battery = sorted(r['millivolts'] for r in rows['battery'])
    if wall and battery:
        print(f'wall/battery observed range overlap: {max(0, min(wall[-1], battery[-1]) - max(wall[0], battery[0])):.1f} mV')
        # Predict battery when voltage is below threshold; report sample-level errors.
        import bisect
        values = sorted(set(wall + battery))
        thresholds = [values[0] - 1] + [(a + b) / 2 for a, b in zip(values, values[1:])] + [values[-1] + 1]
        best = min(thresholds, key=lambda t: bisect.bisect_left(wall, t) / len(wall) + (len(battery) - bisect.bisect_left(battery, t)) / len(battery))
        fp = bisect.bisect_left(wall, best)
        miss = len(battery) - bisect.bisect_left(battery, best)
        print(f'best balanced in-sample rule battery < {best:.2f} mV: wall false positives {fp}/{len(wall)} ({100*fp/len(wall):.1f}%), battery misses {miss}/{len(battery)} ({100*miss/len(battery):.1f}%)')
        print('Threshold is fitted to this recording, not validated on independent runs; adjacent samples are correlated.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    start = sub.add_parser('start', help='start a full run with the PC stage')
    arm = sub.add_parser('arm', help='start with the wall and battery stages')
    battery = sub.add_parser('battery', help='record just the battery stage')
    for p in (start, arm, battery):
        p.add_argument('--port', required=True)
        p.add_argument('--id', required=True, type=valid_id)
        p.add_argument('--seconds', type=seconds_value, default=1800)
    start.add_argument('--live', help='save live PC-stage CSV until wait_wall')
    arm.set_defaults(live=None)
    battery.set_defaults(live=None)
    for name in ('status', 'stop'):
        p = sub.add_parser(name)
        p.add_argument('--port', required=True)
    download = sub.add_parser('download')
    download.add_argument('--port', required=True)
    download.add_argument('--id', required=True, type=valid_id)
    download.add_argument('--output', required=True)
    analysis = sub.add_parser('analyze')
    analysis.add_argument('file')
    analysis.add_argument('--seconds', type=seconds_value, default=1800, help='planned duration per source (default 1800)')
    args = parser.parse_args()
    try:
        if args.command in ('start', 'arm', 'battery'): command_start(args)
        elif args.command in ('status', 'stop'): command_simple(args)
        elif args.command == 'download': command_download(args)
        else: analyze(args)
    except (OSError, RuntimeError, ValueError, UnicodeError, csv.Error) as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('\nInterrupted; the device may still be recording.', file=sys.stderr)
        return 130
    return 0


if __name__ == '__main__':
    sys.exit(main())
