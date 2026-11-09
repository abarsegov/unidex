#!/usr/bin/env python3
"""Arm a battery load test by USB and record the live and complete CSV over LAN."""
import argparse
import csv
import io
import ipaddress
import json
from pathlib import Path
import secrets
import statistics
import sys
import time
import urllib.error
import urllib.request

from battery_log import connect, response, seconds_value, send, valid_id


HEADER = ('elapsed_ms', 'millivolts', 'estimated_percent', 'cpu_mhz', 'wifi_ps_none', 'refresh_ms')
MAX_RESPONSE = 128 * 1024


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def lan_ip(value):
    try:
        address = ipaddress.IPv4Address(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('use the board LAN IPv4 address') from exc
    if not address.is_private or address.is_loopback or address.is_unspecified:
        raise argparse.ArgumentTypeError('use the board private LAN IPv4 address')
    return str(address)


def get(opener, host, token, extension):
    request = urllib.request.Request(
        f'http://{host}/battery-load.{extension}',
        headers={'Authorization': 'Bearer ' + token, 'Connection': 'close'},
    )
    with opener.open(request, timeout=3) as reply:
        data = reply.read(MAX_RESPONSE + 1)
    if len(data) > MAX_RESPONSE:
        raise RuntimeError('unexpected response size')
    return data


def save_complete(data, output):
    reader = csv.reader(io.StringIO(data.decode('ascii')))
    if tuple(next(reader, ())) != HEADER:
        raise RuntimeError('unexpected load test CSV header')
    rows = [tuple(map(int, row)) for row in reader]
    if not rows or any(len(row) != len(HEADER) for row in rows):
        raise RuntimeError('missing or malformed load samples')
    if any(a[0] >= b[0] for a, b in zip(rows, rows[1:])):
        raise RuntimeError('load timestamps are out of order')
    with output.open('xb') as stream:
        stream.write(data)
    first = [row for row in rows if row[0] - rows[0][0] < 30_000]
    last = [row for row in rows if rows[-1][0] - row[0] < 30_000]
    print(f'Saved {len(rows)} samples through {rows[-1][0] / 1000:.1f} s to {output}', flush=True)
    print('Median first/last 30 s under load: '
          f'{statistics.median(row[1] for row in first):g} -> '
          f'{statistics.median(row[1] for row in last):g} mV; '
          f'{statistics.median(row[2] for row in first):g} -> '
          f'{statistics.median(row[2] for row in last):g}% estimated', flush=True)


def run(args):
    token = secrets.token_hex(16)
    output = Path(args.output)
    live = output.with_name(output.stem + '.live.csv')
    if output.exists() or live.exists():
        raise FileExistsError('choose a new output filename')
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())
    with live.open('x', encoding='ascii', newline='', buffering=1) as stream:
        writer = csv.writer(stream, lineterminator='\n')
        writer.writerow(HEADER)
        with connect(args.port) as serial:
            send(serial, f'BATLOG LOAD {args.id} {args.seconds} {token}')
            if response(serial, 'OK BATLOG LOAD ') != f'OK BATLOG LOAD {args.id}':
                raise RuntimeError('unexpected USB arming reply')
        deadline = time.monotonic() + 20 * 60 + args.seconds + 60
        latest = -1
        announced = False
        while time.monotonic() < deadline:
            poll_at = time.monotonic()
            try:
                status = json.loads(get(opener, args.host, token, 'json'))
            except urllib.error.HTTPError:
                raise
            except (urllib.error.URLError, TimeoutError, ConnectionError):
                time.sleep(1)
                continue
            if status.get('id') != args.id:
                raise RuntimeError('board reports a different load test')
            if not announced:
                print(f'LAN ready: {args.id}, {status["state"]}; live file {live}', flush=True)
                announced = True
            row = status['sample']
            if row and (len(row) != len(HEADER) or any(type(n) is not int for n in row)):
                raise RuntimeError('malformed live sample')
            if row and row[0] > latest:
                writer.writerow(row)
                if row[0] // 10_000 > latest // 10_000:
                    print(f'{row[0] / 1000:.1f} s  {row[1]} mV  {row[2]}%  '
                          f'{row[3]} MHz  WiFi awake={row[4]}  refresh={row[5]} ms', flush=True)
                latest = row[0]
            if status['state'] in ('done', 'stopped'):
                if not status['saved']:
                    raise RuntimeError(f'board reports {status["state"]} without a saved test')
                save_complete(get(opener, args.host, token, 'csv'), output)
                print(f'Board result: {status["state"]}', flush=True)
                return
            time.sleep(max(0, 1 - (time.monotonic() - poll_at)))
    raise RuntimeError('no completion received before the timeout; the live file was retained')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--host', required=True, type=lan_ip)
    parser.add_argument('--id', required=True, type=valid_id)
    parser.add_argument('--seconds', default=1800, type=seconds_value)
    parser.add_argument('--output', required=True)
    try:
        run(parser.parse_args())
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
