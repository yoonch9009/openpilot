#!/usr/bin/env python3
"""Offline Casper stop/restart analysis. Reads rlogs; never publishes or sends CAN.

Run with the device Python from /data/openpilot. Input rlogs are not modified.
Signal names/units come from the generic Hyundai DBC, not an OEM ECU specification.
"""
import argparse
import collections
import json
from pathlib import Path
import sys


MESSAGES = {339: 'TCS11', 544: 'ESP12', 608: 'EMS16', 809: 'EMS12',
            916: 'TCS13', 1056: 'SCC11', 1057: 'SCC12', 1151: 'ESP11',
            1287: 'TCS15', 905: 'SCC14'}


def summarize(rows):
  """Do not treat truncated data, manual departures, or stale services as success."""
  results = []
  stationary = None
  observed_start = False
  attempt = None
  previous = None
  manual_in_episode = False

  def finish(outcome, row):
    nonlocal attempt
    if attempt is not None:
      attempt.update(outcome=outcome, end_t=row['t'],
                     elapsed_s=round(row['t'] - attempt['request_t'], 3))
      results.append(attempt)
      attempt = None

  for row in rows:
    fresh = 0 <= row['cs_age'] <= .2
    plan_fresh = 0 <= row['plan_age'] <= .3
    discontinuity = previous is None or row['route'] != previous['route'] or row['t'] - previous['t'] > 1
    if discontinuity or not fresh:
      finish('data_gap', previous or row)
      stationary = None
      observed_start = False
      manual_in_episode = False
    if not fresh:
      previous = row
      continue
    if abs(row['v']) > .1:
      stationary = None
    elif stationary is None:
      stationary = row['t']
      observed_start = not discontinuity and previous is not None and abs(previous['v']) > .1
    if abs(row['v']) > .3:
      manual_in_episode = False
    elif row['brake'] or row['gas']:
      manual_in_episode = True

    if attempt is not None:
      attempt['max_request'] = max(attempt['max_request'], row['request'])
      if row['brake'] or row['gas']:
        finish('driver_intervened', row)
      elif not row['active']:
        finish('control_inactive', row)
      elif abs(row['v']) > .3:
        finish('motion_without_pedal_input', row)
      elif row['should_stop'] and row['request'] <= 0:
        finish('stop_requested_again', row)
    elif (plan_fresh and not manual_in_episode and stationary is not None
          and row['t'] - stationary >= 1 and row['active']
          and not row['brake'] and not row['gas'] and not row['should_stop']
          and row['state'] == 'pid' and row['request'] > .05):
      attempt = {'route': row['route'], 'request_t': row['t'],
                 'stationary_since': stationary,
                 'stationary_s': round(row['t'] - stationary, 3),
                 'stationary_duration_is_lower_bound': not observed_start,
                 'max_request': row['request'], 'request_snapshot': row}
    previous = row
  if previous is not None:
    finish('recording_ended', previous)
  return results


def extract(paths):
  sys.path[:0] = ['/data/openpilot', '/data/openpilot/pydeps']
  import zstandard
  from openpilot.cereal import log
  from opendbc.can import CANParser

  rows, errors, changes = [], [], []
  commits = set()
  counts = collections.Counter()
  route = None
  parsers, seen, last_values, service = {}, {}, {}, {}
  last_sample = -1
  for path in paths:
    current_route = path.parent.name.rsplit('--', 1)[0]
    if route != current_route:
      route = current_route
      parsers = {b: CANParser('hyundai_kia_generic', [(n, 0) for n in MESSAGES.values()], b)
                 for b in (0, 2, 128)}
      seen, last_values, service = {}, {}, {}
      last_sample = -1
    try:
      with zstandard.ZstdDecompressor().stream_reader(path.open('rb')) as stream:
        data = stream.read()
      # Services can be written slightly out of timestamp order. Compare physical
      # states in event-time order; do not mistake a later carState for past input.
      for e in sorted(log.Event.read_multiple_bytes(data), key=lambda e: e.logMonoTime):
        t = e.logMonoTime / 1e9
        kind = e.which()
        if kind == 'initData':
          commits.add(e.initData.gitCommit)
        elif kind == 'can':
          packets = [(m.address, m.dat, m.src) for m in e.can
                     if m.address in MESSAGES and m.src in parsers]
          for bus, cp in parsers.items():
            batch = [p for p in packets if p[2] == bus]
            if not batch:
              continue
            cp.update([e.logMonoTime, batch])
            for address, _, _ in batch:
              name = MESSAGES[address]
              key = f'{bus}:{name}'
              seen[key] = t
              counts[key] += 1
              vals = dict(cp.vl[name])
              # Keep mode/flag transitions at CAN cadence, not just 10 Hz samples.
              if name in ('TCS13', 'TCS15', 'ESP11', 'SCC12', 'SCC14'):
                flags = {k: v for k, v in vals.items() if k in (
                  'ACCEnable', 'ACC_REQ', 'SCCReqLim', 'DCEnable', 'BrakeLight',
                  'StandStill', 'DriverOverride', 'DriverBraking', 'ACCMode',
                  'StopReq', 'AVH_LAMP', 'AVH_STAT', 'LDM_STAT', 'ECD_ACT',
                  'ACCFailInfo', 'TakeOverReq', 'FCA_ACK', 'EBA_ACK')}
                if flags != last_values.get(key):
                  changes.append({'route': route, 't': t, 'message': key, 'flags': flags})
                  last_values[key] = flags
        elif kind == 'carState':
          c = e.carState
          service['cs'] = (t, {'v': c.vEgo, 'brake': c.brakePressed, 'gas': c.gasPressed,
            'hold': c.brakeHoldActive, 'gear': str(c.gearShifter),
            'cruise': c.cruiseState.to_dict(), 'softHold': c.softHoldActive})
        elif kind == 'longitudinalPlan':
          p = e.longitudinalPlan
          service['plan'] = (t, {'should_stop': p.shouldStop, 'target': p.aTarget})
        elif kind == 'carControl' and t - last_sample >= .099:
          if 'cs' not in service or 'plan' not in service:
            continue
          last_sample = t
          c = e.carControl
          signals = {}
          for key, received in seen.items():
            age = t - received
            if 0 <= age <= .5:
              bus, name = key.split(':')
              signals[key] = {'age': age, 'values': dict(parsers[int(bus)].vl[name])}
          rows.append({'route': route, 'segment': path.parent.name, 't': t,
            'cs_age': t - service['cs'][0], 'plan_age': t - service['plan'][0],
            **service['cs'][1], **service['plan'][1], 'active': c.longActive,
            'state': str(c.actuators.longControlState), 'request': c.actuators.accel,
            'signals': signals})
    except Exception as exc:
      errors.append({'file': str(path), 'error': repr(exc)})
      # Never carry stale parser/service state through an unreadable segment.
      route = None
  return {'schema': 1, 'commits': sorted(commits), 'rows': rows, 'flag_changes': changes,
          'received_frame_counts': dict(counts), 'errors': errors,
          'departures': summarize(rows)}


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--input-root', type=Path, required=True)
  ap.add_argument('--output', type=Path, required=True)
  args = ap.parse_args()
  paths = sorted(args.input_root.glob('*/rlog.zst'),
                 key=lambda p: (p.parent.name.rsplit('--', 1)[0], int(p.parent.name.rsplit('--', 1)[1])))
  if not paths:
    raise SystemExit('No rlog.zst files found; no vehicle test has been analyzed.')
  result = extract(paths)
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(result), encoding='utf-8')
  print(json.dumps({'files': len(paths), 'rows': len(result['rows']), 'errors': result['errors'],
    'departures': [{k: v for k, v in d.items() if k != 'request_snapshot'} for d in result['departures']]}))


if __name__ == '__main__':
  main()
