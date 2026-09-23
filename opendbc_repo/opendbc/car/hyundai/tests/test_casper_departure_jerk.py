"""Preserve the restored jerk calculation and fresh TCS13 feedback checks."""
from types import SimpleNamespace as NS

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car.hyundai import hyundaican
from opendbc.car.hyundai.carstate import casper_brake_control_active
from opendbc.car.hyundai.tests.test_casper_stopping import make_state


def state():
  cs = make_state()
  cs.casper_brake_control_active = True
  cs.out.vEgo = 0.0
  cs.out.standstill = True
  cs.out.accFaulted = False
  return cs


def request(cs, **changes):
  args = dict(enabled=True, long_active=True, accel=.2, stopping=False,
              long_override=False, idx=7, soft_hold_mode=0,
              jerk=NS(carrot_cruise=0, cb_upper=.8, cb_lower=.7, jerk_u=.5, jerk_l=1.0),
              hud_control=NS(leadDistance=4.0, leadRelSpeed=.6, leadVisible=True, leadDistanceBars=2),
              set_speed=80, suppress_casper_ev_fca=False, CS=cs)
  args.update(changes)
  return hyundaican.create_acc_commands_scc(CANPacker('hyundai_kia_generic'), **args)


@pytest.mark.parametrize('jerk_upper', [.5, .9, 1.0, 2.0, 5.0])
def test_failed_jerk_floor_is_withdrawn(jerk_upper):
  cs = state()
  jerk = NS(carrot_cruise=0, cb_upper=.8, cb_lower=.7, jerk_u=jerk_upper, jerk_l=1.0)
  encoded = request(cs, jerk=jerk)
  parser = CANParser('hyundai_kia_generic', [('SCC14', 50)], 0)
  parser.update([1_000_000_000, encoded])
  assert parser.vl['SCC14']['JerkUpperLimit'] == pytest.approx(jerk_upper)


@pytest.mark.parametrize(('timestamp', 'now', 'timeout', 'dc', 'expected'), [
  (1_000_000_000, 1_000_000_000, False, 1, True),
  (1_000_000_000, 1_100_000_000, False, 1, True),
  (1_000_000_000, 1_100_000_001, False, 1, False),
  (0, 0, False, 1, False), (2, 1, False, 1, False),
  (1, 1, True, 1, False), (1, 1, False, 0, False),
])
def test_missing_stale_or_invalid_tcs_feedback_cannot_enable(timestamp, now, timeout, dc, expected):
  cp = NS(ts_nanos={'TCS13': {'DCEnable': timestamp}}, _last_update_nanos=now,
          bus_timeout=timeout, vl={'TCS13': {'DCEnable': dc}})
  assert casper_brake_control_active(cp) is expected


def test_feedback_uses_received_tcs13_and_expires_when_bus_stops():
  cp = CANParser('hyundai_kia_generic', [('TCS13', 50)], 0)
  packer = CANPacker('hyundai_kia_generic')
  cp.update([1_000_000_000, [packer.make_can_msg('TCS13', 0, {'DCEnable': 1})]])
  assert casper_brake_control_active(cp)
  cp.update([1_020_000_000, [packer.make_can_msg('TCS13', 0, {'DCEnable': 0})]])
  assert not casper_brake_control_active(cp)
  cp.update([1_040_000_000, [packer.make_can_msg('TCS13', 0, {'DCEnable': 1})]])
  assert casper_brake_control_active(cp)
  cp.update([1_240_000_000, []])
  assert not casper_brake_control_active(cp)


def test_no_tcs13_timestamp_during_parser_startup_disables_trial():
  cp = NS(ts_nanos={}, _last_update_nanos=0, bus_timeout=False, vl={'TCS13': {'DCEnable': 1}})
  assert not casper_brake_control_active(cp)
  cp.ts_nanos['TCS13'] = {}
  assert not casper_brake_control_active(cp)
