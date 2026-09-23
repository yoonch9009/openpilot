"""Offline CAN regressions for the Casper departure jerk-only trial."""
import copy
from types import SimpleNamespace as NS

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.hyundai import hyundaican
from opendbc.car.hyundai.carstate import casper_brake_control_active
from opendbc.car.hyundai.tests.test_casper_stopping import make_state
from opendbc.car.hyundai.values import CAR


def state():
  cs = make_state()
  cs.casper_brake_control_active = True
  cs.out.vEgo = 0.0
  cs.out.standstill = True
  cs.out.accFaulted = False
  return cs


def request(cs, **changes):
  args = dict(enabled=True, long_active=True, accel=0.2, stopping=False,
              long_override=False, idx=7, soft_hold_mode=0,
              jerk=NS(carrot_cruise=0, cb_upper=.8, cb_lower=.7, jerk_u=.5, jerk_l=1.0),
              hud_control=NS(leadDistance=4.0, leadRelSpeed=.5, leadVisible=True, leadDistanceBars=2),
              set_speed=80, suppress_casper_ev_fca=False, CS=cs)
  args.update(changes)
  return hyundaican.create_acc_commands_scc(CANPacker('hyundai_kia_generic'), **args)


def decode14(messages):
  cp = CANParser('hyundai_kia_generic', [('SCC14', 50)], 0)
  cp.update([1_000_000_000, messages])
  return dict(cp.vl['SCC14'])


@pytest.mark.parametrize('upper', [.0, .5, .9, 1.0, 2.0, 5.0])
@pytest.mark.parametrize('accel', [.01, .2, 1.0, 2.5])
def test_only_scc14_upper_allowance_changes(upper, accel):
  cs = state()
  jerk = NS(carrot_cruise=0, cb_upper=.8, cb_lower=.7, jerk_u=upper, jerk_l=1.0)
  before = copy.deepcopy((cs, jerk))
  old = request(cs, long_active=False, jerk=jerk, accel=accel)
  new = request(cs, jerk=jerk, accel=accel)
  assert [m for m in new if m[0] != 905] == [m for m in old if m[0] != 905]
  a, b = decode14(old), decode14(new)
  assert b.pop('JerkUpperLimit') == pytest.approx(max(upper, 1.0))
  a.pop('JerkUpperLimit')
  assert a == b
  assert before == (cs, jerk)


@pytest.mark.parametrize(('field', 'value'), [
  ('canValid', False), ('standstill', False), ('vEgo', .1), ('vEgo', -.1),
  ('vEgo', float('nan')), ('brakePressed', True), ('gasPressed', True),
  ('parkingBrake', True), ('brakeHoldActive', True), ('accFaulted', True),
  ('gearShifter', structs.CarState.GearShifter.park),
  ('gearShifter', structs.CarState.GearShifter.reverse),
])
def test_vehicle_inhibits_trial(field, value):
  cs = state()
  setattr(cs.out, field, value)
  assert request(cs) == request(cs, long_active=False)


@pytest.mark.parametrize('changes', [
  dict(enabled=False), dict(stopping=True), dict(long_override=True),
  dict(accel=0), dict(accel=-.5),
])
def test_no_trial_during_hold_cancel_override_or_decel(changes):
  cs = state()
  assert request(cs, **changes) == request(cs, long_active=False, **changes)


@pytest.mark.parametrize(('field', 'value'), [('leadVisible', False), ('leadDistance', 0), ('leadRelSpeed', 0), ('leadRelSpeed', -.1)])
def test_requires_departing_lead(field, value):
  hud = NS(leadDistance=4., leadRelSpeed=.5, leadVisible=True, leadDistanceBars=2)
  setattr(hud, field, value)
  cs = state()
  assert request(cs, hud_control=hud) == request(cs, hud_control=hud, long_active=False)


def test_inactive_feedback_and_modes_preserve_baseline():
  for field, value in [('casper_brake_control_active', False), ('softHoldActive', 1), ('scc12', None)]:
    cs = state()
    setattr(cs, field, value)
    assert request(cs) == request(cs, long_active=False)
  cs = state()
  cs.out.cruiseState.available = False
  assert request(cs) == request(cs, long_active=False)
  cs = state()
  cs.CP.openpilotLongitudinalControl = False
  assert request(cs) == request(cs, long_active=False)
  for platform in CAR:
    if platform != CAR.HYUNDAI_CASPER:
      cs = state()
      cs.CP.carFingerprint = platform
      assert request(cs) == request(cs, long_active=False)


def test_repeated_stops_release_and_rearm_without_changing_scc12():
  cs = state()
  for _ in range(10):
    for active, v, stopping, accel, expected in [
      (True, 0., True, -.5, .5), (True, 0., False, .2, 1.),
      (False, 0., False, .2, .5), (False, .2, False, .2, .5),
    ]:
      cs.casper_brake_control_active = active
      cs.out.vEgo = v
      new = request(cs, stopping=stopping, accel=accel)
      old = request(cs, stopping=stopping, accel=accel, long_active=False)
      assert decode14(new)['JerkUpperLimit'] == pytest.approx(expected)
      assert [m for m in new if m[0] == 1057] == [m for m in old if m[0] == 1057]


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
  # The real card process receives an empty timestamp map before first CAN.
  cp = NS(ts_nanos={}, _last_update_nanos=0, bus_timeout=False, vl={'TCS13': {'DCEnable': 1}})
  assert not casper_brake_control_active(cp)
  cp.ts_nanos['TCS13'] = {}
  assert not casper_brake_control_active(cp)
