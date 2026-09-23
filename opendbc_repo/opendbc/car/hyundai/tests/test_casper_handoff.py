"""Bounded Casper SCC mode handoff and actual CAN command invariants."""
import copy
from types import SimpleNamespace as NS

import pytest

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.hyundai.casper_handoff import CasperSccHandoff
from opendbc.car.hyundai.tests.test_casper_departure_jerk import request, state
from opendbc.car.hyundai.values import CAR


def stopped_trial():
  trial = CasperSccHandoff()
  for i in range(60):
    assert not trial.update(1_000_000_000 + i * 20_000_000, True, True, -.75, 0., False, True)
  return trial


def decoded(messages, name):
  cp = CANParser('hyundai_kia_generic', [(name, 50)], 0)
  cp.update([1_000_000_000, messages])
  return dict(cp.vl[name])


def test_one_handoff_then_no_repeat_until_a_new_real_stop():
  trial = stopped_trial()
  emitted = []
  for i in range(150):
    t = 2_200_000_000 + i * 20_000_000
    emitted.append(trial.update(t, True, False, .25, 0., True, True))
  assert emitted[:13] == [False] * 13
  assert emitted[13:41] == [True] * 28
  assert not any(emitted[41:])
  assert trial.phase == 'spent'
  # Brief creep and a second standstill must not repeat a brake-release trial.
  for i in range(150):
    assert not trial.update(5_200_000_000 + i * 20_000_000, True, i < 30,
                            -.75 if i < 30 else .25, .31 if i < 60 else 0., True, True)
  assert not trial.update(8_200_000_000, True, True, -.75, 1.0, False, True)
  assert trial.phase == 'idle'
  assert not trial.update(8_220_000_000, True, True, -.75, 0., False, True)
  assert trial.phase == 'holding'


@pytest.mark.parametrize(('change', 'value'), [
  ('ready', False), ('accel', 0), ('accel', -.1),
  ('accel', .81), ('speed', .1), ('lead_departing', False),
  ('brake_control_active', False),
])
def test_transient_or_invalid_departure_aborts_without_retry(change, value):
  trial = stopped_trial()
  inputs = dict(now_ns=2_200_000_000, ready=True, stopping=False, accel=.25,
                speed=0., lead_departing=True, brake_control_active=True)
  inputs[change] = value
  assert not trial.update(**inputs)
  assert trial.phase == 'spent'
  assert not trial.update(2_220_000_000, True, False, .25, 0., True, True)


@pytest.mark.parametrize(('change', 'value'), [
  ('ready', False), ('accel', .81), ('speed', .06),
  ('lead_departing', False), ('brake_control_active', False),
])
def test_started_handoff_stops_on_input_change(change, value):
  trial = stopped_trial()
  for i in range(14):
    current = trial.update(2_200_000_000 + i * 20_000_000, True, False, .25, 0., True, True)
  assert current and trial.phase == 'handoff'
  inputs = dict(now_ns=2_480_000_000, ready=True, stopping=False, accel=.25,
                speed=0., lead_departing=True, brake_control_active=True)
  inputs[change] = value
  assert not trial.update(**inputs)
  assert trial.phase == 'spent'


def test_hold_does_not_trigger_before_actual_departure():
  trial = stopped_trial()
  assert not trial.update(2_200_000_000, True, True, -.75, 0., False, True)
  assert trial.phase == 'holding'


def test_engaging_at_standstill_without_preceding_stop_never_starts_handoff():
  trial = CasperSccHandoff()
  for i in range(100):
    assert not trial.update(1_000_000_000 + i * 20_000_000, True, False, .25, 0., True, True)
  assert trial.phase == 'idle'


def test_frame_gap_and_clock_reversal_cannot_extend_handoff():
  for bad_time in (2_460_000_000, 2_700_000_000):
    trial = stopped_trial()
    for i in range(14):
      trial.update(2_200_000_000 + i * 20_000_000, True, False, .25, 0., True, True)
    assert not trial.update(bad_time, True, False, .25, 0., True, True)
    assert trial.phase == 'spent'


def test_only_both_acc_modes_and_scc12_checksum_change():
  cs = state()
  original = copy.deepcopy(cs)
  baseline = request(cs)
  handoff = request(cs, casper_handoff=True)
  assert cs == original
  assert [m for m in baseline if m[0] not in (1057, 905)] == [m for m in handoff if m[0] not in (1057, 905)]
  s12_old, s12_new = decoded(baseline, 'SCC12'), decoded(handoff, 'SCC12')
  s14_old, s14_new = decoded(baseline, 'SCC14'), decoded(handoff, 'SCC14')
  assert (s12_old.pop('ACCMode'), s12_new.pop('ACCMode')) == (1, 2)
  s12_old.pop('CR_VSM_ChkSum')
  s12_new.pop('CR_VSM_ChkSum')
  assert s12_old == s12_new
  assert s12_new['StopReq'] == 0
  assert s12_new['aReqRaw'] == pytest.approx(.2)
  assert (s14_old.pop('ACCMode'), s14_new.pop('ACCMode')) == (1, 2)
  assert s14_old == s14_new
  assert s14_new['JerkUpperLimit'] == pytest.approx(.5)
  raw = next(m[1] for m in handoff if m[0] == 1057)
  assert sum((byte >> 4) + (byte & 15) for byte in raw) % 16 == 0


@pytest.mark.parametrize(('field', 'value'), [
  ('canValid', False), ('canTimeout', True), ('standstill', False), ('vEgo', .1),
  ('brakePressed', True), ('gasPressed', True), ('parkingBrake', True),
  ('brakeHoldActive', True), ('accFaulted', True),
  ('gearShifter', structs.CarState.GearShifter.park),
])
def test_final_encoder_gate_blocks_invalid_input(field, value):
  cs = state()
  setattr(cs.out, field, value)
  assert request(cs, casper_handoff=True) == request(cs)


def test_missing_feedback_lead_and_other_platforms_never_change_output():
  cs = state()
  cs.casper_brake_control_active = False
  assert request(cs, casper_handoff=True) == request(cs)
  cs = state()
  cs.CP.carFingerprint = CAR.HYUNDAI_CASPER_EV
  assert request(cs, casper_handoff=True) == request(cs)
  cs = state()
  cs.CP.openpilotLongitudinalControl = False
  assert request(cs, casper_handoff=True) == request(cs)
  cs = state()
  assert request(cs, casper_handoff=True, stopping=True, accel=-.75) == request(cs, stopping=True, accel=-.75)
  cs = state()
  assert request(cs, casper_handoff=True, long_override=True) == request(cs, long_override=True)
  cs = state()
  for changed in [dict(leadVisible=False), dict(leadDistance=2.9), dict(leadRelSpeed=.49)]:
    hud = NS(leadDistance=4., leadRelSpeed=.6, leadVisible=True, leadDistanceBars=2)
    for key, value in changed.items():
      setattr(hud, key, value)
    assert request(cs, casper_handoff=True, hud_control=hud) == request(cs, hud_control=hud)
