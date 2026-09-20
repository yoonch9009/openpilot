"""Regression checks after withdrawing the unsuccessful Casper stop latch."""
from types import SimpleNamespace

import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.controls.lib import longcontrol


@pytest.fixture
def control(monkeypatch):
  monkeypatch.setattr(longcontrol, 'Params', lambda: SimpleNamespace(get_float=lambda name: 0.0))
  cp = car.CarParams.new_message(brand='hyundai', carFingerprint='HYUNDAI_CASPER',
    openpilotLongitudinalControl=True, pcmCruise=False, startingState=False,
    vEgoStarting=0.1, stopAccel=-2.0, stoppingDecelRate=0.8)
  cp.longitudinalTuning.kpBP = [0.0]
  cp.longitudinalTuning.kpV = [1.0]
  cp.longitudinalTuning.kiBP = [0.0]
  cp.longitudinalTuning.kiV = [0.0]
  cp.longitudinalTuning.kf = 1.0
  c = longcontrol.LongControl(cp)
  c.long_control_state = longcontrol.LongCtrlState.stopping
  c.last_output_accel = -0.5
  return c


def inputs(source='cruise', should_stop=False):
  cs = car.CarState.new_message(vEgo=0.05, aEgo=0, standstill=True, gearShifter='drive')
  plan = log.LongitudinalPlan.new_message(shouldStop=should_stop, aTarget=1.26,
                                         vTargetNow=0.5, longitudinalPlanSource=source)
  radar = log.RadarState.new_message()
  radar.leadOne.status = True
  radar.leadOne.dRel = 4.26
  radar.leadOne.vLead = 3.06
  return cs, plan, radar


@pytest.mark.parametrize('source', ['lead0', 'cruise'])
def test_confirmed_departure_is_not_blocked_by_previous_stop(control, source):
  cs, plan, radar = inputs(source)
  accel, _, _ = control.update(True, cs, plan, (-3.5, 2.0), 0.0, radar)
  assert control.long_control_state == longcontrol.LongCtrlState.pid
  assert accel > 0


def test_original_stopping_request_already_persists_without_added_latch(control):
  cs, plan, radar = inputs('lead0', should_stop=True)
  plan.aTarget = -0.01
  radar.leadOne.vLead = 0
  for _ in range(30):
    accel, _, _ = control.update(True, cs, plan, (-3.5, 2.0), 0.0, radar)
    assert control.long_control_state == longcontrol.LongCtrlState.stopping
    assert accel <= -0.5


def test_brake_input_still_prevents_departure(control):
  cs, plan, radar = inputs()
  cs.brakePressed = True
  accel, _, _ = control.update(True, cs, plan, (-3.5, 2.0), 0.0, radar)
  assert control.long_control_state == longcontrol.LongCtrlState.stopping
  assert accel <= 0


def test_inactive_controller_does_not_force_braking(control):
  cs, plan, radar = inputs(should_stop=True)
  accel, _, _ = control.update(False, cs, plan, (-3.5, 2.0), 0.0, radar)
  assert control.long_control_state == longcontrol.LongCtrlState.off
  assert accel == 0
