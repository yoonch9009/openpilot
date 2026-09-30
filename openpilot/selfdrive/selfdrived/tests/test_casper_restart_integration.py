from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import car, log, messaging
from openpilot.common.casper_diagnostics import CasperDiagnostics
from openpilot.selfdrive.selfdrived.casper_restart import CasperCruiseRestart
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.selfdrive.selfdrived.state import StateMachine


class SM(dict):
  def refresh(self, now):
    self.logMonoTime = {k: now - (2_000_000 if k == 'longitudinalPlan' else 1_000_000) for k in self}
    self.alive = dict.fromkeys(self, True)
    self.valid = dict.fromkeys(self, True)


def setup():
  sd = SelfdriveD.__new__(SelfdriveD)
  sd.casper_restart = CasperCruiseRestart()
  sd.casper_restart_lead = None
  sd.casper_restart_log = CasperDiagnostics('restart', lambda _: None)
  sd.state_machine = StateMachine()
  sd.state_machine.state = log.SelfdriveState.OpenpilotState.enabled
  sd.enabled = sd.active = True
  sd.events = Events()
  sd.sm = SM({name: getattr(messaging.new_message(name), name)
              for name in ['carControl', 'controlsState', 'longitudinalPlan', 'radarState']})
  cs = car.CarState.new_message()
  cs.canValid = True
  cs.gearShifter = 'drive'
  cs.cruiseState.available = True
  cs.standstill = True
  lead = sd.sm['radarState'].leadOne
  lead.status, lead.dRel, lead.vRel, lead.radarTrackId = True, 4.0, 1.0, 4
  return sd, cs


def tick(sd, cs, now, *, departure=False, stale_cs=False, stale_source=None, driver_cancel=False, old_consumed_plan=False,
         plan_stop=False, epoch_override=None, plan_target=None, departure_inputs_ns=None):
  sd.events.clear()
  if driver_cancel:
    sd.events.add(log.OnroadEvent.EventName.buttonCancel)
  sd.sm.refresh(now)
  if stale_source:
    sd.sm.logMonoTime[stale_source] = now - 1_000_000_000
  sd.casper_cs_valid = True
  sd.casper_cs_mono_ns = now - (200_000_000 if stale_cs else 1_000_000)
  cc, controls, plan = sd.sm['carControl'], sd.sm['controlsState'], sd.sm['longitudinalPlan']
  cc.enabled = cc.longActive = sd.enabled
  state = 'off' if not sd.enabled else ('pid' if departure else 'stopping')
  cc.actuators.longControlState = state
  cc.actuators.accel = 0 if not sd.enabled else (.2 if departure else -.5)
  controls.longControlState = state
  controls.longitudinalPlanMonoTime = sd.casper_restart.request_ns if old_consumed_plan else sd.sm.logMonoTime['longitudinalPlan']
  plan.casperDepartureInputsMonoTime = now - 3_000_000 if departure_inputs_ns is None else departure_inputs_ns
  plan.hasLead = True
  plan.shouldStop = plan_stop or not departure
  plan.aTarget = (.2 if departure else -.5) if plan_target is None else plan_target
  plan.casperRestartRequestMonoTime = sd.casper_restart.plan_marker(now) if epoch_override is None else epoch_override
  sd.update_casper_restart(cs, now)
  sd.enabled, sd.active = sd.state_machine.update(sd.events)


def enter_owned_off(sd, cs):
  for i in range(150):
    tick(sd, cs, 1_000_000_000 + i * 10_000_000, departure=i >= 120)
    if sd.casper_restart.phase == 'off':
      return 1_000_000_000 + i * 10_000_000
  raise AssertionError('Automatic cancel did not occur')


def test_binder_drives_full_state_machine_off_and_on_after_ack():
  sd, cs = setup()
  now = enter_owned_off(sd, cs)
  assert not sd.enabled
  tick(sd, cs, now + 10_000_000, departure=True)
  assert not sd.enabled  # OFF acknowledged, but no plan newer than that ACK.
  tick(sd, cs, now + 20_000_000, departure=True)
  assert sd.enabled and sd.casper_restart.reason == 'resume_requested'


def test_zero_dwell_still_waits_until_controller_consumes_post_off_plan():
  sd, cs = setup()
  now = enter_owned_off(sd, cs)
  tick(sd, cs, now + 10_000_000, departure=True)
  assert not sd.enabled
  for i in range(2, 15):
    tick(sd, cs, now + i * 10_000_000, departure=True, old_consumed_plan=True)
    assert not sd.enabled
  tick(sd, cs, now + 150_000_000, departure=True)
  assert sd.enabled


def test_reentry_needs_current_positive_plan_and_matching_restart_epoch():
  sd, cs = setup()
  now = enter_owned_off(sd, cs)
  tick(sd, cs, now + 10_000_000, departure=True)
  tick(sd, cs, now + 20_000_000, departure=True, plan_stop=True)
  assert not sd.enabled
  tick(sd, cs, now + 30_000_000, departure=True, plan_target=0.)
  assert not sd.enabled
  tick(sd, cs, now + 40_000_000, departure=True, epoch_override=0)
  assert not sd.enabled
  tick(sd, cs, now + 50_000_000, departure=True)
  assert sd.enabled and sd.casper_restart.plan_marker(now + 50_000_000)
  tick(sd, cs, now + 60_000_000, departure=True)
  assert sd.enabled and sd.casper_restart.reason == 'resume_complete'
  assert sd.casper_restart.plan_marker(now + 60_000_000) == 0


def test_moving_lead_waits_for_planner_and_controller_permission():
  sd, cs = setup()
  # The lead is already moving, but the planner continues to request stopping.
  for i in range(140):
    tick(sd, cs, 1_000_000_000 + i * 10_000_000)
    assert sd.enabled
  # No second confirmation delay after the normal controller permits departure.
  tick(sd, cs, 2_400_000_000, departure=True)
  assert not sd.enabled and sd.casper_restart.phase == 'off'


def test_slow_lead_does_not_delay_off_but_resume_waits_for_current_motion():
  sd, cs = setup()
  lead = sd.sm['radarState'].leadOne
  lead.vRel = .1
  now = enter_owned_off(sd, cs)
  assert now == 2_200_000_000  # First cycle in which normal departure is allowed.
  for i in range(1, 21):
    tick(sd, cs, now + i * 10_000_000, departure=True)
    assert not sd.enabled and sd.casper_restart.phase == 'off'
  lead.vRel = .3
  tick(sd, cs, now + 210_000_000, departure=True)
  assert sd.enabled and sd.casper_restart.reason == 'resume_requested'


def test_driver_cancel_during_slow_lead_wait_prevents_later_resume():
  sd, cs = setup()
  sd.sm['radarState'].leadOne.vRel = .1
  now = enter_owned_off(sd, cs)
  tick(sd, cs, now + 10_000_000, departure=True, driver_cancel=True)
  sd.sm['radarState'].leadOne.vRel = 1.
  for i in range(2, 50):
    tick(sd, cs, now + i * 10_000_000, departure=True)
  assert not sd.enabled and sd.casper_restart.phase == 'spent'


def test_new_closing_lead_blocks_older_positive_plan_before_off():
  sd, cs = setup()
  for i in range(140):
    tick(sd, cs, 1_000_000_000 + i * 10_000_000)
  lead = sd.sm['radarState'].leadOne
  lead.radarTrackId, lead.dRel, lead.vRel = 9, 2., -.1
  tick(sd, cs, 2_400_000_000, departure=True)
  assert sd.enabled and sd.casper_restart.phase == 'holding'


def test_closing_lead_during_owned_off_aborts_resume():
  sd, cs = setup()
  now = enter_owned_off(sd, cs)
  sd.sm['radarState'].leadOne.vRel = -.1
  tick(sd, cs, now + 10_000_000, departure=True)
  assert not sd.enabled and sd.casper_restart.phase == 'spent'


@pytest.mark.parametrize('distance', [0.5, 1.5, 1.99, 2.0])
def test_positive_gap_below_two_meters_uses_normal_departure_permission(distance):
  sd, cs = setup()
  sd.sm['radarState'].leadOne.dRel = distance
  for i in range(140):
    tick(sd, cs, 1_000_000_000 + i * 10_000_000)
    assert sd.enabled  # A moving close lead never overrides a stop plan.
  tick(sd, cs, 2_400_000_000, departure=True)
  assert not sd.enabled and sd.casper_restart.phase == 'off'
  for i in range(1, 22):
    tick(sd, cs, 2_400_000_000 + i * 10_000_000, departure=True)
  assert sd.enabled and sd.casper_restart.reason == 'resume_complete'


@pytest.mark.parametrize('distance', [0.0, -1.0, float('nan'), float('inf')])
def test_invalid_gap_cannot_request_cancel_or_resume(distance):
  sd, cs = setup()
  sd.sm['radarState'].leadOne.dRel = distance
  for i in range(180):
    tick(sd, cs, 1_000_000_000 + i * 10_000_000, departure=i >= 120)
  assert sd.enabled and sd.casper_restart.phase == 'holding'
  sd, cs = setup()
  now = enter_owned_off(sd, cs)
  sd.sm['radarState'].leadOne.dRel = distance
  for i in range(1, 40):
    tick(sd, cs, now + i * 10_000_000, departure=True)
  assert not sd.enabled and sd.casper_restart.phase == 'spent'


@pytest.mark.parametrize('source', ['carControl', 'controlsState', 'longitudinalPlan', 'radarState', 'actual_car_state'])
def test_stale_input_during_owned_off_never_auto_enables(source):
  sd, cs = setup()
  now = enter_owned_off(sd, cs)
  tick(sd, cs, now + 10_000_000, departure=True,
       stale_cs=source == 'actual_car_state', stale_source=source if source != 'actual_car_state' else None)
  for i in range(2, 80):
    tick(sd, cs, now + i * 10_000_000, departure=True)
  assert not sd.enabled and sd.casper_restart.phase == 'spent'


def test_real_cancel_during_owned_off_is_never_undone():
  sd, cs = setup()
  now = enter_owned_off(sd, cs)
  tick(sd, cs, now + 10_000_000, departure=True, driver_cancel=True)
  for i in range(2, 80):
    tick(sd, cs, now + i * 10_000_000, departure=True)
  assert not sd.enabled


def test_lead_replacement_aborts_reenable():
  sd, cs = setup()
  now = enter_owned_off(sd, cs)
  sd.sm['radarState'].leadOne.radarTrackId = 9
  tick(sd, cs, now + 10_000_000, departure=True)
  assert sd.casper_restart.phase == 'spent' and not sd.enabled


def test_normal_longcontrol_off_resets_and_serializes_real_disabled_modes(monkeypatch):
  from opendbc.can import CANParser
  from opendbc.car.hyundai.values import CAR, HyundaiFlags
  from opendbc.car.hyundai.tests.test_casper_departure_jerk import request, state
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
  from openpilot.selfdrive.controls.tests.test_longcontrol_hyundai_tuning import make_cp, DictParams
  import openpilot.selfdrive.controls.lib.longcontrol as module
  monkeypatch.setattr(module, 'Params', lambda: DictParams({}))
  cp = make_cp()
  cp.carFingerprint = CAR.HYUNDAI_CASPER
  cp.flags = HyundaiFlags.CAMERA_SCC
  control = LongControl(cp)
  vehicle = state()
  cs = vehicle.out
  cs.aEgo, cs.softHoldActive, cs.canTimeout = 0., 0, False
  cs.cruiseState.standstill = False
  radar = NS(leadOne=NS(status=True, dRel=4., vRel=1.))
  plan = NS(aTarget=0., vTargetNow=0., jTargetNow=0., shouldStop=True)
  for _ in range(150):
    control.update(True, cs, plan, (-3.5, 2.0), 0, radar)
  assert control.last_output_accel < 0
  accel, _, _ = control.update(False, cs, plan, (-3.5, 2.0), 0, radar)
  assert accel == 0 and control.long_control_state == LongCtrlState.off and control.last_output_accel == 0
  off = request(vehicle, enabled=False, long_active=False, stopping=False, accel=accel)
  parser = CANParser('hyundai_kia_generic', [('SCC12', 50), ('SCC14', 50)], 0)
  parser.update([1_000_000_000, off])
  assert parser.vl['SCC12']['ACCMode'] == 0
  assert parser.vl['SCC14']['ACCMode'] == 4
  assert parser.vl['SCC12']['aReqRaw'] == pytest.approx(0)
  plan.shouldStop, plan.aTarget, plan.vTargetNow = False, .3, .1
  accel, _, _ = control.update(True, cs, plan, (-3.5, 2.0), 0, radar)
  assert accel > 0 and control.long_control_state == LongCtrlState.pid
  on = request(vehicle, enabled=True, long_active=True, stopping=False, accel=accel)
  parser.update([1_020_000_000, on])
  assert parser.vl['SCC12']['ACCMode'] == parser.vl['SCC14']['ACCMode'] == 1
  cs.vEgo, cs.aEgo, plan.vTargetNow = .5, .1, .4
  compensated, _, _ = control.update(True, cs, plan, (-3.5, 2.0), 0, radar)
  assert 0 < control.casper_launch.correction <= .2
  assert compensated <= plan.aTarget
  uncorrected, _, _ = control.update(True, cs, plan, (-3.5, 2.0), 0, radar, launch_inputs_valid=False)
  assert control.casper_launch.correction == 0
  assert uncorrected < compensated


def test_preserved_departure_plan_removes_reentry_brake_pulse_without_actuating_during_off(monkeypatch):
  from opendbc.car.hyundai.values import CAR, HyundaiFlags
  from opendbc.car.hyundai.tests.test_casper_departure_jerk import state
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
  from openpilot.selfdrive.controls.tests.test_longcontrol_hyundai_tuning import make_cp, DictParams
  import openpilot.selfdrive.controls.lib.longcontrol as module
  monkeypatch.setattr(module, 'Params', lambda: DictParams({}))
  cp = make_cp()
  cp.carFingerprint, cp.flags = CAR.HYUNDAI_CASPER, HyundaiFlags.CAMERA_SCC
  cs = state().out
  cs.vEgo, cs.aEgo, cs.softHoldActive, cs.canTimeout = 0., 0., 0, False
  cs.cruiseState.standstill = False
  radar = NS(leadOne=NS(status=True, dRel=4., vRel=1.))
  reset_plan = NS(aTarget=0., vTargetNow=0., jTargetNow=0., shouldStop=True)
  live_departure_plan = NS(aTarget=.3, vTargetNow=.01, jTargetNow=.5, shouldStop=False)
  outputs = []
  for post_off_plan in (reset_plan, live_departure_plan):
    control = LongControl(cp)
    for _ in range(120):
      control.update(True, cs, reset_plan, (-3.5, 2.5), 0, radar)
    control.update(True, cs, live_departure_plan, (-3.5, 2.5), 0, radar)
    off, _, _ = control.update(False, cs, post_off_plan, (-3.5, 2.5), 0, radar)
    assert off == 0 and control.long_control_state == LongCtrlState.off
    value, _, _ = control.update(True, cs, post_off_plan, (-3.5, 2.5), 0, radar)
    outputs.append(value)
  assert outputs[0] < 0  # Reproduces the old reset-plan stopping ramp.
  assert outputs[1] > 0  # Uses the live plan; no special brake-clamping rule.


@pytest.mark.parametrize('button_type', ['accelCruise', 'resumeCruise', 'decelCruise'])
def test_waiting_res_requires_release_and_new_consumed_departure_plan(button_type):
  sd, cs = setup()
  for i in range(110):
    tick(sd, cs, 1_000_000_000 + i * 10_000_000)
  now = 2_100_000_000
  cs.buttonEvents = [car.CarState.ButtonEvent.new_message(type=button_type, pressed=True)]
  tick(sd, cs, now, departure=True)
  assert sd.casper_restart.phase == 'holding'
  cs.buttonEvents = []
  for i in range(1, 53):
    tick(sd, cs, now + i * 10_000_000, departure=True)
    assert sd.enabled and sd.casper_restart.phase == 'holding'
  now += 530_000_000
  cs.buttonEvents = [car.CarState.ButtonEvent.new_message(type=button_type, pressed=False)]
  tick(sd, cs, now, departure=True)
  assert sd.enabled and sd.casper_restart.phase == 'holding'
  cs.buttonEvents = []
  tick(sd, cs, now + 1_000_000, departure=True)  # Published plan still predates release.
  assert sd.enabled
  tick(sd, cs, now + 10_000_000, departure=True, old_consumed_plan=True)
  assert sd.enabled
  tick(sd, cs, now + 20_000_000, departure=True, departure_inputs_ns=0)
  assert sd.enabled
  tick(sd, cs, now + 30_000_000, departure=True, departure_inputs_ns=now - 1)
  assert sd.enabled
  tick(sd, cs, now + 40_000_000, departure=True)
  assert not sd.enabled and sd.casper_restart.phase == 'off'


@pytest.mark.parametrize('phase', ['holding', 'off'])
@pytest.mark.parametrize('button_type', ['accelCruise', 'resumeCruise', 'decelCruise'])
def test_cancel_mixed_with_res_has_priority(phase, button_type):
  sd, cs = setup()
  if phase == 'off':
    now = enter_owned_off(sd, cs)
  else:
    for i in range(110):
      tick(sd, cs, 1_000_000_000 + i * 10_000_000)
    now = 2_100_000_000
  cs.buttonEvents = [car.CarState.ButtonEvent.new_message(type=button_type, pressed=True),
                     car.CarState.ButtonEvent.new_message(type='cancel', pressed=True)]
  tick(sd, cs, now + 10_000_000, departure=True, driver_cancel=True)
  assert not sd.enabled and sd.casper_restart.phase == 'spent'
  cs.buttonEvents = [car.CarState.ButtonEvent.new_message(type=button_type, pressed=False)]
  tick(sd, cs, now + 20_000_000, departure=True)
  assert not sd.enabled and sd.casper_restart.phase == 'spent'


@pytest.mark.parametrize('button_type', ['accelCruise', 'resumeCruise', 'decelCruise'])
def test_late_res_release_during_owned_off_aborts(button_type):
  sd, cs = setup()
  now = enter_owned_off(sd, cs)
  cs.buttonEvents = [car.CarState.ButtonEvent.new_message(type=button_type, pressed=False)]
  tick(sd, cs, now + 10_000_000, departure=True)
  assert not sd.enabled and sd.casper_restart.phase == 'spent'


def test_recorded_1833_waiting_res_then_departure_remains_armed():
  sd, cs = setup()
  # Relative to observed holding: RES press +1.671s, release +2.194s,
  # planner departure +10.271s. Replay at the binder's 100 Hz cadence.
  for i in range(1029):
    cs.buttonEvents = []
    if i == 167:
      cs.buttonEvents = [car.CarState.ButtonEvent.new_message(type='accelCruise', pressed=True)]
    elif i == 220:
      cs.buttonEvents = [car.CarState.ButtonEvent.new_message(type='accelCruise', pressed=False)]
    tick(sd, cs, 1_000_000_000 + i * 10_000_000, departure=i >= 1028)
    if i < 1028:
      assert sd.enabled and sd.casper_restart.phase == 'holding'
      assert sd.casper_restart.request_ns == 0
  assert not sd.enabled and sd.casper_restart.phase == 'off'


@pytest.mark.parametrize('button_type', ['accelCruise', 'resumeCruise', 'decelCruise'])
def test_pedal_while_res_held_cannot_restore_automatic_episode(button_type):
  sd, cs = setup()
  for i in range(110):
    tick(sd, cs, 1_000_000_000 + i * 10_000_000)
  cs.buttonEvents = [car.CarState.ButtonEvent.new_message(type=button_type, pressed=True)]
  tick(sd, cs, 2_100_000_000)
  cs.buttonEvents = []
  cs.brakePressed = True
  tick(sd, cs, 2_110_000_000)
  assert sd.casper_restart.phase == 'spent'
  cs.brakePressed = False
  cs.buttonEvents = [car.CarState.ButtonEvent.new_message(type=button_type, pressed=False)]
  tick(sd, cs, 2_120_000_000, departure=True)
  assert sd.casper_restart.phase == 'spent' and sd.casper_restart.request_ns == 0


@pytest.mark.parametrize('button_type', ['accelCruise', 'resumeCruise', 'decelCruise'])
@pytest.mark.parametrize('target', [0., -.2])
def test_waiting_speed_button_cannot_override_a_stop_or_nonpositive_plan(button_type, target):
  sd, cs = setup()
  for i in range(110):
    tick(sd, cs, 1_000_000_000 + i * 10_000_000)
  cs.buttonEvents = [car.CarState.ButtonEvent.new_message(type=button_type, pressed=True)]
  tick(sd, cs, 2_100_000_000, departure=True)
  cs.buttonEvents = [car.CarState.ButtonEvent.new_message(type=button_type, pressed=False)]
  tick(sd, cs, 2_110_000_000, departure=True)
  cs.buttonEvents = []
  tick(sd, cs, 2_120_000_000, departure=True, plan_target=target)
  assert sd.enabled and sd.casper_restart.request_ns == 0
  tick(sd, cs, 2_130_000_000, departure=True, plan_stop=True)
  assert sd.enabled and sd.casper_restart.request_ns == 0
  tick(sd, cs, 2_140_000_000, departure=True)
  assert not sd.enabled and sd.casper_restart.phase == 'off'
