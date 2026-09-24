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
    self.logMonoTime = {k: now - 1_000_000 for k in self}
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


def tick(sd, cs, now, *, departure=False, stale_cs=False, stale_source=None, driver_cancel=False):
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
  controls.longitudinalPlanMonoTime = sd.sm.logMonoTime['longitudinalPlan']
  plan.hasLead = True
  plan.shouldStop = not departure or not sd.enabled
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
  for i in range(1, 56):
    tick(sd, cs, now + i * 10_000_000, departure=True)
    assert not sd.enabled
  tick(sd, cs, now + 560_000_000, departure=True)
  assert sd.enabled and sd.casper_restart.reason == 'resume_requested'


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
