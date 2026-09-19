"""Stopped-lead regression and controller override/compatibility coverage."""
from types import SimpleNamespace

import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.controls.lib.following_stop import FollowingStop
from openpilot.selfdrive.controls.lib import longcontrol
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState, LongControl


def lead(distance=5.5, speed=0.0, track=1):
  return SimpleNamespace(status=True, radar=True, radarTrackId=track, dRel=distance,
                         vLead=speed, vLeadK=speed, vRel=speed)


class Scenario:
  def __init__(self):
    self.hold = FollowingStop()
    self.t = 1.0

  def step(self, target=None, dt=0.05, **overrides):
    self.t += dt
    args = dict(active=True, driver_override=False, standstill=True, v_ego=0.0,
                stopping=True, should_stop=True, following=True, lead=target or lead(),
                timestamp_ns=round(self.t * 1e9), fresh=True)
    args.update(overrides)
    return self.hold.update(**args)

  def stopped(self):
    assert self.step()
    return self


@pytest.mark.parametrize('should_stop', [True, False])
def test_completed_stop_latches_even_at_first_changed_plan(should_stop):
  s = Scenario()
  assert s.step(should_stop=should_stop)
  for _ in range(30):
    assert s.step(should_stop=False)


def test_can_latch_between_radar_samples():
  s = Scenario()
  assert not s.step(standstill=False, v_ego=0.15)
  assert s.step(dt=0, should_stop=False)


@pytest.mark.parametrize('overrides', [dict(active=False), dict(driver_override=True),
  dict(standstill=False), dict(v_ego=0.5), dict(stopping=False), dict(following=False),
  dict(fresh=False), dict(timestamp_ns=0)])
def test_does_not_latch_without_completed_following_stop(overrides):
  assert not Scenario().step(**overrides)


@pytest.mark.parametrize('target', [lead(speed=0.5), lead(distance=30), lead(distance=0), lead(distance=float('nan'))])
def test_unsuitable_lead_does_not_latch(target):
  assert not Scenario().step(target)


def test_persistent_departure_releases_without_relatching():
  s = Scenario().stopped()
  results = [s.step(lead(5.5 + i * 0.05, 1.0), should_stop=False) for i in range(1, 16)]
  assert all(results[:10])
  assert results[-1] is False
  assert not s.step(lead(6.5, 1.0), should_stop=False, stopping=False)


def test_one_velocity_or_distance_spike_cannot_release():
  s = Scenario().stopped()
  assert s.step(lead(5.55, 1.0), should_stop=False)
  for _ in range(20):
    assert s.step(should_stop=False)
  assert s.step(lead(12, 1.0), should_stop=False)
  for _ in range(20):
    assert s.step(lead(12, 1.0), should_stop=False)


def test_moving_speed_without_increasing_gap_is_not_departure():
  s = Scenario().stopped()
  for _ in range(40):
    assert s.step(lead(5.5, 1.0), should_stop=False)


def test_single_late_distance_jump_is_not_sustained_motion():
  s = Scenario().stopped()
  for _ in range(20):
    assert s.step(lead(5.5, 1), should_stop=False)
  assert s.step(lead(5.9, 1), should_stop=False)
  for _ in range(20):
    assert s.step(lead(5.9, 1), should_stop=False)


def test_gap_oscillation_resets_departure_confirmation():
  s = Scenario().stopped()
  for _ in range(20):
    for distance in [5.6, 5.7, 5.8, 5.65]:
      assert s.step(lead(distance, 1), should_stop=False)


def test_new_stop_plan_on_duplicate_radar_sample_resets_confirmation():
  s = Scenario().stopped()
  for i in range(8):
    assert s.step(lead(5.5 + i * 0.05, 1), should_stop=False)
  assert s.step(lead(5.85, 1), dt=0, should_stop=True)
  for i in range(8):
    assert s.step(lead(5.9 + i * 0.05, 1), should_stop=False)


def test_repeated_sample_and_stale_data_cannot_release():
  s = Scenario().stopped()
  assert s.step(lead(5.6, 1), should_stop=False)
  for _ in range(100):
    assert s.step(lead(6, 1), dt=0, should_stop=False)
  assert s.step(lead(6, 1), fresh=False, should_stop=False)


@pytest.mark.parametrize('kind', ['lost', 'changed', 'gap', 'backwards', 'invalid'])
def test_uncertainty_requires_stationary_reacquisition(kind):
  s = Scenario().stopped()
  moving = lead(5.6, 1)
  if kind == 'lost':
    moving.status = False
  elif kind == 'changed':
    moving.radarTrackId = 2
  elif kind == 'invalid':
    moving.vLead = float('nan')
  overrides = dict(dt=0.5) if kind == 'gap' else dict(dt=-0.1) if kind == 'backwards' else {}
  assert s.step(moving, should_stop=False, **overrides)
  for i in range(20):
    assert s.step(lead(5.6 + i * 0.05, 1), should_stop=False)
  for _ in range(20):
    assert s.step(lead(6.6), should_stop=True)
  results = [s.step(lead(6.6 + i * 0.05, 1), should_stop=False) for i in range(1, 16)]
  assert not results[-1]


def test_lead_stops_again_or_planner_requests_stop_resets_confirmation():
  s = Scenario().stopped()
  for i in range(8):
    assert s.step(lead(5.5 + i * 0.05, 1), should_stop=False)
  assert s.step(lead(5.9), should_stop=True)
  for i in range(8):
    assert s.step(lead(5.9 + i * 0.05, 1), should_stop=False)


def test_other_planner_source_cannot_release_hold():
  s = Scenario().stopped()
  for i in range(20):
    assert s.step(lead(5.5 + i * 0.05, 1), should_stop=False, following=False)


@pytest.mark.parametrize('override', [dict(active=False), dict(driver_override=True)])
def test_inactive_or_driver_override_resets_hold(override):
  s = Scenario().stopped()
  assert not s.step(**override)
  assert not s.step(stopping=False, should_stop=False)


@pytest.fixture
def controller(monkeypatch):
  monkeypatch.setattr(longcontrol, 'Params', lambda: SimpleNamespace(get_float=lambda name: 0.0))
  cp = car.CarParams.new_message(brand='hyundai', carFingerprint='HYUNDAI_CASPER',
    openpilotLongitudinalControl=True, pcmCruise=False, startingState=False, vEgoStarting=0.1,
    stopAccel=-2.0, stoppingDecelRate=0.8)
  cp.longitudinalTuning.kpBP = [0.0]
  cp.longitudinalTuning.kpV = [1.0]
  cp.longitudinalTuning.kiBP = [0.0]
  cp.longitudinalTuning.kiV = [0.0]
  cp.longitudinalTuning.kf = 1.0
  return cp


def controller_inputs():
  cs = car.CarState.new_message(vEgo=0, aEgo=0, standstill=True, gearShifter='drive')
  plan = log.LongitudinalPlan.new_message(shouldStop=False, aTarget=0.4, vTargetNow=1.0,
                                         longitudinalPlanSource='lead0')
  radar = log.RadarState.new_message()
  radar.leadOne.status = True
  radar.leadOne.dRel = 5.5
  radar.leadOne.radar = True
  radar.leadOne.radarTrackId = 1
  return cs, plan, radar


def update(lc, cs, plan, radar, *, active=True, stamp=1_000_000_000):
  return lc.update(active, cs, plan, [-3.5, 2.0], 0, radar,
                   radar_timestamp_ns=stamp, radar_age=0, radar_valid=True)[0]


def stopped_controller(cp):
  lc = LongControl(cp)
  lc.long_control_state = LongCtrlState.stopping
  lc.last_output_accel = -0.5
  return lc


def test_stationary_lead_does_not_release_on_changed_plan(controller):
  lc = stopped_controller(controller)
  cs, plan, radar = controller_inputs()
  for i in range(50):
    assert update(lc, cs, plan, radar, stamp=1_000_000_000 + i * 50_000_000) <= -0.5
    assert lc.long_control_state == LongCtrlState.stopping


def test_stronger_planned_braking_is_preserved(controller):
  lc = stopped_controller(controller)
  cs, plan, radar = controller_inputs()
  plan.aTarget = -1.5
  assert update(lc, cs, plan, radar) <= -1.5


def test_controller_resumes_after_confirmed_departure(controller):
  lc = stopped_controller(controller)
  cs, plan, radar = controller_inputs()
  assert update(lc, cs, plan, radar) <= -0.5
  radar.leadOne.vLead = 1
  radar.leadOne.vLeadK = 1
  radar.leadOne.vRel = 1
  for i in range(1, 17):
    radar.leadOne.dRel = 5.5 + i * 0.05
    accel = update(lc, cs, plan, radar, stamp=1_000_000_000 + i * 50_000_000)
    if i < 10:
      assert accel <= -0.5
  assert lc.long_control_state == LongCtrlState.pid
  assert accel > 0


@pytest.mark.parametrize('pedal', ['gasPressed', 'brakePressed'])
def test_pedal_clears_added_hold(controller, pedal):
  lc = stopped_controller(controller)
  cs, plan, radar = controller_inputs()
  update(lc, cs, plan, radar)
  setattr(cs, pedal, True)
  update(lc, cs, plan, radar, stamp=1_050_000_000)
  assert not lc.following_stop.holding
  if pedal == 'brakePressed':
    assert lc.long_control_state == LongCtrlState.stopping


def test_disabled_controller_does_not_force_braking(controller):
  lc = stopped_controller(controller)
  cs, plan, radar = controller_inputs()
  update(lc, cs, plan, radar)
  assert update(lc, cs, plan, radar, active=False) == 0
  assert lc.long_control_state == LongCtrlState.off
  assert not lc.following_stop.holding


@pytest.mark.parametrize('change', [dict(carFingerprint='HYUNDAI_CASPER_EV'),
  dict(carFingerprint='HYUNDAI_SONATA'), dict(brand='toyota'),
  dict(openpilotLongitudinalControl=False), dict(pcmCruise=True)])
def test_other_configurations_keep_previous_release_behavior(controller, change):
  for key, value in change.items():
    setattr(controller, key, value)
  lc = stopped_controller(controller)
  cs, plan, radar = controller_inputs()
  assert update(lc, cs, plan, radar) > 0
  assert lc.long_control_state == LongCtrlState.pid
  assert not lc.following_stop_enabled
