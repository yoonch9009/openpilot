"""Regression specification; no production stop-hold change is enabled yet."""
import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState, long_control_state_trans


def transition_from_following_stop(*, active=True, brake_pressed=False, should_stop=False):
  cp = car.CarParams.new_message(startingState=False, vEgoStarting=0.1)
  radar = log.RadarState.new_message()
  radar.leadOne.status = True
  radar.leadOne.dRel = 5.5
  radar.leadOne.vLead = 0.0
  return long_control_state_trans(
    cp, active, LongCtrlState.stopping, v_ego=0.0,
    should_stop=should_stop, brake_pressed=brake_pressed,
    cruise_standstill=False, a_ego=0.0, stopping_accel=-0.5,
    radarState=radar,
  )


@pytest.mark.xfail(strict=True, reason="Following-stop latch is not implemented; see docs/development/casper-stop-hold.md")
def test_stationary_lead_does_not_release_on_one_changed_plan():
  assert transition_from_following_stop() == LongCtrlState.stopping


def test_driver_brake_prevents_release():
  assert transition_from_following_stop(brake_pressed=True) == LongCtrlState.stopping


def test_disabled_control_does_not_force_hold():
  assert transition_from_following_stop(active=False, should_stop=True) == LongCtrlState.off
