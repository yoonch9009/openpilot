import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

try:
  from openpilot.selfdrive.controls.lib.casper_restart_plan import accepted_restart_request, fresh_input_snapshot_time, planner_reset_state
except ModuleNotFoundError:
  spec = importlib.util.spec_from_file_location('casper_restart_plan', Path(__file__).parents[1] / 'casper_restart_plan.py')
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  accepted_restart_request, planner_reset_state = module.accepted_restart_request, module.planner_reset_state
  fresh_input_snapshot_time = module.fresh_input_snapshot_time


class Messages(dict):
  def __init__(self, now):
    super().__init__(selfdriveState=NS(casperRestartRequestMonoTime=now - 20_000_000),
                     carState=NS(gearShifter='drive', canValid=True, canTimeout=False, accFaulted=False,
                                 gasPressed=False, brakePressed=False, parkingBrake=False, brakeHoldActive=False,
                                 softHoldActive=0, buttonEvents=[]), controlsState=NS(),
                     radarState=NS(leadOne=NS(status=True, dRel=6.0, vRel=0.7)))
    self.logMonoTime = dict.fromkeys(self, now - 10_000_000)
    self.alive = dict.fromkeys(self, True)
    self.valid = dict.fromkeys(self, True)


class TestCasperRestartPlan(unittest.TestCase):
  def setUp(self):
    self.now = 2_000_000_000
    self.sm = Messages(self.now)

  def accepted(self, **changes):
    args = dict(eligible=True, soft_hold_active=False, force_decel=False, prior_departure_ns=1_950_000_000)
    args.update(changes)
    return accepted_restart_request(self.sm, self.now, **args)

  def reset(self, **changes):
    args = dict(openpilot_longitudinal=True, long_control_off=True, enabled=False,
                cruise_initialized=True, soft_hold_active=False, accepted_request=self.accepted())
    args.update(changes)
    return planner_reset_state(**args)

  def test_owned_off_preserves_only_with_exact_request_echo(self):
    request = self.sm['selfdriveState'].casperRestartRequestMonoTime
    self.assertEqual(self.reset(), (False, request))
    self.assertEqual(self.reset(long_control_off=False, enabled=True), (False, request))
    # Neither helper writes enable, acceleration, nor a replacement request time.
    self.assertEqual(self.sm['selfdriveState'].casperRestartRequestMonoTime, request)

  def test_fresh_planner_or_history_after_request_cannot_claim_continuity(self):
    request = self.sm['selfdriveState'].casperRestartRequestMonoTime
    for history in (0, request + 1, request - 300_000_001):
      with self.subTest(history=history):
        self.assertEqual(self.accepted(prior_departure_ns=history), 0)
    self.assertEqual(self.accepted(prior_departure_ns=request - 300_000_000), request)

  def test_request_arriving_during_solver_keeps_input_epoch_baseline(self):
    self.sm['selfdriveState'].casperRestartRequestMonoTime = 0
    input_time = self.sm.logMonoTime['selfdriveState']
    request_time = self.now + 20_000_000
    completed_time = self.now + 60_000_000
    baseline = fresh_input_snapshot_time(self.sm, completed_time)
    self.assertEqual(baseline, input_time)
    self.now = completed_time + 10_000_000
    self.sm = Messages(self.now)
    self.sm['selfdriveState'].casperRestartRequestMonoTime = request_time
    self.assertEqual(self.accepted(prior_departure_ns=baseline), request_time)
    self.assertEqual(self.accepted(prior_departure_ns=completed_time), 0)

  def test_slow_solver_cannot_turn_stale_or_invalid_inputs_into_recent_history(self):
    self.assertEqual(fresh_input_snapshot_time(self.sm, self.now + 100_000_000), 0)
    for service in self.sm:
      with self.subTest(service=service):
        self.sm = Messages(self.now)
        self.sm.valid[service] = False
        self.assertEqual(fresh_input_snapshot_time(self.sm, self.now), 0)

  def test_old_message_and_manual_off_reset(self):
    del self.sm['selfdriveState'].casperRestartRequestMonoTime
    self.assertEqual(self.reset(), (True, 0))
    self.sm['selfdriveState'].casperRestartRequestMonoTime = 0
    self.assertEqual(self.reset(), (True, 0))

  def test_timestamp_expiration_is_not_refreshed_by_new_messages(self):
    request = self.sm['selfdriveState'].casperRestartRequestMonoTime
    self.now = request + 300_000_001
    self.sm.logMonoTime = dict.fromkeys(self.sm, self.now)
    self.assertEqual(self.reset(), (True, 0))
    self.sm['selfdriveState'].casperRestartRequestMonoTime = self.now + 1
    self.assertEqual(self.reset(), (True, 0))

  def test_each_service_rejects_stale_future_invalid_or_dead(self):
    for service, age in (('selfdriveState', 100_000_000), ('carState', 100_000_000),
                         ('controlsState', 100_000_000), ('radarState', 300_000_000)):
      for fault in ('stale', 'future', 'zero', 'invalid', 'dead'):
        with self.subTest(service=service, fault=fault):
          self.sm = Messages(self.now)
          if fault == 'stale':
            self.sm.logMonoTime[service] = self.now - age - 1
          elif fault == 'future':
            self.sm.logMonoTime[service] = self.now + 1
          elif fault == 'zero':
            self.sm.logMonoTime[service] = 0
          elif fault == 'invalid':
            self.sm.valid[service] = False
          else:
            self.sm.alive[service] = False
          self.assertEqual(self.reset(), (True, 0))

  def test_driver_vehicle_and_hold_conditions_preserve_reset(self):
    for field, value in (('gearShifter', 'park'), ('canValid', False), ('canTimeout', True),
                         ('accFaulted', True), ('gasPressed', True), ('brakePressed', True),
                         ('parkingBrake', True), ('brakeHoldActive', True), ('softHoldActive', 1),
                         ('buttonEvents', [NS(type='cancel', pressed=True)])):
      with self.subTest(field=field):
        self.sm = Messages(self.now)
        setattr(self.sm['carState'], field, value)
        self.assertEqual(self.reset(), (True, 0))
    self.sm = Messages(self.now)
    for guard in ('eligible', 'soft_hold_active', 'force_decel'):
      self.assertEqual(self.accepted(**{guard: guard != 'eligible'}), 0)

  def test_missing_closing_or_nonfinite_lead_rejects_marker(self):
    for field, value in (('status', False), ('dRel', 0), ('dRel', -1), ('dRel', float('nan')),
                         ('dRel', float('inf')), ('vRel', -0.01), ('vRel', float('nan')),
                         ('vRel', float('inf'))):
      with self.subTest(field=field, value=value):
        self.sm = Messages(self.now)
        setattr(self.sm['radarState'].leadOne, field, value)
        self.assertEqual(self.reset(), (True, 0))

  def test_ordinary_reset_causes_have_priority(self):
    self.assertEqual(self.reset(cruise_initialized=False), (True, 0))
    self.assertEqual(self.reset(soft_hold_active=True), (True, 0))
    self.assertEqual(self.reset(openpilot_longitudinal=False), (True, 0))

  def test_no_marker_keeps_existing_enabled_behavior(self):
    self.sm['selfdriveState'].casperRestartRequestMonoTime = 0
    self.assertEqual(self.reset(long_control_off=False, enabled=True), (False, 0))
    self.assertEqual(self.reset(openpilot_longitudinal=False, enabled=True), (False, 0))

  def test_zero_relative_speed_does_not_fabricate_departure(self):
    self.sm['radarState'].leadOne.vRel = 0.0
    before = vars(self.sm['radarState'].leadOne).copy()
    self.assertGreater(self.accepted(), 0)
    self.assertEqual(vars(self.sm['radarState'].leadOne), before)


class TestRealPlannerResetIntegration(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    try:
      from openpilot.selfdrive.controls.lib import longitudinal_planner
    except ImportError as error:
      raise unittest.SkipTest(f'Native planner runtime unavailable: {error}') from error
    cls.module = longitudinal_planner

  def test_planner_eligibility_is_exact_classic_gasoline_casper(self):
    module = self.module
    flags = module.HyundaiFlags
    for fingerprint, vehicle_flags, op_long, pcm, expected in (
      (module.CAR.HYUNDAI_CASPER, flags.CAMERA_SCC, True, False, True),
      (module.CAR.HYUNDAI_CASPER_EV, flags.CAMERA_SCC, True, False, False),
      (module.CAR.HYUNDAI_CASPER, flags.CAMERA_SCC | flags.CANFD, True, False, False),
      (module.CAR.HYUNDAI_CASPER, 0, True, False, False),
      (module.CAR.HYUNDAI_CASPER, flags.CAMERA_SCC, False, False, False),
      (module.CAR.HYUNDAI_CASPER, flags.CAMERA_SCC, True, True, False),
    ):
      with self.subTest(fingerprint=fingerprint, flags=vehicle_flags, op_long=op_long, pcm=pcm):
        cp = NS(carFingerprint=fingerprint, flags=vehicle_flags, openpilotLongitudinalControl=op_long, pcmCruise=pcm)
        with patch.object(module, 'LongitudinalMpc'), patch.object(module, 'Params'), \
             patch.object(module, 'is_volkswagen_meb', return_value=False):
          planner = module.LongitudinalPlanner(cp)
        self.assertEqual(planner.casper_restart_eligible, expected)

  def test_owned_off_preserves_state_but_manual_off_and_soft_hold_reset(self):
    module = self.module

    class ReachedLiveMpc(Exception):
      pass

    for marker, hold, primed, expected_reset in ((True, 0, True, False), (False, 0, True, True),
                                               (True, 1, True, True), (True, 0, False, True)):
      with self.subTest(marker=marker, hold=hold, primed=primed):
        now = 2_000_000_000
        sm = Messages(now)
        sm['selfdriveState'].experimentalMode = False
        sm['selfdriveState'].enabled = False
        sm['selfdriveState'].personality = 0
        if not marker:
          sm['selfdriveState'].casperRestartRequestMonoTime = 0
        sm['carState'].__dict__.update(vEgo=0., vCruise=80., vCluRatio=1., aEgo=0., standstill=True)
        sm['controlsState'].__dict__.update(longControlState=module.LongCtrlState.off, forceDecel=False,
                                            desiredCurvature=0., curvature=0.)
        sm['carControl'] = NS(orientationNED=[])
        sm['modelV2'] = NS()
        planner = module.LongitudinalPlanner.__new__(module.LongitudinalPlanner)
        planner.CP = NS(openpilotLongitudinalControl=True)
        planner.casper_restart_eligible = True
        planner.casper_prior_departure_ns = now - 50_000_000 if primed else 0
        planner.coasting_param_time = 0
        planner.dt = .05
        planner.a_desired = .6
        planner.reset_decel_timer = 0
        planner.v_desired_filter = NS(x=.4, update=lambda speed: .4)
        planner.output_should_stop = False
        planner.update_lead_tracks = lambda radar: (0, 0)
        planner.parse_model = lambda model: ([], [], [], [], 1.)
        captured = {}

        def solve(carrot, reset, radar, *args, **kwargs):
          captured.update(reset=reset, radar=radar)
          raise ReachedLiveMpc

        planner.mpc = NS(mode='acc', set_accel_limits=lambda *args: None,
                         set_cur_state=lambda *args: None, update=solve)
        carrot = NS(mode='acc', update=lambda *args: 80., soft_hold_active=hold,
                    get_carrot_accel=lambda speed: 2., leadAccelResponse=0,
                    lane_change_active=False, jerk_factor=1., aChangeCostStarting=1.)
        with patch.object(module, 'monotonic_ns', return_value=now), \
             patch.object(module, 'get_future_curvature', return_value=0.), \
             patch.object(module, 'limit_accel_in_turns', side_effect=lambda v, c, limits, *a, **k: limits[:]), \
             patch.object(module, 'get_cutin_predecel_accel_limit', return_value=None), \
             patch.object(module, 'apply_cutin_predecel_accel_limit', side_effect=lambda upper, *args: upper):
          with self.assertRaises(ReachedLiveMpc):
            planner.update(sm, carrot)
        self.assertEqual(captured['reset'], expected_reset)
        self.assertIs(captured['radar'], sm['radarState'])
        self.assertEqual(planner.a_desired, 0. if expected_reset else .6)
        self.assertEqual(planner.casper_restart_request_ns,
                         0 if expected_reset else sm['selfdriveState'].casperRestartRequestMonoTime)
        self.assertEqual(planner.casper_prior_departure_ns, 0 if expected_reset else now - 50_000_000)


if __name__ == '__main__':
  unittest.main()
