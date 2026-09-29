import importlib.util
from pathlib import Path
import unittest

try:
  from openpilot.selfdrive.selfdrived.casper_restart import CasperCruiseRestart
except ModuleNotFoundError:
  spec = importlib.util.spec_from_file_location('casper_restart', Path(__file__).parents[1] / 'casper_restart.py')
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  CasperCruiseRestart = module.CasperCruiseRestart


class Episode:
  def __init__(self):
    self.controller = CasperCruiseRestart()
    self.now = 1_000_000_000
    self.inputs = dict(enabled=True, healthy=True, stationary=True, stopping=True,
                       departure=False, off_ack=False, plan_after_off=False, user_input=False, moving=False, active_ack=True)

  def tick(self, **changes):
    self.inputs.update(changes)
    self.now += 10_000_000
    return self.controller.update(self.now, **self.inputs)

  def run(self, ticks, **changes):
    self.inputs.update(changes)
    return [action for _ in range(ticks) if (action := self.tick()) is not None]

  def request_off(self):
    assert self.run(110) == []
    assert self.tick(stopping=False, departure=True) == 'off'


class TestCasperCruiseRestart(unittest.TestCase):
  def test_one_owned_cycle_waits_for_ack_and_new_plan(self):
    episode = Episode()
    episode.request_off()
    self.assertEqual(episode.run(5), [])
    self.assertEqual(episode.run(10, enabled=False, off_ack=True), [])
    self.assertEqual(episode.tick(plan_after_off=True), 'on')
    self.assertEqual(episode.run(200, enabled=True, off_ack=False), [])

  def test_no_fixed_dwell_after_required_acknowledgments(self):
    episode = Episode()
    episode.request_off()
    self.assertEqual(episode.run(5, plan_after_off=True), [])
    self.assertIsNone(episode.tick(enabled=False, off_ack=True, plan_after_off=False))
    self.assertEqual(episode.tick(plan_after_off=True), 'on')

  def test_planner_permission_requests_off_without_extra_confirmation(self):
    episode = Episode()
    episode.run(110)
    self.assertIsNone(episode.tick(departure=False))
    self.assertEqual(episode.tick(stopping=False, departure=True), 'off')

  def test_resume_lead_wait_is_bounded_and_does_not_abort_immediately(self):
    episode = Episode()
    episode.request_off()
    self.assertEqual(episode.run(10, enabled=False, off_ack=True,
                                 plan_after_off=True, resume_ready=False), [])
    self.assertEqual(episode.controller.phase, 'off')
    self.assertEqual(episode.tick(resume_ready=True), 'on')
    episode = Episode()
    episode.request_off()
    self.assertEqual(episode.run(170, enabled=False, off_ack=True,
                                 plan_after_off=True, resume_ready=False), [])
    self.assertEqual(episode.controller.phase, 'spent')
    self.assertIsNone(episode.tick(resume_ready=True))

  def test_cancel_at_shortened_resume_boundary_prevents_enable(self):
    episode = Episode()
    episode.request_off()
    episode.tick(enabled=False, off_ack=True, plan_after_off=False)
    self.assertIsNone(episode.tick(user_input=True, plan_after_off=True))
    self.assertEqual(episode.run(100, user_input=False), [])

  def test_stationary_engagement_without_prior_stop_does_not_cycle(self):
    episode = Episode()
    self.assertEqual(episode.run(200, stopping=False, departure=True), [])

  def test_initial_disabled_requires_new_moving_episode(self):
    episode = Episode()
    episode.tick(enabled=False)
    self.assertEqual(episode.run(200, enabled=True), [])
    self.assertEqual(episode.run(20, stopping=False, departure=True), [])
    episode.tick(moving=True, stationary=False)
    episode.tick(moving=False, stationary=True, stopping=True, departure=False)
    episode.request_off()

  def test_missing_ack_or_new_plan_times_out_without_retry(self):
    for acknowledged in (False, True):
      with self.subTest(acknowledged=acknowledged):
        episode = Episode()
        episode.request_off()
        self.assertEqual(episode.run(160, enabled=not acknowledged, off_ack=acknowledged), [])
        self.assertEqual(episode.run(100, enabled=False, off_ack=True, plan_after_off=True), [])
        self.assertEqual(episode.controller.phase, 'spent')

  def test_input_or_invalid_data_aborts_each_phase(self):
    for phase in ('idle', 'holding', 'off', 'acknowledged'):
      for fault in ({'user_input': True}, {'healthy': False}):
        with self.subTest(phase=phase, fault=fault):
          episode = Episode()
          if phase == 'holding':
            episode.run(110)
          elif phase in ('off', 'acknowledged'):
            episode.request_off()
            if phase == 'acknowledged':
              episode.tick(enabled=False, off_ack=True)
          self.assertIsNone(episode.tick(**fault))
          self.assertEqual(episode.run(200, healthy=True, user_input=False, enabled=False,
                                       off_ack=True, plan_after_off=True, departure=True), [])

  def test_driver_cancel_cannot_be_undone_after_cycle(self):
    episode = Episode()
    episode.request_off()
    self.assertEqual(episode.run(56, enabled=False, off_ack=True, plan_after_off=True), ['on'])
    episode.tick(enabled=True)
    episode.tick(enabled=False, user_input=True)
    self.assertEqual(episode.run(200, user_input=False), [])

  def test_unowned_disable_and_enable_abort(self):
    episode = Episode()
    episode.run(110)
    episode.tick(enabled=False)
    self.assertEqual(episode.run(200, enabled=True, stopping=False, departure=True), [])
    episode = Episode()
    episode.request_off()
    episode.tick(enabled=False, off_ack=True)
    episode.tick(enabled=True)
    self.assertEqual(episode.run(100, enabled=False, plan_after_off=True), [])

  def test_time_discontinuity_prevents_resume(self):
    for delta in (-20_000_000, 0, 150_000_000):
      with self.subTest(delta=delta):
        episode = Episode()
        episode.request_off()
        episode.tick(enabled=False, off_ack=True, plan_after_off=True)
        episode.now += delta - 10_000_000
        self.assertIsNone(episode.tick())
        self.assertEqual(episode.run(200), [])

  def test_small_motion_does_not_rearm_episode(self):
    episode = Episode()
    episode.request_off()
    episode.tick(stationary=False)
    self.assertEqual(episode.run(120, stationary=True, stopping=True, departure=False), [])
    self.assertEqual(episode.run(20, stopping=False, departure=True), [])

  def test_caller_plan_permission_is_required_during_owned_off(self):
    episode = Episode()
    episode.request_off()
    self.assertEqual(episode.run(10, enabled=False, off_ack=True, departure=False,
                                 stopping=True, plan_after_off=True, resume_ready=False), [])
    self.assertEqual(episode.tick(resume_ready=True), 'on')

  def test_marker_spans_on_request_until_controller_ack(self):
    episode = Episode()
    episode.request_off()
    marker = episode.controller.request_ns
    self.assertEqual(episode.controller.plan_marker(episode.now), marker)
    episode.tick(enabled=False, off_ack=True)
    self.assertEqual(episode.tick(plan_after_off=True), 'on')
    self.assertEqual(episode.controller.plan_marker(episode.now), marker)
    episode.tick(enabled=True, active_ack=True)
    self.assertEqual(episode.controller.reason, 'resume_complete')
    self.assertEqual(episode.controller.plan_marker(episode.now), 0)

  def test_epoch_expires_without_refreshing_at_boundary(self):
    episode = Episode()
    episode.request_off()
    request = episode.controller.request_ns
    self.assertEqual(episode.run(29, enabled=False, off_ack=True), [])
    self.assertEqual(episode.controller.plan_marker(request + 299_999_999), request)
    self.assertEqual(episode.controller.plan_marker(request + 300_000_000), 0)
    self.assertIsNone(episode.tick(plan_after_off=True))
    self.assertEqual(episode.controller.phase, 'spent')

  def test_missing_active_ack_cancels_owned_reentry_once(self):
    episode = Episode()
    episode.request_off()
    episode.tick(enabled=False, off_ack=True, active_ack=False)
    self.assertEqual(episode.tick(plan_after_off=True), 'on')
    self.assertEqual(episode.run(30, enabled=True), ['off'])
    self.assertEqual(episode.run(100), [])


class TestRestartWithRealStateMachine(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    try:
      from openpilot.cereal import log
      from openpilot.selfdrive.selfdrived.events import Events
      from openpilot.selfdrive.selfdrived.state import StateMachine
    except ImportError as error:
      raise unittest.SkipTest(f'Native openpilot runtime unavailable: {error}') from error
    cls.Events, cls.StateMachine = Events, StateMachine
    cls.EventName = log.OnroadEvent.EventName

  def test_cancel_enable_obeys_no_entry_and_real_cancel(self):
    for blocker in (None, 'doorOpen', 'buttonCancel', 'controlsMismatch'):
      with self.subTest(blocker=blocker):
        machine = self.StateMachine()
        events = self.Events()
        events.add(self.EventName.buttonEnable)
        enabled, _ = machine.update(events)
        self.assertTrue(enabled)
        episode = Episode()
        episode.request_off()
        events.clear()
        events.add(self.EventName.buttonCancel)
        enabled, _ = machine.update(events)
        self.assertFalse(enabled)
        self.assertEqual(episode.run(56, enabled=enabled, off_ack=True, plan_after_off=True), ['on'])
        events.clear()
        events.add(self.EventName.buttonEnable)
        if blocker:
          events.add(getattr(self.EventName, blocker))
        enabled, _ = machine.update(events)
        self.assertEqual(enabled, blocker is None)
        self.assertEqual(episode.run(200, enabled=enabled), [])


if __name__ == '__main__':
  unittest.main()
