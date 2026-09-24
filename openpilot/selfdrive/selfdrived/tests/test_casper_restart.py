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
                       departure=False, off_ack=False, plan_after_off=False, user_input=False, moving=False)

  def tick(self, **changes):
    self.inputs.update(changes)
    self.now += 10_000_000
    return self.controller.update(self.now, **self.inputs)

  def run(self, ticks, **changes):
    self.inputs.update(changes)
    return [action for _ in range(ticks) if (action := self.tick()) is not None]

  def request_off(self):
    assert self.run(110) == []
    assert self.run(11, stopping=False, departure=True) == ['off']


class TestCasperCruiseRestart(unittest.TestCase):
  def test_one_owned_cycle_waits_for_ack_and_new_plan(self):
    episode = Episode()
    episode.request_off()
    self.assertEqual(episode.run(20), [])
    self.assertEqual(episode.run(56, enabled=False, off_ack=True), [])
    self.assertEqual(episode.tick(plan_after_off=True), 'on')
    self.assertEqual(episode.run(200, enabled=True, off_ack=False), [])

  def test_off_dwell_is_measured_from_acknowledgement(self):
    episode = Episode()
    episode.request_off()
    episode.run(30)
    self.assertEqual(episode.run(55, enabled=False, off_ack=True, plan_after_off=True), [])
    self.assertEqual(episode.tick(), 'on')

  def test_departure_noise_does_not_cancel(self):
    episode = Episode()
    episode.run(110)
    self.assertEqual(episode.run(9, stopping=False, departure=True), [])
    episode.tick(stopping=True, departure=False)
    self.assertEqual(episode.run(9, stopping=False, departure=True), [])
    self.assertEqual(episode.controller.phase, 'holding')
    self.assertEqual(episode.run(2), ['off'])

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

  def test_off_planner_reset_does_not_prevent_owned_resume(self):
    # The caller independently validates the moving lead while OFF. A reset
    # longitudinal planner can report stopping until enabled again.
    episode = Episode()
    episode.request_off()
    self.assertEqual(episode.run(56, enabled=False, off_ack=True, departure=False,
                                 stopping=True, plan_after_off=True), ['on'])


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
