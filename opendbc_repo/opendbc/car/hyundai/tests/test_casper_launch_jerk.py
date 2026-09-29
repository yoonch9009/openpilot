import importlib.util
from pathlib import Path
import unittest


spec = importlib.util.spec_from_file_location('casper_launch_jerk', Path(__file__).parents[1] / 'casper_launch_jerk.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TestCasperLaunchJerk(unittest.TestCase):
  def setUp(self):
    self.helper = module.CasperLaunchJerk()
    self.now = 10.0

  def step(self, **changes):
    self.now += .02
    args = dict(active=True, stopping=False, speed=.5, healthy=True, feedback_fresh=True,
                brake_released=True, lead_valid=True, lead_distance=5., lead_relative_speed=.5,
                target=.8, request=.6, measured=.1, planned_jerk=.1, original=.5)
    args.update(changes)
    return self.helper.update(self.now, **args)

  def restart(self):
    self.step(stopping=True, speed=0, request=-.5, brake_released=False, lead_relative_speed=0.)
    self.step(speed=0, brake_released=False)
    for _ in range(25):
      self.step(active=False, speed=0, request=0, brake_released=False)
    self.step(speed=0, stopping=True, request=-.5, brake_released=False)

  def test_no_startup_or_stationary_assist(self):
    self.assertEqual(self.step(), .5)
    self.restart()
    for _ in range(20):
      self.assertEqual(self.step(speed=0), .5)
    self.assertGreater(self.step(), .5)

  def test_bound_and_slew_and_preserve_higher_original(self):
    self.restart()
    previous = .5
    for _ in range(35):
      value = self.step()
      self.assertLessEqual(value - previous, .020000001)
      self.assertLessEqual(value, 1.)
      previous = value
    self.assertEqual(value, 1.)
    self.assertEqual(self.step(original=2.), 2.)

  def test_invalid_input_cancels_window(self):
    for change in (dict(healthy=False), dict(feedback_fresh=False), dict(lead_valid=False),
                   dict(lead_distance=1.9), dict(lead_relative_speed=0), dict(planned_jerk=-.01),
                   dict(request=-.1), dict(stopping=True), dict(speed=float('nan'))):
      with self.subTest(change=change):
        self.setUp()
        self.restart()
        self.assertGreater(self.step(), .5)
        self.assertEqual(self.step(**change), .5)
        self.assertEqual(self.step(), .5)

  def test_no_boost_with_brake_or_no_deficit_or_high_speed(self):
    self.restart()
    for change in (dict(brake_released=False), dict(measured=.5), dict(speed=10 / 3.6)):
      self.assertEqual(self.step(**change), .5)

  def test_motion_wait_and_moving_window_expire(self):
    self.restart()
    for _ in range(110):
      self.assertEqual(self.step(speed=0, stopping=True, request=-.5), .5)
    self.assertEqual(self.step(), .5)
    self.setUp()
    self.restart()
    for _ in range(70):
      self.step(speed=0, stopping=True, request=-.5)
    for _ in range(145):
      self.assertGreater(self.step(), .5)
    for _ in range(10):
      self.step()
    self.assertEqual(self.step(), .5)

  def test_off_timeout_and_long_time_gap(self):
    self.restart()
    self.step(active=False, speed=0)
    for _ in range(110):
      self.step(active=False, speed=0)
    self.assertEqual(self.step(), .5)
    self.setUp()
    self.restart()
    self.now += .2
    self.assertEqual(self.step(), .5)

  def test_encoder_changes_only_upper_jerk_and_default_is_identical(self):
    try:
      from opendbc.can import CANParser
      from opendbc.car.hyundai.values import CAR, HyundaiFlags
      from opendbc.car.hyundai.tests.test_casper_departure_jerk import request, state
    except ImportError:
      self.skipTest('native CAN dependencies require device test environment')
    cs = state()
    cs.CP.flags = HyundaiFlags.CAMERA_SCC
    expected = request(cs)
    self.assertEqual(request(cs, launch_jerk_upper=None), expected)
    changed = request(cs, launch_jerk_upper=1.)
    self.assertEqual([m for m in changed if m[0] != 905], [m for m in expected if m[0] != 905])
    def decode(messages):
      parser = CANParser('hyundai_kia_generic', [('SCC14', 50)], 0)
      parser.update([1_000_000_000, messages])
      return dict(parser.vl['SCC14'])
    before, after = decode(expected), decode(changed)
    self.assertEqual(after.pop('JerkUpperLimit'), 1.)
    before.pop('JerkUpperLimit')
    self.assertEqual(before, after)
    cs.CP.carFingerprint = CAR.HYUNDAI_SONATA
    self.assertEqual(request(cs, launch_jerk_upper=1.), request(cs))
    cs.CP.carFingerprint = CAR.HYUNDAI_CASPER
    for flags in (0, HyundaiFlags.CAMERA_SCC | HyundaiFlags.CANFD):
      cs.CP.flags = flags
      self.assertEqual(request(cs, launch_jerk_upper=1.), request(cs))


if __name__ == '__main__':
  unittest.main()
