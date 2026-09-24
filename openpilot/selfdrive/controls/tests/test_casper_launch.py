import unittest

from openpilot.selfdrive.controls.lib.casper_launch import CasperReengageAccel


class TestCasperReengageAccel(unittest.TestCase):
  def setUp(self):
    self.helper = CasperReengageAccel()

  def step(self, **changes):
    inputs = dict(active=True, stopping=False, speed=0.5, measured_accel=0.1,
                  target_accel=0.5, target_speed=0.4, requested_accel=0.2,
                  lead_valid=True, lead_distance=5.0, lead_relative_speed=0.5,
                  driver_input=False, control_valid=True)
    inputs.update(changes)
    return self.helper.update(**inputs)

  def reenable(self):
    self.step(stopping=True, speed=0.0, requested_accel=-0.5)
    self.step(active=False, speed=0.0, requested_accel=0.0)
    self.step(speed=0.0, stopping=True, target_accel=0.0, requested_accel=-0.5)

  def test_no_startup_or_continuous_stop_boost(self):
    self.assertEqual(self.step(), 0.2)
    self.step(stopping=True, speed=0.0)
    self.assertEqual(self.step(), 0.2)

  def test_bounded_rise_and_target_cap(self):
    self.reenable()
    previous = 0.2
    for _ in range(60):
      result = self.step()
      self.assertLessEqual(result - previous, 0.005 + 1e-9)
      self.assertLessEqual(result, 0.4 + 1e-9)
      previous = result
    self.assertAlmostEqual(previous, 0.4)
    self.assertAlmostEqual(self.step(target_accel=0.23), 0.23)
    self.assertEqual(self.step(requested_accel=0.6), 0.6)

  def test_no_stationary_brake_boost_and_initial_stopping_allowed(self):
    self.reenable()
    for _ in range(30):
      self.assertEqual(self.step(stopping=True, speed=0.0, requested_accel=-0.5), -0.5)
    self.assertGreater(self.helper.remaining_s, 0.0)
    self.assertEqual(self.step(speed=0.0), 0.2)
    self.assertGreater(self.step(), 0.2)

  def test_immediate_drop_and_no_reactivation_after_invalid_inputs(self):
    for invalid in (dict(driver_input=True), dict(control_valid=False), dict(lead_valid=False),
                    dict(lead_relative_speed=-0.1), dict(lead_distance=2.9),
                    dict(dt=0.0), dict(dt=0.1), dict(speed=float('nan'))):
      with self.subTest(invalid=invalid):
        self.helper = CasperReengageAccel()
        self.reenable()
        self.assertGreater(self.step(), 0.2)
        self.assertEqual(self.step(**invalid), 0.2)
        self.assertEqual(self.helper.correction, 0.0)
        self.assertEqual(self.step(), 0.2)

  def test_eligibility_drops_correction(self):
    for blocked in (dict(stopping=True), dict(requested_accel=-0.1), dict(target_accel=-0.1),
                    dict(target_speed=0.6), dict(speed=0.09), dict(speed=3.0), dict(measured_accel=0.49)):
      with self.subTest(blocked=blocked):
        self.helper = CasperReengageAccel()
        self.reenable()
        self.step()
        expected = blocked.get('requested_accel', 0.2)
        self.assertEqual(self.step(**blocked), expected)
        self.assertEqual(self.helper.correction, 0.0)

  def test_off_preserves_request_and_timeout_disarms(self):
    self.step(stopping=True, speed=0.0)
    for _ in range(201):
      self.assertEqual(self.step(active=False, speed=0.0, requested_accel=-0.1), -0.1)
    self.step(speed=0.0)
    self.assertEqual(self.step(), 0.2)

  def test_window_expires(self):
    self.reenable()
    for _ in range(301):
      self.step()
    self.assertEqual(self.helper.remaining_s, 0.0)
    self.assertEqual(self.step(), 0.2)

  def test_two_second_off_boundary_and_departing_lead_required(self):
    self.step(stopping=True, speed=0.0)
    for _ in range(200):
      self.step(active=False, speed=0.0)
    self.step(speed=0.0)
    self.assertGreater(self.step(), 0.2)
    for first_input in (dict(lead_valid=False), dict(lead_relative_speed=0.0), dict(speed=0.3)):
      with self.subTest(first_input=first_input):
        self.helper = CasperReengageAccel()
        self.step(stopping=True, speed=0.0)
        self.step(active=False, speed=0.0)
        inputs = dict(speed=0.0)
        inputs.update(first_input)
        self.step(**inputs)
        self.assertEqual(self.step(), 0.2)

  def test_measured_deficit_caps_correction(self):
    self.reenable()
    for _ in range(60):
      result = self.step(measured_accel=0.44)
    self.assertAlmostEqual(result, 0.26)

  def test_stop_then_stationary_pid_before_cancel_preserves_arm(self):
    self.step(stopping=True, speed=0.0, requested_accel=-0.5)
    for _ in range(15):
      self.assertEqual(self.step(speed=0.0), 0.2)
    for _ in range(55):
      self.assertEqual(self.step(active=False, speed=0.0, requested_accel=0.0), 0.0)
    self.step(speed=0.0, stopping=True)
    self.assertGreater(self.step(), 0.2)

  def test_motion_before_cancel_disarms_preceding_stop(self):
    self.step(stopping=True, speed=0.0)
    self.step(speed=0.3)
    self.step(speed=0.0)
    self.step(active=False, speed=0.0)
    self.step(speed=0.0)
    self.assertEqual(self.step(), 0.2)

  def test_repeated_stop_cycles_require_new_cancel(self):
    for _ in range(3):
      self.reenable()
      self.assertGreater(self.step(), 0.2)
      self.step(stopping=True, speed=0.0)
      self.assertEqual(self.step(), 0.2)


if __name__ == '__main__':
  unittest.main()
