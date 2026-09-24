import math


class CasperReengageAccel:
  """Bound a moving-car tracking correction after a stopped CANCEL/re-enable.

  This helper does not release a brake or enable longitudinal control. The caller
  supplies the normal controller request and retains its final actuator limits.
  """

  def __init__(self):
    self.correction = 0.0
    self.remaining_s = 0.0
    self._was_active = False
    self._armed = False
    self._off_s = 0.0
    self._launch_started = False

  def _clear(self):
    self.correction = 0.0
    self.remaining_s = 0.0
    self._armed = False
    self._off_s = 0.0
    self._launch_started = False

  def update(self, active, stopping, speed, measured_accel, target_accel, target_speed,
             requested_accel, lead_valid, lead_distance, lead_relative_speed,
             driver_input, control_valid, dt=0.01):
    rising_active = active and not self._was_active
    self._was_active = active
    finite = all(math.isfinite(v) for v in (speed, measured_accel, target_accel, target_speed,
                                          requested_accel, lead_distance, lead_relative_speed, dt))
    if driver_input or not control_valid or not finite or not 0.0 < dt <= 0.05:
      self._clear()
      return requested_accel

    if not active:
      self.correction = 0.0
      self.remaining_s = 0.0
      self._launch_started = False
      self._off_s += dt
      if self._off_s > 2.0 + 1e-9 or abs(speed) >= 0.3:
        self._armed = False
      return requested_accel

    if rising_active:
      if self._armed and 0.0 < self._off_s <= 2.0 + 1e-9 and abs(speed) < 0.3 and lead_valid and lead_relative_speed > 0.2:
        self.remaining_s = 3.0
        self._launch_started = False
      self._armed = False
      self._off_s = 0.0

    if self.remaining_s > 0.0:
      if not lead_valid or lead_relative_speed <= 0.2 or lead_distance < 3.0 or (stopping and self._launch_started):
        self._clear()
      else:
        self.remaining_s = max(0.0, self.remaining_s - dt)
        self._launch_started |= not stopping

    # Keep the observed stop through a PID departure request while the car is
    # still stationary: the driver may CANCEL only after noticing no movement.
    if self.remaining_s <= 0.0:
      self.correction = 0.0
      if stopping and abs(speed) < 0.1:
        self._armed = True
      elif abs(speed) >= 0.3:
        self._armed = False
      return requested_accel

    eligible = (not stopping and 0.1 <= speed < 3.0 and target_speed < speed and
                target_accel > 0.0 and requested_accel >= 0.0 and
                target_accel - measured_accel > 0.05)
    if not eligible:
      self.correction = 0.0
      return requested_accel

    desired = min(0.2, max(target_accel - requested_accel, 0.0), max(target_accel - measured_accel, 0.0))
    self.correction = min(desired, self.correction + 0.5 * dt)
    return requested_accel + self.correction
