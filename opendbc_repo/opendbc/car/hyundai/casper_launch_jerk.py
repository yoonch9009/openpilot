"""Bounded jerk allowance after an observed stopped longitudinal re-enable."""
import math


class CasperLaunchJerk:
  def __init__(self):
    self.last_time = None
    self.was_active = False
    self.clear()

  def clear(self):
    self.armed = False
    self.off_time = None
    self.wait_until = None
    self.end_time = None
    self.launched = False
    self.extra = 0.0

  def update(self, now, *, active, stopping, speed, healthy, feedback_fresh, brake_released,
             lead_valid, lead_distance, lead_relative_speed, target, request, measured, planned_jerk, original):
    dt = 0.02 if self.last_time is None else now - self.last_time
    self.last_time = now
    rising = active and not self.was_active
    self.was_active = active
    finite = all(math.isfinite(v) for v in (now, speed, lead_distance, lead_relative_speed,
                                          target, request, measured, planned_jerk, original))
    if not finite or not 0 < dt <= 0.1 or not healthy or not feedback_fresh or not lead_valid or lead_distance < 2:
      self.clear()
      return original
    if not active:
      self.extra = 0.0
      self.wait_until = self.end_time = None
      self.launched = False
      if self.off_time is None:
        self.off_time = now
      if now - self.off_time > 2 or abs(speed) >= 0.3:
        self.armed = False
      return original
    if rising:
      if self.armed and self.off_time is not None and now - self.off_time <= 2 and abs(speed) < 0.3 and lead_relative_speed > 0.2:
        self.wait_until = now + 2.0
      self.armed = False
      self.off_time = None
    if self.wait_until is not None:
      if (lead_relative_speed <= 0.2 or (stopping and self.launched)
          or (not stopping and (target <= 0 or request <= 0 or planned_jerk < 0))):
        self.clear()
      elif self.end_time is None:
        if now >= self.wait_until:
          self.clear()
        elif speed >= 0.1:
          self.end_time = now + 3.0
      elif now >= self.end_time:
        self.clear()
    if self.wait_until is None:
      if stopping and abs(speed) < 0.1:
        self.armed = True
      elif abs(speed) >= 0.3:
        self.armed = False
      return original
    self.launched |= not stopping
    eligible = (not stopping and self.end_time is not None and 0.1 <= speed < 10 / 3.6 and brake_released
                and target > 0 and request > 0 and min(target, request) - measured > 0.15 and planned_jerk >= 0)
    if not eligible:
      self.extra = 0.0
      return original
    self.extra = min(max(1.0 - original, 0.0), self.extra + dt)
    return original + self.extra
