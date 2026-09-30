"""Bounded jerk allowance after an observed stopped longitudinal re-enable."""
import math


class CasperLaunchJerk:
  def __init__(self):
    self.last_time = None
    self.was_active = False
    self.stage = 'idle'
    self.reason = 'not_armed'
    self.last_transition_time = None
    self.last_pause_time = None
    self.last_pause_reason = None
    self.last_clear_time = None
    self.last_clear_reason = None
    self.clear(reason='not_armed')

  def _transition(self, stage, reason, now):
    if self.stage == stage and self.reason == reason:
      return False
    self.stage, self.reason = stage, reason
    self.last_transition_time = now if now is not None and math.isfinite(now) else None
    if stage == 'paused':
      self.last_pause_time = self.last_transition_time
      self.last_pause_reason = reason
    return True

  def clear(self, reason='cleared', now=None):
    now = self.last_time if now is None else now
    self.armed = False
    self.off_time = None
    self.wait_until = None
    self.end_time = None
    self.launched = False
    self.extra = 0.0
    if self._transition('idle' if now is None else 'ended', reason, now):
      self.last_clear_time = self.last_transition_time
      self.last_clear_reason = reason

  def diagnostic_snapshot(self):
    """Return episode state without changing it; all times are monotonic seconds."""
    return dict(stage=self.stage, reason=self.reason, armed=self.armed, off_time=self.off_time,
                wait_until=self.wait_until, end_time=self.end_time, launched=self.launched, extra=self.extra,
                last_transition_time=self.last_transition_time, last_pause_time=self.last_pause_time,
                last_pause_reason=self.last_pause_reason, last_clear_time=self.last_clear_time,
                last_clear_reason=self.last_clear_reason)

  def update(self, now, *, active, stopping, speed, healthy, feedback_fresh, brake_released,
             lead_valid, lead_distance, lead_relative_speed, target, request, measured, planned_jerk, original):
    dt = 0.02 if self.last_time is None else now - self.last_time
    self.last_time = now
    rising = active and not self.was_active
    self.was_active = active
    finite = all(math.isfinite(v) for v in (now, speed, lead_distance, lead_relative_speed,
                                          target, request, measured, planned_jerk, original))
    fault = ('nonfinite_input' if not finite else 'time_gap' if not 0 < dt <= 0.1 else
             'health_fault' if not healthy else 'feedback_stale' if not feedback_fresh else
             'lead_invalid' if not lead_valid else 'lead_too_close' if lead_distance < 2 else None)
    if fault is not None:
      self.clear(reason=fault, now=now)
      return original
    if not active:
      if self.wait_until is not None:
        self.clear(reason='control_inactive', now=now)
      self.extra = 0.0
      self.wait_until = self.end_time = None
      self.launched = False
      if self.off_time is None:
        self.off_time = now
      if now - self.off_time > 2 or abs(speed) >= 0.3:
        self.armed = False
      reason = ('off_timeout' if now - self.off_time > 2 else 'moving_while_inactive' if abs(speed) >= 0.3 else
                'waiting_reenable' if self.armed else 'not_armed')
      self._transition('off', reason, now)
      return original
    if rising:
      if self.armed and self.off_time is not None and now - self.off_time <= 2 and abs(speed) < 0.3 and lead_relative_speed > 0.2:
        self.wait_until = now + 2.0
      self.armed = False
      self.off_time = None
    if self.wait_until is not None:
      end_reason = ('lead_not_departing' if lead_relative_speed <= 0.2 else
                    'restopping' if stopping and self.launched else
                    'nonpositive_target' if not stopping and target <= 0 else
                    'nonpositive_request' if not stopping and request <= 0 else None)
      if end_reason is not None:
        self.clear(reason=end_reason, now=now)
      elif self.end_time is None:
        if now >= self.wait_until:
          self.clear(reason='wait_timeout', now=now)
        elif speed >= 0.1:
          self.end_time = now + 3.0
      elif now >= self.end_time:
        self.clear(reason='moving_timeout', now=now)
    if self.wait_until is None:
      if stopping and abs(speed) < 0.1:
        self.armed = True
        self._transition('armed', 'stopping', now)
      elif abs(speed) >= 0.3:
        self.armed = False
        if self.stage != 'ended':
          self._transition('idle', 'not_armed', now)
      elif self.stage == 'off':
        self._transition('idle', 'not_armed', now)
      return original
    self.launched |= not stopping
    eligible = (not stopping and self.end_time is not None and 0.1 <= speed < 10 / 3.6 and brake_released
                and target > 0 and request > 0 and min(target, request) - measured > 0.15 and planned_jerk >= 0)
    if not eligible:
      self.extra = 0.0
      if not stopping and planned_jerk < 0:
        # Both deadlines above keep running while the planned jerk suppresses the allowance.
        self._transition('paused', 'planned_jerk_negative', now)
      elif stopping:
        self._transition('wait', 'stopping', now)
      elif self.end_time is None:
        self._transition('wait', 'waiting_motion', now)
      elif not 0.1 <= speed < 10 / 3.6:
        self._transition('paused', 'outside_speed_window', now)
      elif not brake_released:
        self._transition('paused', 'brake_control_active', now)
      else:
        self._transition('paused', 'no_accel_deficit', now)
      return original
    self._transition('moving', 'assist', now)
    self.extra = min(max(1.0 - original, 0.0), self.extra + dt)
    return original + self.extra
