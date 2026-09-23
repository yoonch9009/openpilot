"""One bounded SCC mode handoff trial after a genuine Casper follow stop."""
import math


class CasperSccHandoff:
  HOLD_MIN_NS = 1_000_000_000
  LEAD_CONFIRM_NS = 250_000_000
  HANDOFF_NS = 550_000_000

  def __init__(self):
    self.phase = 'idle'
    self.hold_since = 0
    self.confirm_since = 0
    self.handoff_since = 0
    self.last_now = 0

  def update(self, now_ns, ready, stopping, accel, speed, lead_departing, brake_control_active):
    if self.last_now and (now_ns <= self.last_now or now_ns - self.last_now > 100_000_000):
      self.phase = 'spent'
      return False
    self.last_now = now_ns
    # A brief roll must not re-arm the experiment; require actual travel before
    # considering another independent following stop.
    if math.isfinite(speed) and speed >= 1.0:
      self.__init__()
      return False

    if (now_ns <= 0 or not math.isfinite(speed) or not math.isfinite(accel) or not ready):
      if self.phase != 'idle':
        self.phase = 'spent'
      return False

    stationary = abs(speed) < .1
    if self.phase == 'idle':
      if stationary and stopping and accel < 0:
        self.phase = 'holding'
        self.hold_since = now_ns
      return False

    if self.phase == 'holding':
      if stopping and stationary:
        return False
      eligible = (stationary and not stopping and 0 < accel <= .5
                  and lead_departing and brake_control_active
                  and now_ns - self.hold_since >= self.HOLD_MIN_NS)
      if eligible:
        self.phase = 'confirming'
        self.confirm_since = now_ns
      else:
        self.phase = 'spent'
      return False

    departure_valid = (stationary and not stopping and 0 < accel <= .8
                       and lead_departing and brake_control_active)
    if self.phase == 'confirming':
      if not departure_valid:
        self.phase = 'spent'
      elif now_ns - self.confirm_since >= self.LEAD_CONFIRM_NS:
        self.phase = 'handoff'
        self.handoff_since = now_ns
        return True
      return False

    if self.phase == 'handoff':
      if (not departure_valid or abs(speed) >= .05
          or now_ns - self.handoff_since >= self.HANDOFF_NS):
        self.phase = 'spent'
        return False
      return True

    return False
