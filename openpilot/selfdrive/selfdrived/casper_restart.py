"""Own one automatic cancel/resume episode without undoing a driver's cancel."""


class CasperCruiseRestart:
  HOLD_NS = 1_000_000_000
  CONFIRM_NS = 100_000_000
  OFF_NS = 550_000_000
  TIMEOUT_NS = 1_500_000_000

  def __init__(self):
    self.phase = 'idle'
    self.hold_ns = 0
    self.confirm_ns = 0
    self.request_ns = 0
    self.ack_ns = 0
    self.last_ns = 0
    self.reason = ''

  def update(self, now_ns, *, enabled, healthy, stationary, stopping, departure,
             off_ack, plan_after_off, user_input, moving):
    action = None
    if moving:
      self.__init__()
      self.last_ns = now_ns
      return action
    gap = self.last_ns and (now_ns <= self.last_ns or now_ns - self.last_ns > 100_000_000)
    self.last_ns = now_ns
    if user_input or not healthy or gap:
      self.phase = 'spent'
      self.reason = 'driver_input' if user_input else 'unhealthy_or_gap'
      return action
    if self.phase == 'spent':
      return action
    if self.phase == 'off':
      if now_ns - self.request_ns > self.TIMEOUT_NS:
        self.phase, self.reason = 'spent', 'off_ack_timeout'
      elif not stationary:
        self.phase, self.reason = 'spent', 'unexpected_motion'
      elif not enabled and off_ack:
        if not self.ack_ns:
          self.ack_ns = now_ns
        if now_ns - self.ack_ns >= self.OFF_NS and plan_after_off:
          self.phase, self.reason = 'spent', 'resume_requested'
          action = 'on'
      elif self.ack_ns:
        self.phase, self.reason = 'spent', 'external_enable'
      return action
    if not enabled:
      self.phase, self.reason = 'spent', 'external_disable'
      return action
    if self.phase == 'idle':
      if stationary and stopping:
        self.phase = 'holding'
        self.hold_ns = now_ns
      return action
    if not stationary:
      self.phase, self.reason = 'spent', 'already_moving'
      return action
    if not departure:
      self.confirm_ns = 0
      return action
    if now_ns - self.hold_ns < self.HOLD_NS:
      return action
    if not self.confirm_ns:
      self.confirm_ns = now_ns
    elif now_ns - self.confirm_ns >= self.CONFIRM_NS:
      self.phase, self.reason = 'off', 'cancel_requested'
      self.request_ns = now_ns
      action = 'off'
    return action
