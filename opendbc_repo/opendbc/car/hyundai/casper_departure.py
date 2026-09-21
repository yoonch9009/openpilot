"""One-shot classic-CAN Casper departure request, timed by CAN send frames.

The single-frame transition is an experimental reproduction of a successful
recorded departure, not a documented ESC protocol or a periodic release retry.
"""
import math


class CasperDeparture:
  def __init__(self):
    self.armed = False
    self.last_frame = None

  def update(self, *, frame, eligible, stopping, v_ego, accel, a_target):
    # Called on every controller tick, including non-transmit ticks, so an
    # intervening pedal/cancel or a scheduling discontinuity clears the event.
    if self.last_frame is None or frame != self.last_frame + 1:
      self.armed = False
    self.last_frame = frame
    if not eligible or not all(math.isfinite(v) for v in (v_ego, accel, a_target)) or abs(v_ego) > 0.1:
      self.armed = False
      return False
    if stopping:
      # Observe actual negative hold output; never arm from the old mixed
      # stopping-state/positive-acceleration transition.
      self.armed = accel < 0.0
      return False
    if frame % 2:
      return False
    if self.armed and accel > 0.0 and a_target > 0.0:
      self.armed = False
      return True
    return False
