"""Confirm lead departure before releasing a completed following stop.

This consumes existing lead estimates; it does not change radar/lead selection.
Only fresh, distinct radar samples can accumulate departure evidence.
"""
import math


MAX_SAMPLE_AGE = 0.25
MAX_STOP_DISTANCE = 20.0
STOPPED_EGO_SPEED = 0.1
STOPPED_LEAD_SPEED = 0.2
DEPARTURE_SPEED = 0.3
DEPARTURE_REL_SPEED = 0.2
DEPARTURE_TIME = 0.5
DEPARTURE_DISTANCE = 0.3


class FollowingStop:
  def __init__(self):
    self.reset()

  def reset(self):
    self.holding = False
    self.track = None
    self.last_time = None
    self.last_distance = None
    self.departure_time = None
    self.departure_distance = None
    self.departure_growth_samples = 0
    self.uncertain = False
    self.reacquire_time = None

  def clear_departure(self):
    self.departure_time = None
    self.departure_distance = None
    self.departure_growth_samples = 0

  def update(self, *, active, driver_override, standstill, v_ego, stopping,
             should_stop, following, lead, timestamp_ns, fresh):
    if not active or driver_override:
      self.reset()
      return False

    valid = (fresh and timestamp_ns > 0 and lead.status and
             all(math.isfinite(x) for x in (lead.dRel, lead.vLead, lead.vLeadK, lead.vRel)) and
             lead.dRel > 0.0)
    if not valid:
      self.clear_departure()
      self.reacquire_time = None
      self.uncertain = self.holding
      return self.holding

    t = timestamp_ns * 1e-9
    if should_stop or not following:
      self.clear_departure()
    track = (bool(lead.radar), int(lead.radarTrackId))
    stationary = abs(lead.vLead) <= STOPPED_LEAD_SPEED and abs(lead.vLeadK) <= STOPPED_LEAD_SPEED

    if not self.holding:
      self.last_time = t
      if (standstill and abs(v_ego) <= STOPPED_EGO_SPEED and stopping and
          following and stationary and lead.dRel <= MAX_STOP_DISTANCE):
        self.holding = True
        self.track = track
        self.last_distance = lead.dRel
      return self.holding

    if t <= self.last_time:
      # Reusing a sample at control frequency must not count as confirmation.
      if t < self.last_time:
        self.clear_departure()
        self.uncertain = True
        self.reacquire_time = None
      return True

    elapsed = t - self.last_time
    gap_change = lead.dRel - self.last_distance
    continuous = (elapsed <= MAX_SAMPLE_AGE and track == self.track and
                  abs(lead.dRel - self.last_distance) <= 0.5 + 2.0 * abs(lead.vLead) * elapsed)
    self.last_time = t
    self.last_distance = lead.dRel
    if not continuous:
      self.uncertain = True
      self.clear_departure()
      self.reacquire_time = None

    if self.uncertain:
      # After lost/changed/discontinuous estimates, first observe another stable
      # stationary lead. A newly appearing moving object cannot release the hold.
      if continuous and following and stationary and lead.dRel <= MAX_STOP_DISTANCE:
        if self.reacquire_time is None:
          self.reacquire_time = t
        elif t - self.reacquire_time >= DEPARTURE_TIME:
          self.uncertain = False
          self.reacquire_time = None
      else:
        self.reacquire_time = None
        self.track = track
      return True

    departing = (following and not should_stop and lead.vLead >= DEPARTURE_SPEED and
                 lead.vLeadK >= DEPARTURE_SPEED and lead.vRel >= DEPARTURE_REL_SPEED and gap_change >= -0.02)
    if not departing:
      self.clear_departure()
    elif self.departure_time is None:
      self.departure_time = t
      self.departure_distance = lead.dRel
    elif lead.dRel < self.departure_distance:
      self.clear_departure()
    else:
      self.departure_growth_samples += int(gap_change > 0.01)
      if (t - self.departure_time >= DEPARTURE_TIME and
          lead.dRel - self.departure_distance >= DEPARTURE_DISTANCE and self.departure_growth_samples >= 3):
        self.reset()
        return False
    return True
