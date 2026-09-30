"""Require range evidence before predicting departure from a stopped lead.

Only planner input copies are conditioned. Raw radar measurements remain
available for establishing movement, and MPC still owns the stop/start output.
"""

from dataclasses import dataclass
import math
from typing import Any


STOPPED_EGO_SPEED = 0.10
STOPPED_LEAD_SPEED = 0.30
STABLE_RANGE_SPAN = 0.10
STABLE_TIME = 0.20
# Two 5 cm range bins: a single quantized range step is not departure.
DEPARTURE_DISTANCE = 0.10
DEPARTURE_TIME = 0.10
MAX_FRAME_GAP = 0.20


@dataclass
class _LeadEvidence:
  track_id: int
  since: float
  min_distance: float
  max_distance: float
  anchor_distance: float
  last_distance: float
  held: bool = False
  departure_since: float | None = None
  motion_since: float | None = None
  motion_samples: int = 0
  range_steps: int = 0
  departed: bool = False


class StoppingLeadFilter:
  def __init__(self, *, reuse_departure_evidence: bool = False):
    self.reuse_departure_evidence = reuse_departure_evidence
    self._leads: dict[str, _LeadEvidence] = {}
    self._last_time: float | None = None
    self._lead_times: dict[str, float] = {}
    self._last_release: dict[str, dict[str, Any]] = {}
    self.held_mask = 0
    self.fresh_mask = 0

  @staticmethod
  def _mono_ns(value: float | None) -> int:
    return round(value * 1e9) if value is not None else 0

  @classmethod
  def _evidence_snapshot(cls, evidence: _LeadEvidence) -> dict[str, Any]:
    return dict(
      track_id=evidence.track_id, held=evidence.held and not evidence.departed,
      stable_since_mono_ns=cls._mono_ns(evidence.since), anchor_distance=evidence.anchor_distance,
      measured_distance=evidence.last_distance, range_growth=evidence.last_distance - evidence.anchor_distance,
      first_motion_mono_ns=cls._mono_ns(evidence.motion_since), positive_motion_samples=evidence.motion_samples,
      range_growth_steps=evidence.range_steps, gap_evidence_mono_ns=cls._mono_ns(evidence.departure_since),
      departed=evidence.departed,
    )

  def snapshot(self) -> dict[str, Any]:
    # Retain each last release after the control state leaves stopping, so a
    # bounded diagnostic producer does not miss the actual measurement time.
    return dict(
      held_mask=self.held_mask, fresh_mask=self.fresh_mask,
      reuse_departure_evidence=self.reuse_departure_evidence,
      observation_mono_ns=self._mono_ns(self._last_time),
      leads={role: dict(
        observation_mono_ns=self._mono_ns(self._lead_times.get(role)),
        evidence=self._evidence_snapshot(self._leads[role]) if role in self._leads else None,
        last_release=dict(self._last_release[role]) if role in self._last_release else None,
      ) for role in ('leadOne', 'leadTwo')},
    )

  def _hold_lead(self, role: str, lead: Any, v_ego: float, now: float, fresh: bool) -> bool:
    valid = (
      lead.status and lead.radar and lead.radarTrackId >= 0
      and all(math.isfinite(v) for v in (lead.dRel, lead.vRel, lead.vLead, lead.aLeadK, lead.jLead))
      and lead.dRel > 0.2
    )
    if not valid:
      self._leads.pop(role, None)
      return False

    evidence = self._leads.get(role)
    if evidence is not None and evidence.track_id != lead.radarTrackId:
      self._leads.pop(role)
      evidence = None

    # Keep explicit approaching/decelerating observations in the original path.
    if lead.vLead < -STOPPED_LEAD_SPEED or lead.aLeadK < -0.5:
      self._leads.pop(role, None)
      return False

    quiet = v_ego <= STOPPED_EGO_SPEED and abs(lead.vLead) <= STOPPED_LEAD_SPEED
    if evidence is None:
      if not quiet or not fresh:
        return False
      evidence = _LeadEvidence(int(lead.radarTrackId), now, lead.dRel, lead.dRel, lead.dRel, lead.dRel)
      self._leads[role] = evidence

    if evidence.departed:
      return False
    if not fresh:
      return evidence.held

    if not evidence.held:
      evidence.min_distance = min(evidence.min_distance, lead.dRel)
      evidence.max_distance = max(evidence.max_distance, lead.dRel)
      if not quiet or evidence.max_distance - evidence.min_distance > STABLE_RANGE_SPAN + 1e-6:
        self._leads.pop(role)
        return False
      if now - evidence.since < STABLE_TIME - 1e-6:
        evidence.last_distance = lead.dRel
        return False
      evidence.held = True
      evidence.anchor_distance = lead.dRel
      evidence.last_distance = lead.dRel

    # A closing gap is still supplied to MPC; never freeze the obstacle range.
    # Re-establish evidence at the closer position instead of treating a rebound
    # from a range drop as departure from the old lead position.
    if lead.dRel < evidence.anchor_distance - STABLE_RANGE_SPAN - 1e-6:
      self._leads.pop(role)
      return False

    range_step = lead.dRel - evidence.last_distance
    evidence.last_distance = lead.dRel
    positive_speed = lead.vRel > 0.0 and lead.vLead > 0.0
    if not positive_speed or range_step < -1e-6 or lead.dRel <= evidence.anchor_distance + 1e-6:
      evidence.motion_since = None
      evidence.motion_samples = evidence.range_steps = 0
    elif range_step > 1e-6 or evidence.motion_since is not None:
      # Start with an observed range increase, not positive speed noise at a
      # fixed gap. Multiple increasing observations may establish the same
      # 100 ms of movement before the cumulative gap reaches two range bins.
      if evidence.motion_since is None:
        evidence.motion_since = now
      evidence.motion_samples += 1
      if range_step > 1e-6:
        evidence.range_steps += 1

    opening = (
      lead.dRel - evidence.anchor_distance >= DEPARTURE_DISTANCE - 1e-6
      and positive_speed and (not self.reuse_departure_evidence or range_step >= -1e-6)
    )
    if opening:
      if evidence.departure_since is None:
        evidence.departure_since = now
      gap_confirmed = now - evidence.departure_since >= DEPARTURE_TIME - 1e-6
      motion_confirmed = (
        self.reuse_departure_evidence
        and evidence.motion_since is not None and evidence.motion_samples >= 3 and evidence.range_steps >= 2
        and now - evidence.motion_since >= DEPARTURE_TIME - 1e-6
      )
      if gap_confirmed or motion_confirmed:
        evidence.departed = True
        self._last_release[role] = dict(
          self._evidence_snapshot(evidence), release_mono_ns=self._mono_ns(now),
          release_reason='rangeMotionConfirmed' if motion_confirmed else 'gapConfirmed',
        )
        return False
    else:
      evidence.departure_since = None
    return True

  def update(self, radar_state: Any, *, stopping: bool, v_ego: float, mono_time_ns: int, valid: bool = True,
             lead_mono_times: dict[str, int] | None = None) -> Any:
    self.held_mask = 0
    self.fresh_mask = 0
    now = mono_time_ns * 1e-9
    if not stopping or not valid or not math.isfinite(v_ego) or mono_time_ns <= 0:
      self._leads.clear()
      self._last_time = None
      self._lead_times.clear()
      return radar_state

    if self._last_time is not None and abs(now - self._last_time) > MAX_FRAME_GAP + 1e-6:
      self._leads.clear()
      self._last_time = None
      self._lead_times.clear()
    # A model-clock fallback can expose an older full-radard observation after
    # a fast update. It must neither confirm departure nor erase a held lead.
    self._last_time = now if self._last_time is None or now > self._last_time else self._last_time

    for role, mask in (("leadOne", 1), ("leadTwo", 2)):
      sample_ns = mono_time_ns if lead_mono_times is None else lead_mono_times[role]
      sample_time = sample_ns * 1e-9
      previous_time = self._lead_times.get(role)
      if previous_time is not None and abs(sample_time - previous_time) > MAX_FRAME_GAP + 1e-6:
        self._leads.pop(role, None)
        previous_time = None
      fresh = sample_ns > 0 and (previous_time is None or sample_time > previous_time)
      if fresh:
        self._lead_times[role] = sample_time
        self.fresh_mask |= mask
      if self._hold_lead(role, getattr(radar_state, role), abs(v_ego), sample_time, fresh):
        self.held_mask |= mask
    if not self.held_mask:
      return radar_state

    output = radar_state.as_builder() if hasattr(radar_state, "as_builder") else radar_state.as_reader().as_builder()
    for role, mask in (("leadOne", 1), ("leadTwo", 2)):
      if self.held_mask & mask:
        lead = getattr(output, role)
        lead.vLead = lead.vLeadK = 0.0
        lead.aLead = lead.aLeadK = 0.0
        lead.jLead = 0.0
    return output
