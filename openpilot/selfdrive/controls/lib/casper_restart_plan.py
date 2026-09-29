"""Validate a short, owned cruise restart before preserving planner continuity."""
import math


def fresh_input_snapshot_time(sm, now_ns):
  """Identify the inputs used by a solve, not its later completion time."""
  try:
    for service, max_age in (('selfdriveState', 100_000_000), ('carState', 100_000_000),
                             ('controlsState', 100_000_000), ('radarState', 300_000_000)):
      timestamp = sm.logMonoTime[service]
      if not sm.valid[service] or not sm.alive[service] or timestamp <= 0 or not 0 <= now_ns - timestamp <= max_age:
        return 0
    return int(sm.logMonoTime['selfdriveState'])
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    return 0


def accepted_restart_request(sm, now_ns, *, eligible, soft_hold_active, force_decel, prior_departure_ns=0):
  """Return the request identity, never a refreshed timestamp or an enable command."""
  if not eligible or soft_hold_active or force_decel:
    return 0
  try:
    request_ns = int(getattr(sm['selfdriveState'], 'casperRestartRequestMonoTime', 0))
    if request_ns <= 0 or not 0 <= now_ns - request_ns < 300_000_000:
      return 0
    # A marker alone cannot establish solver continuity after process startup.
    if not 0 < prior_departure_ns <= request_ns or request_ns - prior_departure_ns > 300_000_000:
      return 0
    if not fresh_input_snapshot_time(sm, now_ns):
      return 0
    cs = sm['carState']
    if (str(cs.gearShifter) != 'drive' or not cs.canValid or cs.canTimeout or cs.accFaulted
        or cs.gasPressed or cs.brakePressed or cs.parkingBrake or cs.brakeHoldActive or cs.softHoldActive):
      return 0
    # The marker cannot override a real button operation while its messages are in flight.
    if cs.buttonEvents:
      return 0
    lead = sm['radarState'].leadOne
    if (not lead.status or not math.isfinite(lead.dRel) or lead.dRel <= 0
        or not math.isfinite(lead.vRel) or lead.vRel < 0):
      return 0
    return request_ns
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
    return 0


def planner_reset_state(*, openpilot_longitudinal, long_control_off, enabled,
                        cruise_initialized, soft_hold_active, accepted_request):
  """Suppress only the owned OFF reset; ordinary reset causes retain priority."""
  reset = (long_control_off and not accepted_request) if openpilot_longitudinal else not enabled
  reset = bool(reset or not cruise_initialized or soft_hold_active)
  return reset, 0 if reset else accepted_request
