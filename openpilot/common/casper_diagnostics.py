"""Best-effort, bounded Casper diagnostics. Never supplies control inputs."""
import time
import math


class CasperDiagnostics:
  def __init__(self, source, sink=None, clock=time.monotonic_ns):
    self.source = source
    self.sink = sink
    self.clock = clock
    self.last_ns = -1_000_000_000
    self.near_stop_ns = -10_000_000_000
    self.sequence = 0
    self.errors = 0

  def record(self, speed, build):
    # Includes five seconds after moving away. At most 10 Hz per producer;
    # the existing raw CAN log retains individual frame transitions.
    try:
      now = self.clock()
      if abs(speed) < 2.0:
        self.near_stop_ns = now
      if now - self.near_stop_ns > 5_000_000_000 or now - self.last_ns < 100_000_000:
        return
      self.last_ns = now
      data = build()
      self.sequence += 1
      payload = dict(event='casper_departure_diagnostic', schema=1, source=self.source,
                     mono_ns=now, sequence=self.sequence, diagnostic_errors=self.errors, **data)
      if self.sink is None:
        from openpilot.common.swaglog import cloudlog
        cloudlog.info(payload)
      else:
        self.sink(payload)
    except Exception:
      # A missing diagnostic field or logger failure must not stop card/controlsd.
      self.errors += 1


def control_snapshot(sm, CS, CC, publish_ns):
  radar = sm['radarState'].leadOne
  plan = sm['longitudinalPlan']
  return dict(
    car_control_mono_ns=int(publish_ns),
    services={name: dict(mono_ns=int(sm.logMonoTime[name]), valid=bool(sm.valid[name]),
                        alive=bool(sm.alive[name])) for name in ('radarState', 'longitudinalPlan', 'carState')},
    plan_has_lead=bool(plan.hasLead), should_stop=bool(plan.shouldStop), target_accel=float(plan.aTarget),
    lead_visibility_disagreement=bool(plan.hasLead) != bool(radar.status),
    radar_lead=dict(status=bool(radar.status), distance=float(radar.dRel), relative_speed=float(radar.vRel)),
    hud_lead=dict(visible=bool(CC.hudControl.leadVisible), distance=float(CC.hudControl.leadDistance),
                  relative_speed=float(CC.hudControl.leadRelSpeed)),
    enabled=bool(CC.enabled), long_active=bool(CC.longActive), override=bool(CC.cruiseControl.override),
    state=str(CC.actuators.longControlState), request_accel=float(CC.actuators.accel),
    request_jerk=float(CC.actuators.jerk), speed=float(CS.vEgo), estimated_accel=float(CS.aEgo), standstill=bool(CS.standstill),
    gas=bool(CS.gasPressed), brake=bool(CS.brakePressed), gear=str(CS.gearShifter),
    parking_brake=bool(CS.parkingBrake), brake_hold=bool(CS.brakeHoldActive))


def scc_snapshot(CS, enabled, long_active, stopping, accel, override, hud, jerk, upper, mode12, mode14, stop_req):
  from opendbc.car import structs
  checks = dict(enabled=bool(enabled), long_active=bool(long_active), not_stopping=not stopping,
                finite_requests=math.isfinite(accel) and math.isfinite(jerk.jerk_u),
                positive_request=accel > 0, no_override=not override,
                fresh_brake_control=bool(getattr(CS, 'casper_brake_control_active', False)),
                can_valid=bool(CS.out.canValid), standstill=bool(CS.out.standstill), low_speed=abs(CS.out.vEgo) < .1,
                drive=CS.out.gearShifter == structs.CarState.GearShifter.drive, cruise_available=bool(CS.out.cruiseState.available),
                no_acc_fault=not CS.out.accFaulted, no_brake=not CS.out.brakePressed,
                no_gas=not CS.out.gasPressed, no_parking_brake=not CS.out.parkingBrake,
                no_brake_hold=not CS.out.brakeHoldActive, no_soft_hold=CS.softHoldActive == 0,
                scc12_present=CS.scc12 is not None, lead_visible=bool(hud.leadVisible),
                positive_lead_distance=hud.leadDistance > 0, departing_lead=hud.leadRelSpeed > 0,
                active_modes=mode12 == 1 and mode14 == 1)
  return dict(checks=checks, blocked_by=[name for name, ok in checks.items() if not ok],
              jerk_before=float(jerk.jerk_u), jerk_after=float(upper), jerk_changed=upper != jerk.jerk_u,
              floor_already_met=jerk.jerk_u >= 1.0,
              jerk_lower=float(jerk.jerk_l), comfort_upper=float(jerk.cb_upper), comfort_lower=float(jerk.cb_lower),
              request_accel=float(accel), stop_req=int(stop_req), mode12=int(mode12), mode14=int(mode14),
              speed=float(CS.out.vEgo), state_stopping=bool(stopping),
              tcs13_mono_ns=int(getattr(CS, 'casper_tcs13_mono_ns', 0)),
              lead_visible=bool(hud.leadVisible), lead_distance=float(hud.leadDistance), lead_relative_speed=float(hud.leadRelSpeed))
