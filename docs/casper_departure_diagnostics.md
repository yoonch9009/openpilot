# Casper departure diagnostics

The diagnostic records are observations. The Casper mode-2 handoff trial was
withdrawn. The current restart uses the normal selfdrived
cancel/enable event path so LongControl and the planner see an actual OFF period.
The diagnostic producer itself does not supply control inputs.

The gasoline HYUNDAI_CASPER emits structured `logMessage` records with
`event=casper_departure_diagnostic`, `schema=1`. The producers are `controlsd`,
`scc`, `restart` and `planner`. Each producer is limited to 10 Hz below 2 m/s and for five seconds
after leaving that speed range. The existing logmessaged nonblocking IPC path
is used; control processes do not write diagnostic files. Capture is best
effort. Sequence gaps can indicate lost records; diagnostic_errors counts
contained snapshot/transport exceptions, not silent IPC drops.

## Questions addressed

1. **Casper restart:** `source=restart` records the owned OFF/ON phase, request
   and acknowledgment timestamps, input freshness, lead continuity and abort
   reason. SCC snapshots record pre-packing mode and requests. The old
   `handoff_active` field stays false for compatibility with prior recordings.
   DCEnable is not brake pressure.
   `res_held`, `res_release_ns`, `res_plan_ready` and
   `departure_inputs_mono_ns` distinguish held RES/+ or SET/- buttons from the fresh,
   normally engaged departure solve needed after release. Empty buttonEvents
   is not a release event. Already-engaged waiting RES/+ and SET/- operations preserve
   preparation; other buttons, pedals and inputs during owned OFF/ON abort it.
   The historical `res_*` field names cover both speed-setting buttons. Normal
   set-speed adjustment is unchanged; only automatic-restart preparation is
   preserved through these already-engaged waiting operations.
2. **Lead consistency:** Control snapshots contain plan hasLead, current radar
   lead and transmitted HUD lead fields, alongside service publication times,
   validity and alive flags. These are publication times, not sensor capture
   times; matching values does not establish that they describe the same object.
3. **Departure transitions:** Original carControl state, activation, override,
   request sign and lead-condition transitions are extracted at message cadence.
   A five-second motion follow-up distinguishes brief creep followed by another
   stop, intervention, inactive control, data gaps and a truncated recording.
   `motion_observed_for_5s` is an observation, not proof of normal or safe ACC.
4. **Acceleration/jerk relationship:** Compare plan target, actuator request,
   SCC pre-packing request, upper/lower jerk, comfort bands and aEgo estimate.
   `launch_accel_correction` records the bounded post-reenable tracking correction.
   Confirm final quantized transmitted values against original CAN frames;
   a pre-packing record is not an ECU acceptance acknowledgement.
   SCC `launch_jerk` contains the actual helper stage, pause/clear reason,
   absolute deadlines, input age and current health checks. The historical
   top-level `checks`/`blocked_by` are labelled
   `checks_scope=legacy_standstill_trial`; they do not describe launch-jerk
   eligibility. `brake_feedback`, `feedback_transitions_mono_ns` and
   `raw_motion_transition_mono_ns` retain received TCS/wheel transitions.
   TQI_SCC is a CAN signal, not measured engine torque.
5. **Detection versus planning:** Planner records contain the filter's actual
   input leads (after any fast overlay), per-role observation time, range
   evidence and release reason/time, followed by the published plan's exact
   timestamp, target, shouldStop and processing delay. These separate filter
   release from the subsequent MPC departure decision without changing the
   recorded raw radar or asserting a physical front-car departure time.

Only gasoline Casper with openpilot longitudinal control requests one automatic
CANCEL/ENABLE episode after a genuine following stop and confirmed departing
lead. Both carControl and controlsState must acknowledge OFF; there is no fixed
OFF dwell. A newer positive, nonstopping plan with the exact accepted restart
identity must be consumed by controlsd before requesting enable through
the normal no-entry checks. A real driver cancel, pedal, stale input, replaced
lead or error invalidates the owned episode and cannot be automatically undone.
Stock longitudinal and other cars are excluded. It remains a vehicle experiment.

For three seconds from first movement after stopped re-engagement, a moving Casper can recover at
most 0.4m/s² of acceleration lost to negative velocity error, with a 1.5m/s³
rise limit. This is allowed only below 3m/s, behind a departing lead, and when
estimated acceleration is below the positive plan target. It never exceeds the
plan target or existing actuator limits, and never boosts a stationary vehicle.

The classic camera-SCC gasoline Casper also has a separate post-motion upper
jerk allowance, bounded by 1.0m/s³ without reducing a higher original allowance.
It requires fresh feedback, a departing valid lead, positive target/request,
nonnegative planned jerk and estimated acceleration below the requested target.
A negative planned jerk temporarily suppresses this allowance and clears its
ramp value; it does not extend the two-second motion wait or the three-second
moving window. Genuine stopping, nonpositive requests and invalid inputs still
end the episode. Brake release is feedback eligibility, not a command from this
helper to release the brakes.

Stopped-lead conditioning still requires at least 10cm of opening range and
100ms of fresh confirmation. Multiple monotonic range increases with positive
lead motion can contribute confirmation before the final distance threshold;
a single range spike, constant range or rebound cannot use this earlier credit.

## Extraction

Use full rlogs on a compatible openpilot Python environment:

```sh
python tools/casper_stop_observer.py --input-root ROUTE_DIRECTORY --output report.json
```

ROUTE_DIRECTORY contains segment directories with `rlog.zst` files. Report
schema 2 adds `diagnostics`, `diagnostic_source_counts`, `control_changes` and
`motion_followups`. Existing rows, CAN flag changes and departure observations
remain available. Older logs can have an empty diagnostics array; this is not
evidence that the jerk trial was inactive. Records with source `self_test` and
validation_only=true are transport checks, not driving evidence.

CAN flag transitions retain their original cadence; 10 Hz snapshots can miss
short intermediate states. Use log_t (log service publication) and payload
mono_ns (producer snapshot) distinctly when aligning diagnostic events.

## Validation boundary

Restart records include departure_ready, should_stop, off_dwell_ns, the owned
request/acknowledgment times and the RES input/plan-readiness fields. Separate
filter confirmation from planner permission and OFF acknowledgment when
measuring latency. Snapshot rate is still 10Hz; these fields do not prove ECU
acceptance.

Tests compare every generated CAN byte with diagnostics enabled/disabled and
with a failing sink. They cover rate limiting, feedback/lead snapshots, parsing,
sub-sample control transitions, repeated stopping and incomplete motion windows.
Software and synthetic-log tests cannot verify ESC internals or vehicle restart
behavior. Actual driving evidence is still required.

The combined 2026-09-30 change passed 1,202 native tests and 318 subtests across
36 files. Coverage includes the recorded 18:33 RES sequence, held/released
buttons, stale or unconsumed plans, cancellation and pedal intervention,
negative-jerk pause/recovery/deadlines, unchanged CAN bytes with failed logging,
and Casper-only range-evidence reuse. The wider Hyundai firmware/fingerprint
data test has the same 16 failing entries on the unchanged b3da211 baseline;
these were reproduced separately and are outside this change. These software
results do not establish actual vehicle response improvements.

The SET/- follow-up uses the same waiting-button path as RES/+. Its targeted
native regression run passed 82 tests and 72 subtests, including held/released
SET inputs, stale departure plans, simultaneous cancellation, pedal priority,
late release during owned OFF and nonpositive/stopping plans. Vehicle behavior
after this follow-up has not yet been compared on a new drive.
