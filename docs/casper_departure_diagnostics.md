# Casper departure diagnostics

The diagnostic records are observations. The Casper mode-2 handoff trial and
jerk-floor trial were withdrawn. The current restart uses the normal selfdrived
cancel/enable event path so LongControl and the planner see an actual OFF period.
The diagnostic producer itself does not supply control inputs.

The gasoline HYUNDAI_CASPER emits structured `logMessage` records with
`event=casper_departure_diagnostic`, `schema=1`. The producers are `controlsd`,
`scc` and `restart`. Each producer is limited to 10 Hz below 2 m/s and for five seconds
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

Only gasoline Casper with openpilot longitudinal control requests one automatic
CANCEL/ENABLE episode after a genuine following stop and confirmed departing
lead. Both carControl and controlsState must acknowledge OFF; after 550ms OFF,
a newer plan received by controlsd is required before requesting enable through
the normal no-entry checks. A real driver cancel, pedal, stale input, replaced
lead or error invalidates the owned episode and cannot be automatically undone.
Stock longitudinal and other cars are excluded. It remains a vehicle experiment.

For three seconds after stopped re-engagement, a moving Casper can recover at
most 0.2m/s² of acceleration lost to negative velocity error, with a 0.5m/s³
rise limit. This is allowed only below 3m/s, behind a departing lead, and when
estimated acceleration is below the positive plan target. It never exceeds the
plan target or existing actuator limits, and never boosts a stationary vehicle.

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

Tests compare every generated CAN byte with diagnostics enabled/disabled and
with a failing sink. They cover rate limiting, feedback/lead snapshots, parsing,
sub-sample control transitions, repeated stopping and incomplete motion windows.
Software and synthetic-log tests cannot verify ESC internals or vehicle restart
behavior. Actual driving evidence is still required.
