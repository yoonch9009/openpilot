# Casper departure diagnostics

These are observations, not additional control logic. The existing jerk-floor
trial remains unchanged. No acceleration, StopReq, lead-selection, hold, or
state-machine behavior is changed by this instrumentation.

The gasoline HYUNDAI_CASPER emits structured `logMessage` records with
`event=casper_departure_diagnostic`, `schema=1`. The producers are `controlsd`
and `scc`. Each producer is limited to 10 Hz below 2 m/s and for five seconds
after leaving that speed range. The existing logmessaged nonblocking IPC path
is used; control processes do not write diagnostic files. Capture is best
effort. Sequence gaps can indicate lost records; diagnostic_errors counts
contained snapshot/transport exceptions, not silent IPC drops.

## Questions addressed

1. **Jerk trial:** SCC snapshots contain the actual pre-packing upper limit
   before/after correction, floor-already-met flag and failed eligibility checks.
   TCS13 publication time allows checking feedback age. DCEnable is not brake
   pressure. The gate's long_active field includes the caller's PID-state gate.
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
   Confirm final quantized transmitted values against original CAN frames;
   a pre-packing record is not an ECU acceptance acknowledgement.

No changes to candidates 2, 3 or 4 are enabled by collecting these fields.

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
