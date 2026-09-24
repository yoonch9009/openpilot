# Casper stop/creep investigation and ordinary-deceleration fix

## Result and scope

On 2026-09-20 the owner of the tested gasoline Casper (2022 model year) reported
that trial C completely resolved the reproduced standstill-holding problem.
The vehicle used HYUNDAI_CASPER, camera SCC and openpilot longitudinal control.
The queried ESC identified itself as part 58900-O6810, software `E14- 1918`
followed by byte 01, hardware 1.00. The part-number suffix starts with letter O.

The change is limited to the classic-CAN HYUNDAI_CASPER camera-SCC path.
During enabled stopping with ACCMode=1, a negative acceleration request,
no brake/gas input and no soft hold, SCC12 StopReq is cleared. Both acceleration
request fields retain their existing negative value. The checksum is recomputed.
SCC11, SCC14, jerk limits, comfort bands, safety hooks, tuning and saved defaults
are unchanged. Casper EV and other platforms keep their existing behavior.

This is a platform-specific command workaround, not a new planner latch, an EPB
implementation or a guarantee for every Casper ECU. Firmware matching is not used
as a runtime gate: manual vehicle selection may skip firmware queries. The observed
success belongs to the identified vehicle; other ECU versions remain unverified.

## Evidence and unsuccessful trials

The initial stop/creep interval retained stopping, shouldStop=true, StopReq=1,
ACCMode=1 and negative acceleration requests while the vehicle moved again.
Actual transmitted-frame echoes were decoded; the finding was not based solely
on queued sendcan messages. The experimental following-stop latch in 063259d did
not solve this and could delay legitimate departure when the planner source
changed from lead0 to cruise. It was withdrawn and is not restored by this fix.

Each later trial started from the same pre-trial command baseline rather than
combining changes:

| Trial | Change | Evidence/result |
| --- | --- | --- |
| A | aReqRaw=0, retain negative aReqValue | Owner reported no improvement; raw=0 was confirmed in transmitted-frame echoes. |
| B | ComfortBandUpper/Lower=0, restore both negative requests | Owner reported no improvement. Across a 7.70-second, 386-frame interval, StopReq=1, ACCMode=1, raw=value=-0.74 and both bands=0 persisted while speed rose again to about 1.49 km/h. |
| C | StopReq=0, restore original raw/value and comfort bands | Encoding and recorded-frame comparisons passed before installation. Owner explicitly reported complete success after vehicle testing. |

For trial B, driver pedal flags were false and the analyzed interval ends before
the recorded master-cylinder pressure began rising. ESC StandStill toggled about
1.3 seconds after each assertion, but that signal does not establish hydraulic
hold engagement. Master-cylinder pressure is not wheel-brake pressure. These logs
do not prove that StopReq is unsupported in every condition or identify the ECU's
internal release rule.

## Validation and limits

The pre-install trial C checks covered six regression tests and 355 recorded SCC12
frames. Baseline regeneration matched those recorded bytes exactly; the candidate
changed only StopReq and its checksum. Substituting the baseline failed the
change-detection test as expected. These are command-encoding checks, not an ECU
or hydraulic simulation.

The permanent regression suite checks StopReq clearing, retention of both negative
requests, checksum integrity, other-platform isolation, unchanged pedal/override/
soft-hold handling, immediate return to normal departure/cancel output, missing
messages and input-dictionary immutability. It uses the existing generic camera-SCC
path as the reference for unchanged fields.

Vehicle startup, source hash, unchanged settings, valid CAN, running processes and
Hyundai safety mode were checked after trial installation. The success drive's
logs have not yet been independently analyzed for this commit. The reported road
result is the owner's observation, not a claim of exhaustive verification.
Separate restart, brake/cancel intervention, slopes, extended holding and other
firmware versions remain to be validated. Continue driver supervision; do not delay
braking to reproduce a fault. If ordinary deceleration does not maintain a stop,
restore the previous file and investigate without adding periodic release pulses.

Private route logs, credentials, VINs and device identifiers are not published.
The original file and local trial artifacts are retained outside the repository.
AutoEngage=2 and other owner-selected defaults are unaffected.

## Departure filter adjustment (2026-09-21)

The shared StoppingLeadFilter departure distance is reduced from 0.15 to 0.10 m;
the continuous fresh-observation confirmation remains 0.10 s. Raw radar data,
lead selection, SCC12 StopReq handling and acceleration limits are unchanged.
The filter conditions planner input copies after the fast-radar overlay, so this
is not a radar detector/lead-selection or NAS radar-replay service change.

The deterministic 0.10 m/s lead crawl with 5 cm range quantization releases the
filter at 1.10 s instead of 1.60 s. This is a synthetic filter result, not a
closed-loop or road-measured departure improvement. Regression coverage includes
small range noise, isolated spikes, interrupted confirmation, both lead roles,
stale observations and positive-speed requirements. The 0.10 s confirmation
is explicitly tested at 0.099 and 0.100 s. Vehicle results remain pending.

## Deterministic departure transition trial (withdrawn)

Vehicle testing reported no departure improvement from the one-frame positive-acceleration StopReq pulse. The pulse, its Casper-specific controller state alignment, and its dedicated tests were removed. The observations below are retained only as investigation history and are not part of the active workaround.

Frame-level inspection found one StopReq=1 frame with positive acceleration in
the successful departure, followed by StopReq=0. Three failed departures had no
such frame. controlsd copied longControlState before LongControl.update but
acceleration afterwards; its 100 Hz mixed-state message was sometimes skipped
by 50 Hz SCC transmission. This establishes software timing dependence, not
proof of an ESC protocol requirement.

For gasoline Casper only, publish state after the longitudinal update. A
CasperDeparture instance tracks an eligible negative hold at |vEgo| <= 0.1 m/s
and consumes its departure event on the next SCC transmit tick with non-stopping
state, positive aTarget and positive acceleration. StopReq=1 is emitted for one
frame, then returns to zero. This is not a timed retry or an acceleration boost.
Controller continuity, active control, gear, CAN validity, driver overrides and
soft hold gate the event. The encoder rechecks the final mode and request after
Carrot overrides. No safety hooks, raw radar or lead selection change.

Unit tests exercise 20 control phases, ten repeated stop/departure cycles,
intervening non-transmit-tick interruptions, frame discontinuities, platform
isolation and real CAN packing/checksums. The actual longitudinal publication
block is executed with LongControl; the old code fails its departure and cancel
state-alignment checks. Focused suites: 342 passed. Recorded-input replay uses
four departure windows and both 50 Hz phases: exactly one event in all eight
cases, without changing recorded acceleration. It substitutes the same-tick
controlsState for the old published actuator state; it is an open-loop event
replay, not a complete controller/vehicle or hydraulic simulation.

The state alignment also removes the old one-tick lag for Casper jerk shaping
and cancellation. Therefore this candidate is not claimed to differ from every
old transmitted packet only in StopReq. Encoding tests separately verify that
the explicit departure flag changes only SCC12 StopReq and checksum at otherwise
identical inputs. Repeated vehicle operation and ECU acceptance remain pending.

## Legacy gasoline Casper AVH interlock trial (withdrawn 2026-09-22)

The AVH bypass has been withdrawn at the owner's request. Reinspection of the
stored successful departure and three failed departures found AVH_LAMP=0 and
brakeHoldActive=false throughout the four available segments. The bypass therefore
did not change the relevant control path in those records. Classic-CAN vehicles,
including Casper, again use AVH_LAMP==2 for brakeHoldActive. This restores the
original interlock; it does not enable openpilot soft hold.

The ordinary-deceleration StopReq=0 workaround remains. The ineffective one-frame
departure pulse remains withdrawn. No acceleration, jerk, planner, or safety-hook
change accompanies this restoration. The exact prolonged-stop release condition
is still unverified; do not label this revert a proven departure fix.

## Ordinary-driving log analysis

The owner cannot perform fixed-duration or fixed-order stop trials. Use natural
stop/departure events from existing rlogs instead. No prescribed waiting interval,
extra CAN transmission, or onroad diagnostic process is required. Driver takeover
must not be delayed to collect data. After driving, preserve the complete route,
including preceding segments, so a cropped segment is not mistaken for a short stop.

`tools/casper_stop_observer.py` reads rlogs offline with the full cereal schema and
Hyundai DBC. It separates received bus 0/2 messages from bus 128 transmit echoes,
records CAN flag transitions and approximately 10 Hz state/request snapshots, and
summarizes observed departure attempts. Only actually received, recent messages
are included; missing-message parser defaults are not treated as measured zeros.
Events are ordered by their monotonic timestamps for analysis, not as a simulation
of realtime process delivery. Missing segments and incomplete recordings are
explicitly marked. Driver intervention is distinguished from motion without pedal
input. The generic DBC is not an OEM specification of this Casper ECU.

On the device, after driving and while offroad:

```bash
cd /data/openpilot
/usr/local/venv/bin/python tools/casper_stop_observer.py \
  --input-root /data/media/0/realdata \
  --output /data/casper-stop-diagnostics/trial-analysis.json
```

The tool does not change Params, vehicle commands, raw logs, or startup services.
Reports contain route metadata and should remain local. Synthetic tests cover
missing data, stale plans, route boundaries, pedal intervention, repeated stops,
and incomplete recordings. Archived-log replay identifies the previously observed
one automatic departure and three driver interventions; it does not validate the
current revision on the vehicle or establish an ESC reset sequence.

## Near-standstill departure comfort-band trial (withdrawn 2026-09-22)

At the owner's request, the next vehicle trial changes SCC14 ComfortBandUpper
and ComfortBandLower to zero only for gasoline Casper near-standstill positive
acceleration. In archived departures, initial requests around +0.14 to +0.17 m/s2
were accompanied by bands around 0.82 to 0.94. Zero bands are also used by the
inspected FrogPilot classic-CAN encoder. However, similar nonzero bands occurred
in the successful departure, and failed requests later exceeded the band width.
The bands are therefore a tracking-shaping hypothesis, not a proved explanation
of the long-stop failure or a documented Casper ESC timeout.

The gate requires openpilot longitudinal control, available cruise, enabled
control, non-stopping state, positive final acceleration, both ACC modes equal
to 1, SCC12 present, valid CAN, drive gear, abs(vEgo) below 0.3 m/s, no driver
brake/gas, no parking brake and no soft hold. It also covers initial positive
engagement near standstill; it does not require a preceding timed stop. The
original bands return immediately outside these conditions. There is no timer,
pulse, retry, forced override or automatic cancellation/re-engagement sequence.
SCC12, acceleration requests, StopReq, modes, jerk limits and other platforms
are unchanged. The previously successful negative-deceleration stop workaround
and restored AVH interlock remain in place.

Earlier trial B zeroed bands while StopReq=1 during negative-acceleration stopping
and did not prevent creeping. This trial instead changes only positive departure
requests; it does not repeat the old stopping combination. Vehicle effectiveness
remains unverified, and crisper initial acceleration is a possible behavior change.
No fixed-duration driving test is required. Analyze naturally occurring events
with the offline observer, including the transmitted SCC14 bands, requested
acceleration, vehicle motion and driver intervention. Keep complete route logs.
Revert this departure-band change if vehicle behavior worsens; no inference of
ECU acceptance or hydraulic behavior follows from passing encoder tests alone.


The owner reported two failed departures after the comfort-band trial. The
transmitted bands were zero, yet positive requests up to +2.50 and +1.63 m/s2 did
not produce departure before driver intervention. This trial is now withdrawn;
SCC14 comfort bands use the original jerk helper values again. The offline
analysis tool remains available.

## Owner-requested continuous departure StopReq trial (withdrawn 2026-09-22)

At the owner's explicit request, after stopping further route-log investigation,
this trial keeps StopReq=1 on every eligible SCC12 transmit tick during a
near-standstill positive departure request. It is not the previously ineffective
single-frame pulse. There is no duration timer or pulse-count limit. Eligibility
requires gasoline Casper, openpilot longitudinal control, enabled and longActive,
cruise available, non-stopping state, positive final accel, SCC12 and SCC14
available with normal mode 1, valid CAN, D gear, abs(vEgo)<0.3 m/s, and no gas,
brake, parking brake or soft hold. Other conditions immediately use the original
encoder logic. The established negative-acceleration stopping workaround still
clears StopReq. ACC modes, raw/value acceleration, jerk limits, safety hooks and
other vehicle platforms are unchanged. This condition is stateless and can also
apply to an initial positive engagement near standstill; it is not latched into
normal driving.

The generic signal is a stop request, not a documented Casper release command.
Sustained assertion with positive acceleration is an explicitly requested vehicle
experiment; its effect on this ECU is not established by software tests. Do not
represent it as an OEM protocol or a verified fix. Tests check continuous output
across 250 consecutive transmit calls, repeated cycles, return to normal handling
on movement/intervention, unchanged other fields, and checksum integrity. They do
not simulate the ECU or prove standstill/departure behavior. No new driving-log
replay is part of this change; the owner explicitly asked to stop that analysis.


## Restore ordinary departure StopReq handling (2026-09-22)

The owner requested withdrawal of the continuous departure assertion after the
vehicle briefly moved and then nearly stopped again. Restore the pre-trial
Hyundai controller and encoder from 72bce11: negative-acceleration normal Casper
stopping retains StopReq=0, and normal non-stopping departure also uses StopReq=0.
Pedal/override/soft-hold and other platform behavior keep the original handling;
this is not an unconditional override of every StopReq field in every mode.
The ineffective comfort-band trial remains withdrawn and the AVH interlock stays
restored. Keep the offline diagnostic tool and owner-selected settings unchanged.

The new regression specifically holds a positive departure request across many
transmit calls, crosses 0.3 m/s in both directions, and returns to stopping. It
checks that the withdrawn trial cannot reassert StopReq during departure and
that the checksum and other bytes match the ordinary encoder. This restoration
is not a claim that the unresolved restart problem is fixed. No pedal-signal
spoofing, forced override, stronger acceleration, or new release mechanism is
included in this change.

## Bounded SCC mode handoff trial (2026-09-23, withdrawn)

The owner reported that the Casper-specific jerk-floor experiment transmitted
the intended SCC14 value but did not restore automatic departure. The failed
jerk-floor override is removed; ordinary HyundaiJerk output is preserved.

This new, unvalidated experiment tracks a real negative-request following
stop, requires at least one second stationary, then requires a moving lead and
a sustained small positive request for 250 ms. On gasoline HYUNDAI_CASPER
classic camera-SCC only, it sends SCC12 and SCC14 ACCMode=2 for at most
550 ms, then returns to their normal ACCMode=1. StopReq, SCC acceleration
request, jerk, comfort bands and other CAN messages are unchanged. The mode
handoff is one attempt per stop; it cannot re-arm after a brief creep. Driver
pedals, invalid/stale CAN, cruise cancellation, missing lead, unexpected motion,
parking brake, AutoHold and soft hold prevent or end the attempt.

The observed physical accelerator press also changed engine-side signals, so
this software mode transition is **not** equivalent to a pedal input and may
do nothing. Its timing and output are checked in offline regression tests, not
with a vehicle ECU model. A road result must include lead movement, the exact
sent SCC frames, TCS13 feedback, vehicle speed and driver intervention before
claiming success. The proven StopReq=0 negative-deceleration stop remains.

## Owned cancel/resume restart (2026-09-24)

The owner found that steering-wheel CANCEL followed by RES/SET permitted
departure. Three recorded cycles disabled CC.enabled/longActive for about
0.54–0.69 seconds, reset LongControl to zero/off, and sent SCC12 mode0 and
SCC14 mode4 before returning to mode1. This differs from the withdrawn mode2
encoder-only trial. The new implementation requests the existing cancel/enable
events in selfdrived and waits for fresh control OFF and planner acknowledgments.
It never clears a driver's cancel latch or bypasses no-entry events. Any real
button/pedal intervention, stale source, fault, or replaced/lost lead invalidates
the owned automatic episode; it never repeatedly toggles at the same stop.

The observed post-resume controller could request less than the positive plan
target because its reset velocity trajectory lagged the moving car. A bounded
three-second Casper-only correction recovers at most 0.2m/s² of this shortfall
once moving, ramps up at 0.5m/s³, requires measured acceleration below target,
and cannot exceed the target or normal actuator limits. Plan/radar/vehicle data
must be fresh. It is not a fixed launch acceleration or a standstill boost.

Software regressions and recorded input comparison do not establish repeatable
ECU acceptance. The owner's manual OFF/ON result is the basis for this experiment;
automatic OFF/ON still requires vehicle validation.
