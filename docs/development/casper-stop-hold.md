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
