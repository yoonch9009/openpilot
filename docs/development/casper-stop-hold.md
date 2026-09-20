# Casper stop-hold investigation: experimental latch withdrawn

The experimental following-stop latch introduced in `063259d` has been withdrawn.
It did not fix the reported stop/creep cycling and could delay a legitimate restart.
The owner-selected defaults, including AutoEngage=2, are preserved.

## Field evidence

The tested vehicle logged HYUNDAI_CASPER with Hyundai openpilot longitudinal
control and pcmCruise=false. In a representative 7.26-second no-pedal interval,
all 364 SCC12 commands retained stopping, StopReq=1, ACCMode=1 and
both acceleration requests at -0.50m/s². Wheel-derived speed nevertheless rose
to approximately 2.02km/h and tracked lead distance decreased from 2.9m to 1.9m.
355 transmitted-frame echoes were independently decoded using the installed DBC
parser. These observations reject planner release as the cause of this interval;
they do not establish brake pressure, ECU acceptance of a hold mode, or hardware failure.
A different interval also showed low-speed movement with a sustained -1.24m/s² request.

The added latch also retained stopping after the planner requested departure and
its limiting source changed from lead0 to cruise. Requiring lead0 throughout release
confirmation was not valid for that observed transition. Unit tests that assumed
otherwise did not establish correct vehicle behavior.

## Withdrawal

Restore the controller and caller to the pre-latch implementation and remove the
helper. Replace the speculative xfail with controller regressions for departure,
continued planner-requested stopping, brake input and inactive control. This removes
the added delayed-departure behavior; it does not resolve the underlying stop/creep issue.

## Remaining vehicle-specific investigation

The camera-SCC path used by this vehicle copies the received SCC messages and sets
SCC12 aReqRaw and aReqValue to the same acceleration request. The other Hyundai
path in this source instead sets aReqRaw=0 during StopReq while retaining aReqValue.
This is a comparison candidate, not evidence that zeroing aReqRaw fixes Casper.
SCC14 comfort bands and jerk limits, SCC11 state, and ESC feedback must be considered
together. No unverified CAN-value experiment is included in this change.

The manufacturer's first-generation Casper price list dated 2023-04-27 specifies
SCC without Stop & Go:
https://m.casper.hyundai.com/wcontents/repn-car/catalog/AX01/AX_CASPER_price.pdf
This is relevant platform evidence, not a verified 2022 ECU specification or proof
that an aftermarket command can or cannot maintain a stop. Identify a validated
hold protocol for the actual controller before implementing another hold strategy.
Do not infer hydraulic pressure or command semantics from signal names alone.

A supported hold mode and successful controlled vehicle validation remain unproven.
Do not delay driver braking to recreate this issue on public roads.

## Withdrawal validation

56 controller and tuning tests passed, including the recorded departure scenario.
Controller and caller match the pre-latch source exactly. Caller syntax compilation,
user-documentation checks and diff checks passed. No new CAN values are introduced.
