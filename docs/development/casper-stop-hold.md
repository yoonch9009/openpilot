# Casper stop-hold investigation

## Maintenance baseline

This fork's default branch is `carrot-wip`, based on upstream
`1130b07462259e1f6c3950bbe4aeef3b8adc6c61`. The previous `carrot2-v6` branch
is preserved. Do not install `carrot2-v6` to obtain these changes.

Target: 2022 Hyundai Casper, selected as `Hyundai Casper 2023` in this fork,
with `HyundaiCameraSCC=1`, non-CAN-FD, and openpilot longitudinal control.
This identifies the investigated configuration, not a certification of compatibility.

## Requested behavior

앞차 추종으로 완전히 정지한 경우 앞차가 계속 정지해 있으면 제동을 유지한다.
앞차의 출발을 확인하면 추종을 재개한다. 운전자의 페달 및 CANCEL 조작과
시스템 비활성화 동작을 보존한다. 전자식 파킹브레이크 명령을 추가하는 작업이 아니다.

The requested behavior is to hold a completed following stop while the lead remains
stationary, then resume following after confirming its departure. Preserve driver
override, cancellation and fault handling. This is not an electronic parking brake feature.

## Evidence and limits

The current transition from `stopping` to `pid` requires only `shouldStop=false`,
`cruiseState.standstill=false` and no brake press when `startingState=false`.
Hyundai openpilot-longitudinal mode supplies `cruiseState.standstill=false`.
The planner derives `shouldStop` from two future planned speeds, not solely from
the actual vehicle speed. A single changed plan can therefore release stopping.
This path existed in the previously installed 2024 software as well.

A unit reproducer with a stationary vehicle and lead demonstrates the release.
This is a possible mechanism, not a confirmed diagnosis of the reported lurching.
Available historical qlogs showed driver brake cancellation near stops; no
unambiguous fully-stopped, stationary-lead, unintended restart was established.
Do not infer synchronized CAN commands from asynchronous, downsampled messages.

## Implemented controller boundary

A dedicated following-stop state is implemented at the longitudinal-controller boundary,
not a global override of CAN `StopReq` and not a change to the cruise minimum speed.

1. Enter only after a completed, active following stop with a valid stationary lead.
   Do not turn traffic stops or ordinary low-speed rolling into following holds.
2. Retain the state across a one-frame planner release or temporary lead loss.
   Lead loss must not by itself constitute departure evidence.
3. Release only on consistent departure evidence. Check lead identity/continuity,
   motion and increasing separation; do not use a noisy velocity sample alone.
4. Reset on driver override, cancellation, loss of active longitudinal control,
   or configuration changes. Do not force braking after control is disabled.
5. Preserve collision-related braking and existing fault/disengagement handling.
   A hold must never inhibit a stronger requested deceleration.

The controller enables this state only for `HYUNDAI_CASPER` with Hyundai openpilot
longitudinal control and `pcmCruise=false`. It requires D, reported standstill,
absolute ego speed at most 0.1m/s, an existing stopping state, and lead0 as the
planner source. Lead raw/filtered speed must be within 0.2m/s, with distance in
(0, 20]m. Entry tolerates a changed plan on that same cycle.

Initial departure criteria are raw and filtered lead speed at least 0.3m/s,
relative speed at least 0.2m/s, and at least 0.3m gap growth over at least 0.5s.
At least three distinct samples must show gap growth greater than 0.01m; a gap
reduction beyond 0.02m resets confirmation, so one late jump is insufficient.
The planner must select lead0 and no longer request stopping. Fresh distinct radar
samples are required, with no sample gap over 0.25s; plan age is also bounded to
0.25s. Duplicate messages cannot accumulate confirmation time. Track changes,
non-finite estimates, lost messages and implausible distance jumps invalidate
confirmation. Reacquisition requires another stationary, continuous lead for 0.5s;
a newly appearing moving object cannot release the hold. If this cannot be
established, the driver must intervene; there is no timeout that silently releases.

These are initial engineering thresholds, not values calibrated or proven safe
on this vehicle. They deliberately favor retaining a stop over an uncertain
restart, which can delay normal departure. Full-rate replay and controlled vehicle
validation remain outstanding. This commit is a source change, not deployment.

The hold retains existing stopping effort (at least the existing -0.5m/s² stopping
request) and preserves stronger planned braking within the existing actuator
limits. It does not add EPB commands, increase torque limits or change CAN safety.
Controller state and acceleration are published from the same cycle for Casper.
The helper consumes existing leads; no radar detection or selection code changes.

## Acceptance cases before activation

- Stationary lead and ego: a brief positive planned speed does not release hold.
- Genuine lead departure: normal following resumes without repeated re-latching.
- Lead loss, identity change, cut-in/out, and noisy distance/velocity: no false release.
- Rolling traffic and a lead stopping again: no unnecessary hold or blocked braking.
- Gas, brake, CANCEL, inactive controller and faults: driver and safety paths win.
- Non-Casper configurations and stock longitudinal control: unchanged behavior.
- Replay full-rate logs, then verify SCC stop request and actual service-brake response
  in a controlled vehicle test. Unit tests do not establish vehicle safety.

Do not ask a driver to delay braking to reproduce this issue on public roads.
The previous strict-xfail reproducer is now a passing controller regression.
Tests exercise the actual modified controller plus the pure state helper. Unit
results do not establish safe real-vehicle braking or correct perception.

## Verification for this change

The changed controller/helper were loaded from an isolated temporary directory
against the installed runtime dependencies; the running vehicle software was not
replaced. The new regression suite, existing state-transition tests and Hyundai
longitudinal-tuning tests completed with **93 passed**, no xfails. Controller caller
syntax compilation and the user-docs validator also passed.

Full-rate replay, closed-course vehicle validation and on-device deployment remain
outstanding. In particular, verify false-release rate, departure delay, brake
holding over time, and all override paths before treating this as vehicle-ready.
