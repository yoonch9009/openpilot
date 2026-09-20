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

## Proposed implementation boundary

Implement a dedicated following-stop state at the longitudinal-controller boundary,
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

Departure duration, motion/distance thresholds, reacquisition behavior and the
maximum permitted uncertainty period remain **unselected**. Determine them using
actual stopped/departure traces and review the tradeoff between false release
and delayed departure. No production hold implementation is enabled in this commit.

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
The regression test is marked strict xfail until a reviewed implementation exists;
remove the marker when the required behavior is implemented and verified.
