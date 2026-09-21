"""Virtual CAN-only departure checks; no ECU or hydraulic model."""
from types import SimpleNamespace as NS

import pytest

from opendbc.car.hyundai.casper_departure import CasperDeparture
from opendbc.car.hyundai.values import CAR
from opendbc.car.hyundai import hyundaican
from opendbc.car.hyundai.tests.test_casper_stopping import make_state, encode, decode


def tick(controller, frame, **overrides):
  args = dict(eligible=True, stopping=True, v_ego=0.0, accel=-0.53, a_target=-0.01)
  args.update(overrides)
  return controller.update(frame=frame, **args)


@pytest.mark.parametrize('transition_frame', range(8, 28))
def test_every_control_phase_sends_one_frame_and_preserves_acceleration(transition_frame):
  controller = CasperDeparture()
  asserted = []
  for frame in range(100):
    stopping = frame < transition_frame
    accel = -0.53 if stopping else 0.12
    pulse = tick(controller, frame, stopping=stopping, accel=accel, a_target=accel)
    if frame % 2:
      assert not pulse
      continue
    messages = encode(hyundaican, make_state(), stopping=stopping, accel=accel,
                      idx=frame // 2, casper_departure=pulse)
    baseline = encode(hyundaican, make_state(), stopping=stopping, accel=accel, idx=frame // 2)
    decoded = decode(messages)
    assert decoded['aReqRaw'] == pytest.approx(accel)
    assert decoded['aReqValue'] == pytest.approx(accel)
    assert decoded['ACCMode'] == 1
    if decoded['StopReq']:
      asserted.append(frame)
      assert not stopping
      assert {k: v for k, v in decoded.items() if k not in ('StopReq', 'CR_VSM_ChkSum')} == {
        k: v for k, v in decode(baseline).items() if k not in ('StopReq', 'CR_VSM_ChkSum')}
    else:
      assert messages == baseline
    payload = next(m[1] for m in messages if m[0] == 1057)
    assert sum((v >> 4) + (v & 15) for v in payload) % 16 == 0
  assert asserted == [transition_frame + transition_frame % 2]


def test_repeated_stops_rearm_but_never_retry_during_one_departure():
  controller = CasperDeparture()
  pulses = []
  for frame in range(1000):
    stopping = frame % 100 < 20
    if tick(controller, frame, stopping=stopping, accel=-0.5 if stopping else .2,
            a_target=-.01 if stopping else .2):
      pulses.append(frame)
  assert pulses == list(range(20, 1000, 100))


@pytest.mark.parametrize('interruption', [dict(eligible=False), dict(v_ego=.101),
  dict(v_ego=float('nan')), dict(accel=float('nan')), dict(a_target=float('inf'))])
def test_intervening_non_send_tick_clears_pending_departure(interruption):
  controller = CasperDeparture()
  for frame in range(5):
    assert not tick(controller, frame)
  assert not tick(controller, 5, **interruption)
  for frame in range(6, 20):
    assert not tick(controller, frame, stopping=False, accel=.2, a_target=.2)


@pytest.mark.parametrize('frame', [4, 2, 10])
def test_duplicate_backward_and_missing_ticks_discard_previous_stop(frame):
  controller = CasperDeparture()
  for i in range(5):
    tick(controller, i)
  assert not tick(controller, frame, stopping=False, accel=.2, a_target=.2)


def test_starting_without_observed_hold_never_pulses():
  controller = CasperDeparture()
  for frame in range(20):
    assert not tick(controller, frame, stopping=False, accel=.2, a_target=.2)


def test_waits_for_positive_plan_and_restop_cancels_pending_departure():
  controller = CasperDeparture()
  for frame in range(4):
    tick(controller, frame)
  assert not tick(controller, 4, stopping=False, accel=.2, a_target=-.01)
  assert not tick(controller, 5, stopping=False, accel=.2, a_target=.2)
  assert not tick(controller, 6)  # lead stopped again before the send tick
  assert not tick(controller, 7)
  assert tick(controller, 8, stopping=False, accel=.2, a_target=.2)
  assert not tick(controller, 9, stopping=False, accel=.2, a_target=.2)
  assert not tick(controller, 10, stopping=False, accel=.2, a_target=.2)


@pytest.mark.parametrize('platform', [CAR.HYUNDAI_CASPER_EV, CAR.HYUNDAI_SONATA])
def test_other_platforms_ignore_departure_flag(platform):
  assert encode(hyundaican, make_state(platform), stopping=False, accel=.2, casper_departure=True) == \
         encode(hyundaican, make_state(platform), stopping=False, accel=.2)


@pytest.mark.parametrize('field', ['brakePressed', 'gasPressed', 'brakeHoldActive'])
def test_encoder_rechecks_pedal_and_hold_before_sending(field):
  cs = make_state()
  setattr(cs.out, field, True)
  assert encode(hyundaican, cs, stopping=False, accel=.2, casper_departure=True) == \
         encode(hyundaican, cs, stopping=False, accel=.2)


@pytest.mark.parametrize('args', [dict(enabled=False), dict(long_override=True), dict(accel=0), dict(accel=-.2)])
def test_encoder_rechecks_active_mode_and_final_acceleration(args):
  options = dict(stopping=False, accel=.2)
  options.update(args)
  assert encode(hyundaican, make_state(), **options, casper_departure=True) == \
         encode(hyundaican, make_state(), **options)


@pytest.mark.parametrize('hold', [1, 2])
def test_soft_hold_cannot_use_departure_event(hold):
  cs = make_state()
  cs.softHoldActive = hold
  assert encode(hyundaican, cs, stopping=False, accel=.2, casper_departure=True) == \
         encode(hyundaican, cs, stopping=False, accel=.2)


def test_carcontroller_call_site_rejects_each_ineligible_input():
  import ast
  from pathlib import Path
  from opendbc.car import structs

  path = Path(__file__).parents[1] / 'carcontroller.py'
  tree = ast.parse(path.read_text())
  node = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign)
              and any(isinstance(t, ast.Name) and t.id == 'casper_departure' for t in n.targets))
  code = compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec')

  def run(change=None):
    cs = make_state()
    cs.out.vEgo = 0.0
    cs.out.canValid = True
    cs.out.canTimeout = False
    cs.out.gearShifter = structs.CarState.GearShifter.drive
    cc = NS(enabled=True, longActive=True, cruiseControl=NS(override=False))
    cp = NS(carFingerprint=CAR.HYUNDAI_CASPER, openpilotLongitudinalControl=True)
    controller = NS(CP=cp, casper_departure=CasperDeparture(), hyundai_jerk=NS(carrot_cruise=0), frame=0)
    ns = dict(self=controller, CS=cs, CC=cc, CAR=CAR, structs=structs,
              camera_scc=True, stopping=True, accel=-.5, actuators=NS(aTarget=-.01))
    exec(code, ns)
    controller.frame = 1
    if change:
      target, field, value = change
      if target == 'namespace':
        ns[field] = value
      else:
        setattr({'out':cs.out, 'cs':cs, 'cc':cc, 'cruise':cc.cruiseControl,
                 'cp':cp, 'jerk':controller.hyundai_jerk}[target], field, value)
    exec(code, ns)  # also exercise the non-send tick interlock
    controller.frame = 2
    ns.update(stopping=False, accel=.2, actuators=NS(aTarget=.2))
    exec(code, ns)
    return ns['casper_departure']

  assert run()
  for change in [('out', 'brakePressed', True), ('out', 'gasPressed', True),
                 ('out', 'brakeHoldActive', True), ('out', 'canValid', False), ('out', 'canTimeout', True),
                 ('out', 'gearShifter', structs.CarState.GearShifter.park),
                 ('out', 'gearShifter', structs.CarState.GearShifter.reverse),
                 ('out', 'gearShifter', structs.CarState.GearShifter.neutral),
                 ('out', 'gearShifter', structs.CarState.GearShifter.unknown),
                 ('cs', 'softHoldActive', 1), ('cs', 'scc12', None),
                 ('cc', 'enabled', False), ('cc', 'longActive', False), ('cruise', 'override', True),
                 ('cp', 'carFingerprint', CAR.HYUNDAI_CASPER_EV), ('cp', 'openpilotLongitudinalControl', False),
                 ('jerk', 'carrot_cruise', 1), ('namespace', 'camera_scc', False)]:
    assert not run(change), change
