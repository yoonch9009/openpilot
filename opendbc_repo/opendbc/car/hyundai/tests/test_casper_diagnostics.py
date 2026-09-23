import copy
import json
from types import SimpleNamespace as NS

import pytest

from openpilot.common.casper_diagnostics import CasperDiagnostics, control_snapshot
from opendbc.car.hyundai.tests.test_casper_departure_jerk import state, request
from tools.casper_stop_observer import parse_diagnostic, motion_followups


def test_rate_bound_tail_and_failed_sink_do_not_escape():
  now = [0]
  records = []
  log = CasperDiagnostics('test', records.append, lambda: now[0])
  for i in range(1000):
    now[0] = i * 10_000_000
    log.record(0., lambda: dict(value=1))
  assert len(records) == 100
  now[0] += 6_000_000_000
  log.record(5., lambda: dict(value=2))
  assert len(records) == 100
  def fail(_):
    raise RuntimeError('sink unavailable')
  log.sink = fail
  log.record(0., lambda: {})
  assert log.errors == 1
  log.sink = records.append
  now[0] += 100_000_000
  log.record(0., lambda: {})
  assert records[-1]['diagnostic_errors'] == 1
  assert parse_diagnostic(json.dumps(dict(msg=records[-1]))) == records[-1]


@pytest.mark.parametrize('mode', ['eligible', 'stopping', 'pedal', 'invalid', 'high_jerk', 'lost_feedback', 'missing_scc12'])
def test_logging_cannot_change_any_can_byte_or_input(mode):
  cs = state()
  kwargs = {}
  if mode == 'stopping':
    kwargs.update(stopping=True, accel=-.5)
  elif mode == 'pedal':
    cs.out.gasPressed = True
  elif mode == 'invalid':
    cs.out.canValid = False
  elif mode == 'lost_feedback':
    cs.casper_brake_control_active = False
  elif mode == 'missing_scc12':
    cs.scc12 = None
  elif mode == 'high_jerk':
    kwargs['jerk'] = NS(carrot_cruise=0, cb_upper=.8, cb_lower=.7, jerk_u=2., jerk_l=1.)
  before = copy.deepcopy(cs)
  baseline = request(cs, **kwargs)
  records = []
  assert request(cs, diagnostics=CasperDiagnostics('scc', records.append), **kwargs) == baseline
  assert len(records) == 1
  assert cs == before
  r = records[0]
  if mode == 'eligible':
    assert not r['jerk_changed'] and not r['blocked_by']
    assert r['request_accel'] == .2 and r['stop_req'] == 0
  elif mode == 'high_jerk':
    assert not r['jerk_changed'] and r['floor_already_met']
  else:
    assert r['blocked_by'] and not r['jerk_changed']
  def broken(_):
    raise RuntimeError('diagnostic transport down')
  assert request(cs, diagnostics=CasperDiagnostics('scc', broken), **kwargs) == baseline


def test_build_failure_is_contained_and_malformed_logs_ignored():
  diag = CasperDiagnostics('test', lambda _: None)
  diag.record(0, lambda: 1 / 0)
  assert diag.errors == 1
  for text in ['bad', '[]', '{}', '{"msg":"hello"}']:
    assert parse_diagnostic(text) is None


def test_plan_and_radar_disagreement_is_recorded_without_rewriting_hud():
  class SM(dict):
    logMonoTime = {'radarState': 100, 'longitudinalPlan': 80, 'carState': 110}
    valid = {'radarState': True, 'longitudinalPlan': False, 'carState': True}
    alive = {'radarState': True, 'longitudinalPlan': True, 'carState': True}
  sm = SM(radarState=NS(leadOne=NS(status=False, dRel=0., vRel=0.)),
          longitudinalPlan=NS(hasLead=True, shouldStop=False, aTarget=.2))
  cs = state().out
  cs.aEgo = 0.0
  cc = NS(enabled=True, longActive=True, cruiseControl=NS(override=False),
          actuators=NS(longControlState='pid', accel=.2, jerk=.4),
          hudControl=NS(leadVisible=True, leadDistance=0., leadRelSpeed=0.))
  before = copy.deepcopy(cc)
  r = control_snapshot(sm, cs, cc, 120)
  assert r['lead_visibility_disagreement']
  assert r['services']['longitudinalPlan']['mono_ns'] == 80
  assert not r['services']['longitudinalPlan']['valid']
  assert cc == before


@pytest.mark.parametrize(('change', 'outcome'), [
  ({'v': 0.}, 'restopped_within_window'), ({'gas': True}, 'driver_intervened'),
  ({'active': False}, 'control_inactive'), ({'cs_age': 1.}, 'data_gap'),
])
def test_initial_motion_is_not_sustained_departure(change, outcome):
  row = dict(route='r', t=1.1, cs_age=0., v=.4, gas=False, brake=False, active=True)
  row.update(change)
  attempt = dict(route='r', request_t=0., end_t=1., outcome='motion_without_pedal_input')
  assert motion_followups([row], [attempt])[0]['outcome'] == outcome


def test_motion_followup_requires_full_window_and_rejects_gaps():
  attempt = dict(route='r', request_t=0., end_t=1., outcome='motion_without_pedal_input')
  rows = [dict(route='r', t=i / 10, cs_age=0., v=.4, gas=False, brake=False, active=True) for i in range(11, 61)]
  assert motion_followups(rows[:5], [attempt])[0]['outcome'] == 'recording_ended'
  assert motion_followups(rows, [attempt])[0]['outcome'] == 'motion_observed_for_5s'
  assert motion_followups(rows[-1:], [attempt])[0]['outcome'] == 'data_gap'


def test_extraction_retains_sub_sample_control_transitions(tmp_path):
  from openpilot.cereal import messaging
  import zstandard
  from tools.casper_stop_observer import extract
  messages = []
  for offset, accel in [(0, -.5), (20_000_000, .2), (40_000_000, -.5)]:
    e = messaging.new_message('carControl')
    e.logMonoTime = 1_000_000_000 + offset
    e.carControl.enabled = True
    e.carControl.longActive = True
    e.carControl.actuators.accel = accel
    e.carControl.actuators.longControlState = 'pid' if accel > 0 else 'stopping'
    messages.append(e.to_bytes())
  folder = tmp_path / 'synthetic--0'
  folder.mkdir()
  path = folder / 'rlog.zst'
  path.write_bytes(zstandard.ZstdCompressor().compress(b''.join(messages)))
  result = extract([path])
  assert not result['errors']
  assert [r['flags']['positive_request'] for r in result['control_changes']] == [False, True, False]
  assert [r['t'] for r in result['control_changes']] == pytest.approx([1., 1.02, 1.04])
