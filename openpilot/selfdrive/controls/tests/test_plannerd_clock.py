import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.cereal import car, log
from openpilot.common.casper_diagnostics import CasperDiagnostics
from openpilot.common.runtime_diagnostics import RuntimeDiagnostics
from openpilot.selfdrive.carrot.radar import effective_radar_track_mode
from openpilot.selfdrive.carrot.radar_motion.timing import front_radar_distance_delay_s
from openpilot.selfdrive.controls.lib.longitudinal_fast_radar import RadarStateOverride
from openpilot.selfdrive.controls.lib.longitudinal_stopping_lead import StoppingLeadFilter


class EndOfInputs(Exception):
  pass


def run_planner_events(mocker, brand, configured_mode, radar_period_ms, *, stopping=False, gas_pressed=False,
                       fingerprint='', openpilot_longitudinal=True, flags=0, pcm_cruise=False, diagnostic_sink_failure=False,
                       departing_lead=False):
  model_times = list(range(0, 1000, 50))
  radar_times = list(range(20, 1000, radar_period_ms))
  events = iter(sorted(set(model_times + radar_times)))
  services = ('modelV2', 'radarState', 'liveTracks', 'carState')
  radar_state = log.RadarState.new_message()
  radar_state.leadOne.status = radar_state.leadOne.radar = True
  radar_state.leadOne.radarTrackId = 35
  radar_state.leadOne.dRel = 5.5
  radar_state.leadOne.vLead = radar_state.leadOne.vLeadK = 0.217
  radar_state.leadOne.vRel = 0.17

  class SubMaster:
    def __init__(self):
      self.now = 10.0
      self.seen = dict.fromkeys(services, False)
      self.updated = dict.fromkeys(services, False)
      self.valid = dict.fromkeys(services, True)
      self.alive = dict.fromkeys(services, True)
      self.recv_time = dict.fromkeys(services, 0.0)
      self.logMonoTime = dict.fromkeys(services, 0)

    def update(self):
      try:
        offset_ms = next(events)
      except StopIteration:
        raise EndOfInputs from None
      self.now = 10.0 + offset_ms / 1000.0
      if departing_lead:
        radar_state.leadOne.dRel = 5.6 if offset_ms >= 600 else 5.55 if offset_ms >= 500 else 5.5
      for service in services:
        updated = offset_ms in (radar_times if service == 'liveTracks' else model_times)
        self.updated[service] = updated
        if updated:
          self.seen[service] = True
          self.recv_time[service] = self.now
          self.logMonoTime[service] = 10_000_000_000 + offset_ms * 1_000_000

    def __getitem__(self, service):
      return {
        'selfdriveState': SimpleNamespace(experimentalMode=False),
        'carState': SimpleNamespace(vEgo=0.05 if stopping else 10.0, gasPressed=gas_pressed),
        'controlsState': SimpleNamespace(longControlState=(
          car.CarControl.Actuators.LongControlState.stopping if stopping else car.CarControl.Actuators.LongControlState.pid)),
        'radarState': radar_state,
        'modelV2': SimpleNamespace(frameId=round((self.now - 10.0) * 20)),
      }.get(service, SimpleNamespace())

    def all_checks(self, _services):
      return True

  sm = SubMaster()
  sub_master_factory = mocker.Mock(return_value=sm)
  cp = SimpleNamespace(brand=brand, carFingerprint=fingerprint, flags=flags, radarUnavailable=False, radarDelay=0.8,
                       openpilotLongitudinalControl=openpilot_longitudinal, pcmCruise=pcm_cruise)
  params = mocker.Mock()
  params.get_int.return_value = configured_mode
  fast_radar = mocker.Mock()
  fast_radar.lead_one_ready.return_value = True
  fast_radar.build.return_value = SimpleNamespace(
    radar_state=radar_state, lead_mask=1, lead_one_track_id=35, lead_one_reason='active',
  )
  planner = mocker.Mock()
  calls = []
  payloads, diagnostic_instances = [], []

  def publish_plan(*args, **kwargs):
    calls.append((sm.now, kwargs))
    send_ns = round(sm.now * 1e9) + 1000
    return SimpleNamespace(logMonoTime=send_ns, longitudinalPlan=SimpleNamespace(
      shouldStop=stopping, aTarget=0.25, aTargetBase=0.2, vTargetNow=0.1, jTargetNow=0.5,
      processingDelay=(send_ns - sm.logMonoTime['modelV2']) / 1e9,
      plannerExecutionTime=kwargs['planner_execution_time'], longitudinalPlanSource='lead0', fcw=False,
    ))

  def diagnostic_sink(payload):
    if diagnostic_sink_failure:
      raise OSError('diagnostic sink unavailable')
    payloads.append(payload)

  def make_diagnostics(source, clock):
    instance = CasperDiagnostics(source, sink=diagnostic_sink, clock=clock)
    diagnostic_instances.append(instance)
    return instance

  planner.publish.side_effect = publish_plan
  planner.input_lead_samples = []
  planner.update.side_effect = lambda view, carrot: planner.input_lead_samples.append((sm.now, float(view['radarState'].leadOne.vLead)))
  planner.diagnostic_payloads = payloads
  planner.diagnostic_instances = diagnostic_instances
  planner.diagnostic_factory = mocker.Mock(side_effect=make_diagnostics)
  namespace = {
    'time': SimpleNamespace(monotonic=lambda: sm.now, thread_time=lambda: sm.now, monotonic_ns=lambda: round(sm.now * 1e9)),
    'car': car,
    'CAR': CAR,
    'HyundaiFlags': HyundaiFlags,
    'CasperDiagnostics': planner.diagnostic_factory,
    'Params': lambda: params,
    'Priority': SimpleNamespace(CTRL_LOW=0),
    'config_realtime_process': mocker.Mock(),
    'cloudlog': mocker.Mock(),
    'RuntimeDiagnostics': RuntimeDiagnostics,
    'LongitudinalPlanner': mocker.Mock(return_value=planner),
    'LateralPlanner': mocker.Mock(),
    'FastRadarOverlay': mocker.Mock(return_value=fast_radar),
    'RadarStateOverride': RadarStateOverride,
    'StoppingLeadFilter': StoppingLeadFilter,
    'LaneDepartureWarning': mocker.Mock(),
    'CarrotPlanner': lambda: SimpleNamespace(mode='acc'),
    'effective_radar_track_mode': effective_radar_track_mode,
    'front_radar_distance_delay_s': front_radar_distance_delay_s,
    'messaging': SimpleNamespace(
      log_from_bytes=lambda value, schema: cp,
      PubMaster=mocker.Mock(),
      SubMaster=sub_master_factory,
      new_message=lambda service: SimpleNamespace(driverAssistance=SimpleNamespace()),
    ),
  }
  # Exercise the production event loop without loading hardware, messaging or
  # the native MPC solver. Only imports and the __main__ invocation are omitted.
  path = Path(__file__).parents[1] / 'plannerd.py'
  tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
  tree.body = [node for node in tree.body if isinstance(node, (ast.Assign, ast.FunctionDef))]
  exec(compile(tree, str(path), 'exec'), namespace)
  with pytest.raises(EndOfInputs):
    namespace['main']()
  assert planner.update.call_count == len(calls)
  return calls, sub_master_factory.call_args.kwargs['poll'], fast_radar, planner


@pytest.mark.parametrize('brand', ('volkswagen', 'toyota', 'honda', 'ford', 'subaru'))
@pytest.mark.parametrize('radar_period_ms', (20, 40, 50))
@pytest.mark.parametrize('configured_mode', (0, 1, 3))
def test_other_brands_plan_at_model_cadence_with_a_stable_radar_lead(mocker, brand, radar_period_ms, configured_mode):
  calls, poll, fast_radar, _ = run_planner_events(mocker, brand, configured_mode, radar_period_ms)

  assert [timestamp for timestamp, _ in calls] == pytest.approx([10.0 + n * 0.05 for n in range(20)])
  assert poll == 'modelV2'
  assert all(values['planning_trigger'] == 'modelV2' for _, values in calls)
  assert all(values['fast_lead_mask'] == 0 for _, values in calls)
  fast_radar.build.assert_not_called()


@pytest.mark.parametrize('configured_mode', (1, 2, 3))
def test_hyundai_keeps_current_radar_cadence_and_overlay(mocker, configured_mode):
  calls, poll, fast_radar, _ = run_planner_events(mocker, 'hyundai', configured_mode, 50)

  # The first model arrives before radar; subsequent plans use the radar clock.
  expected = [10.0, *(10.0 + ms / 1000.0 for ms in range(70, 1000, 50))]
  assert [timestamp for timestamp, _ in calls] == pytest.approx(expected)
  assert poll == ['modelV2', 'liveTracks']
  assert calls[0][1]['planning_trigger'] == 'modelV2'
  assert all(values['planning_trigger'] == 'liveTracks' for _, values in calls[1:])
  assert fast_radar.build.call_count == len(calls) - 1


@pytest.mark.parametrize('configured_mode', (-2, -1, 0))
def test_hyundai_without_radar_tracks_keeps_model_cadence(mocker, configured_mode):
  calls, poll, fast_radar, _ = run_planner_events(mocker, 'hyundai', configured_mode, 50)

  assert [timestamp for timestamp, _ in calls] == pytest.approx([10.0 + n * 0.05 for n in range(20)])
  assert poll == 'modelV2'
  fast_radar.build.assert_not_called()


@pytest.mark.parametrize('configured_mode', (0, 1))
def test_stopping_conditions_final_planner_input_on_model_and_fast_radar_clocks(mocker, configured_mode):
  _, _, fast_radar, planner = run_planner_events(mocker, 'hyundai', configured_mode, 50, stopping=True)
  inputs = [call.args[0]['radarState'].leadOne.vLead for call in planner.update.call_args_list]
  assert inputs[0] == pytest.approx(0.217)
  assert inputs[-5:] == [0.0] * 5
  # The fast source is still noisy: conditioning happened after that refresh.
  assert fast_radar.build.return_value.radar_state.leadOne.vLead == pytest.approx(0.217)


def test_driver_gas_override_bypasses_stopping_input_filter(mocker):
  _, _, _, planner = run_planner_events(mocker, 'hyundai', 1, 50, stopping=True, gas_pressed=True)
  assert all(call.args[0]['radarState'].leadOne.vLead == pytest.approx(0.217) for call in planner.update.call_args_list)


def test_gas_casper_diagnostics_link_filter_input_to_the_actual_published_plan(mocker):
  calls, _, _, planner = run_planner_events(
    mocker, 'hyundai', 1, 50, stopping=True, fingerprint=CAR.HYUNDAI_CASPER, flags=HyundaiFlags.CAMERA_SCC,
  )
  payloads = planner.diagnostic_payloads
  assert 1 < len(payloads) <= 10 < len(calls)
  assert all(b['mono_ns'] - a['mono_ns'] >= 100_000_000 for a, b in zip(payloads, payloads[1:]))
  final = payloads[-1]
  assert final['source'] == 'planner'
  assert final['diagnostic_errors'] == 0
  assert final['filter_input_leads']['leadOne']['speed'] == pytest.approx(0.217)
  assert final['filtered_leads']['leadOne']['speed'] == 0.0
  assert final['stopping_lead_filter']['held_mask'] == 1
  assert final['filter_input_mono_times'] == {
    'leadOne': final['services']['liveTracks']['mono_ns'], 'leadTwo': final['services']['radarState']['mono_ns'],
  }
  assert final['plan_should_stop'] is True
  assert final['plan_target_accel'] == 0.25
  assert final['plan_target_accel_base'] == 0.2
  assert final['plan_target_jerk'] == 0.5
  assert final['plan_target_speed'] == 0.1
  assert final['processing_delay_s'] == pytest.approx(
    (final['plan_publish_mono_ns'] - final['services']['modelV2']['mono_ns']) / 1e9,
  )


@pytest.mark.parametrize('fingerprint,openpilot_longitudinal,flags,pcm_cruise', (
  (CAR.HYUNDAI_CASPER_EV, True, HyundaiFlags.CAMERA_SCC, False),
  ('other_hyundai', True, HyundaiFlags.CAMERA_SCC, False),
  (CAR.HYUNDAI_CASPER, False, HyundaiFlags.CAMERA_SCC, False),
  (CAR.HYUNDAI_CASPER, True, 0, False),
  (CAR.HYUNDAI_CASPER, True, HyundaiFlags.CAMERA_SCC | HyundaiFlags.CANFD, False),
  (CAR.HYUNDAI_CASPER, True, HyundaiFlags.CAMERA_SCC, True),
))
def test_planner_departure_diagnostics_only_run_for_supported_casper(mocker, fingerprint, openpilot_longitudinal, flags, pcm_cruise):
  _, _, _, planner = run_planner_events(
    mocker, 'hyundai', 1, 50, stopping=True, fingerprint=fingerprint, openpilot_longitudinal=openpilot_longitudinal,
    flags=flags, pcm_cruise=pcm_cruise,
  )
  planner.diagnostic_factory.assert_not_called()
  assert planner.diagnostic_payloads == []


def test_casper_planning_continues_when_departure_diagnostic_logging_fails(mocker):
  calls, _, _, planner = run_planner_events(
    mocker, 'hyundai', 1, 50, stopping=True, fingerprint=CAR.HYUNDAI_CASPER, diagnostic_sink_failure=True,
    flags=HyundaiFlags.CAMERA_SCC,
  )
  assert len(calls) == 20
  assert planner.diagnostic_instances[0].errors > 0
  assert planner.diagnostic_payloads == []


@pytest.mark.parametrize('fingerprint,flags,pcm_cruise,expected_release', (
  (CAR.HYUNDAI_CASPER, HyundaiFlags.CAMERA_SCC, False, 10.62),
  (CAR.HYUNDAI_CASPER_EV, HyundaiFlags.CAMERA_SCC, False, 10.72),
  ('other_hyundai', HyundaiFlags.CAMERA_SCC, False, 10.72),
  (CAR.HYUNDAI_CASPER, 0, False, 10.72),
  (CAR.HYUNDAI_CASPER, HyundaiFlags.CAMERA_SCC | HyundaiFlags.CANFD, False, 10.72),
  (CAR.HYUNDAI_CASPER, HyundaiFlags.CAMERA_SCC, True, 10.72),
))
def test_departure_time_reuse_is_limited_to_supported_gas_casper(mocker, fingerprint, flags, pcm_cruise, expected_release):
  _, _, _, planner = run_planner_events(
    mocker, 'hyundai', 1, 50, stopping=True, fingerprint=fingerprint, flags=flags, pcm_cruise=pcm_cruise, departing_lead=True,
  )
  assert any(speed == 0.0 for _, speed in planner.input_lead_samples)
  first_departure = next(t for t, speed in planner.input_lead_samples if t >= 10.5 and speed > 0.0)
  assert first_departure == pytest.approx(expected_release)
