import pytest

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.longitudinal_stopping_lead import StoppingLeadFilter


def radar_state(*, distance=5.5, v_lead=0.05, v_rel=0.0, a_lead=0.0, j_lead=0.0, track_id=42, role='leadOne'):
  state = log.RadarState.new_message()
  lead = getattr(state, role)
  lead.status = lead.radar = True
  lead.radarTrackId = track_id
  lead.dRel = distance
  lead.vRel = v_rel
  lead.vLead = lead.vLeadK = v_lead
  lead.aLead = lead.aLeadK = a_lead
  lead.jLead = j_lead
  lead.aLeadTau = 1.5
  return state


def update(guard, state=None, *, t=1.0, stopping=True, v_ego=0.05, valid=True, lead_mono_times=None):
  return guard.update(
    radar_state() if state is None else state,
    stopping=stopping, v_ego=v_ego, mono_time_ns=round(t * 1e9), valid=valid,
    lead_mono_times=lead_mono_times,
  )


def establish_stop(guard, **kwargs):
  for index in range(5):
    output = update(guard, radar_state(**kwargs), t=1.0 + index * 0.05)
  return output


def test_stopped_lead_requires_a_history_of_distinct_stable_observations():
  guard = StoppingLeadFilter()
  for index in range(4):
    source = radar_state()
    assert update(guard, source, t=1.0 + index * 0.05) is source
  assert update(guard, t=1.2).leadOne.vLead == 0.0


def test_small_range_step_and_rising_speed_do_not_predict_departure():
  guard = StoppingLeadFilter()
  establish_stop(guard)
  # Measured speed/range pairs from a stop where ego stayed around 0.04 m/s.
  samples = ((5.5, 0.05, 0.093946), (5.5, 0.08, 0.12146), (5.5, 0.09, 0.12999),
             (5.55, 0.11, 0.149906), (5.55, 0.14, 0.18026), (5.55, 0.16, 0.202281), (5.55, 0.17, 0.217369))
  for index, (distance, v_rel, v_lead) in enumerate(samples):
    source = radar_state(distance=distance, v_rel=v_rel, v_lead=v_lead, a_lead=0.25, j_lead=0.4)
    output = update(guard, source.as_reader(), t=1.25 + index * 0.05, v_ego=0.043)
    lead = output.leadOne
    assert (lead.vLead, lead.vLeadK, lead.aLead, lead.aLeadK, lead.jLead) == (0.0,) * 5
    assert lead.dRel == pytest.approx(distance)
    assert lead.vRel == pytest.approx(v_rel)
    assert source.leadOne.vLead == pytest.approx(v_lead)
    assert source.leadOne.aLeadK == pytest.approx(0.25)
    assert guard.held_mask == 1


def test_fast_real_departure_restores_measured_kinematics_after_range_confirmation():
  guard = StoppingLeadFilter()
  establish_stop(guard)
  for t, distance in ((1.25, 5.6), (1.3, 5.7)):
    assert update(guard, radar_state(distance=distance, v_lead=0.8, v_rel=0.75), t=t).leadOne.vLead == 0.0
  source = radar_state(distance=5.8, v_lead=1.0, v_rel=0.95, a_lead=1.0, j_lead=0.5)
  assert update(guard, source, t=1.35) is source
  assert guard.held_mask == 0


@pytest.mark.parametrize('reuse_evidence,expected_release', ((False, 1.1), (True, 1.0)))
def test_slow_departure_accumulates_range_from_fixed_stop_position(reuse_evidence, expected_release):
  guard = StoppingLeadFilter(reuse_departure_evidence=reuse_evidence)
  establish_stop(guard)
  first_release = None
  for index in range(1, 41):
    elapsed = index * 0.05
    # 0.10 m/s crawl, quantized at 5 cm. No individual frame grows by 10 cm.
    distance = 5.5 + int((elapsed * 0.1 + 1e-9) / 0.05) * 0.05
    output = update(guard, radar_state(distance=distance, v_lead=0.1, v_rel=0.1), t=1.2 + elapsed, v_ego=0.0)
    if output.leadOne.vLead > 0.0 and first_release is None:
      first_release = elapsed
  # Casper can reuse the first bin's 500 ms of movement at the second bin.
  assert first_release == pytest.approx(expected_release)


def test_one_large_range_spike_does_not_confirm_departure():
  guard = StoppingLeadFilter()
  establish_stop(guard)
  for t, distance in ((1.25, 5.75), (1.3, 5.5), (1.35, 5.55)):
    assert update(guard, radar_state(distance=distance, v_lead=0.3, v_rel=0.25), t=t).leadOne.vLead == 0.0


def test_repeated_or_older_fallback_measurement_does_not_accumulate_departure_time():
  guard = StoppingLeadFilter()
  establish_stop(guard)
  for t in (1.25, 1.25, 1.20, 1.25, 1.30):
    assert update(guard, radar_state(distance=5.75, v_lead=0.5, v_rel=0.45), t=t).leadOne.vLead == 0.0
  assert update(guard, radar_state(distance=5.8, v_lead=0.5, v_rel=0.45), t=1.35).leadOne.vLead > 0.0


@pytest.mark.parametrize('kwargs', ({'stopping': False}, {'valid': False}))
def test_disabled_or_invalid_input_clears_stop_history(kwargs):
  guard = StoppingLeadFilter()
  establish_stop(guard)
  source = radar_state(v_lead=0.2)
  assert update(guard, source, t=1.25, **kwargs) is source
  assert update(guard, source, t=1.3) is source


@pytest.mark.parametrize('time', (0.5, 2.0))
def test_timestamp_discontinuity_requires_new_stop_evidence(time):
  guard = StoppingLeadFilter()
  establish_stop(guard)
  source = radar_state(v_lead=0.2)
  assert update(guard, source, t=time) is source


@pytest.mark.parametrize('role', ('leadOne', 'leadTwo'))
def test_stationary_classification_belongs_to_the_same_track_and_role(role):
  guard = StoppingLeadFilter()
  output = establish_stop(guard, role=role)
  assert getattr(output, role).vLead == 0.0
  assert guard.held_mask == (1 if role == 'leadOne' else 2)
  replacement = radar_state(track_id=43, role=role, v_lead=0.2)
  assert update(guard, replacement, t=1.25) is replacement


@pytest.mark.parametrize('changes', ({'v_lead': -0.5}, {'a_lead': -2.0}, {'distance': 5.2}))
def test_approaching_or_decelerating_lead_is_not_overwritten(changes):
  guard = StoppingLeadFilter()
  establish_stop(guard)
  source = radar_state(**changes)
  assert update(guard, source, t=1.25) is source


@pytest.mark.parametrize('changes', ({'v_lead': 2.0}, {'v_lead': -2.0}, {'distance': float('nan')}))
def test_nonstationary_or_nonfinite_lead_never_acquires_stop_hold(changes):
  guard = StoppingLeadFilter()
  for index in range(10):
    source = radar_state(**changes)
    assert update(guard, source, t=1.0 + index * 0.05) is source


def test_matching_speed_and_constant_range_while_driving_is_not_stationary():
  guard = StoppingLeadFilter()
  for index in range(10):
    source = radar_state(v_lead=5.0, v_rel=0.0)
    assert update(guard, source, t=1.0 + index * 0.05, v_ego=5.0) is source


def test_missing_or_vision_only_track_does_not_inherit_stopped_radar_history():
  guard = StoppingLeadFilter()
  establish_stop(guard)
  source = radar_state()
  source.leadOne.radar = False
  assert update(guard, source, t=1.25) is source
  source.leadOne.radar = True
  assert update(guard, source, t=1.3) is source
  source.leadOne.status = False
  assert update(guard, source, t=1.35) is source


@pytest.mark.parametrize('role', ('leadOne', 'leadTwo'))
def test_ten_centimeter_departure_still_requires_full_confirmation(role):
  guard = StoppingLeadFilter()
  establish_stop(guard, role=role)
  for t in (1.25, 1.30, 1.349):
    output = update(guard, radar_state(distance=5.6, v_lead=0.1, v_rel=0.1, role=role), t=t)
    assert getattr(output, role).vLead == 0.0
  source = radar_state(distance=5.6, v_lead=0.1, v_rel=0.1, role=role)
  assert update(guard, source, t=1.35) is source


def test_range_rebound_resets_departure_confirmation():
  guard = StoppingLeadFilter()
  establish_stop(guard)
  for t, distance in ((1.25, 5.6), (1.3, 5.55), (1.35, 5.6), (1.4, 5.6)):
    assert update(guard, radar_state(distance=distance, v_lead=0.1, v_rel=0.1), t=t).leadOne.vLead == 0.0
  source = radar_state(distance=5.6, v_lead=0.1, v_rel=0.1)
  assert update(guard, source, t=1.45) is source


@pytest.mark.parametrize('v_lead,v_rel', ((0.0, 0.1), (0.1, 0.0), (0.1, -0.1)))
def test_ten_centimeter_range_change_alone_does_not_release(v_lead, v_rel):
  guard = StoppingLeadFilter()
  establish_stop(guard)
  for index in range(10):
    output = update(guard, radar_state(distance=5.6, v_lead=v_lead, v_rel=v_rel), t=1.25 + index * 0.05)
    assert guard.held_mask == 1
    assert output.leadOne.vLead == 0.0


@pytest.mark.parametrize('role', ('leadOne', 'leadTwo'))
def test_two_range_steps_reuse_confirmed_movement_before_ten_centimeters(role):
  guard = StoppingLeadFilter(reuse_departure_evidence=True)
  establish_stop(guard, role=role)
  for t in (1.25, 1.3):
    assert getattr(update(guard, radar_state(distance=5.55, v_lead=0.1, v_rel=0.1, role=role), t=t), role).vLead == 0.0
  source = radar_state(distance=5.6, v_lead=0.1, v_rel=0.1, role=role)
  assert update(guard, source, t=1.35) is source
  release = guard.snapshot()['leads'][role]['last_release']
  assert release['anchor_distance'] == pytest.approx(5.5)
  assert release['range_growth'] == pytest.approx(0.1)
  assert release['first_motion_mono_ns'] == 1_250_000_000
  assert release['gap_evidence_mono_ns'] == release['release_mono_ns'] == 1_350_000_000
  assert release['release_reason'] == 'rangeMotionConfirmed'
  update(guard, t=1.4, stopping=False)
  assert guard.snapshot()['leads'][role]['last_release'] == release


def test_positive_speed_noise_at_constant_gap_does_not_credit_a_later_range_jump():
  guard = StoppingLeadFilter(reuse_departure_evidence=True)
  establish_stop(guard, v_lead=0.1, v_rel=0.1)
  for index in range(20):
    assert update(guard, radar_state(v_lead=0.1, v_rel=0.1), t=1.25 + index * 0.05).leadOne.vLead == 0.0
  for t in (2.25, 2.3, 2.349):
    assert update(guard, radar_state(distance=5.6, v_lead=0.1, v_rel=0.1), t=t).leadOne.vLead == 0.0
  assert update(guard, radar_state(distance=5.6, v_lead=0.1, v_rel=0.1), t=2.35).leadOne.vLead > 0.0
  assert guard.snapshot()['leads']['leadOne']['last_release']['release_reason'] == 'gapConfirmed'


def test_pre_threshold_range_rebound_cannot_reuse_earlier_movement():
  guard = StoppingLeadFilter(reuse_departure_evidence=True)
  establish_stop(guard)
  for t, distance in ((1.25, 5.55), (1.3, 5.55), (1.35, 5.5), (1.4, 5.6), (1.45, 5.6)):
    assert update(guard, radar_state(distance=distance, v_lead=0.1, v_rel=0.1), t=t).leadOne.vLead == 0.0
  assert update(guard, radar_state(distance=5.6, v_lead=0.1, v_rel=0.1), t=1.5).leadOne.vLead > 0.0


def test_closing_range_above_ten_centimeters_resets_confirmation():
  guard = StoppingLeadFilter(reuse_departure_evidence=True)
  establish_stop(guard)
  for t, distance in ((1.25, 5.7), (1.3, 5.65), (1.35, 5.65), (1.4, 5.65)):
    assert update(guard, radar_state(distance=distance, v_lead=0.1, v_rel=0.1), t=t).leadOne.vLead == 0.0
  assert update(guard, radar_state(distance=5.65, v_lead=0.1, v_rel=0.1), t=1.45).leadOne.vLead > 0.0


@pytest.mark.parametrize('v_lead,v_rel', ((0.0, 0.1), (0.1, 0.0), (0.1, -0.1)))
def test_interrupted_motion_before_threshold_requires_new_confirmation(v_lead, v_rel):
  guard = StoppingLeadFilter(reuse_departure_evidence=True)
  establish_stop(guard)
  assert update(guard, radar_state(distance=5.55, v_lead=0.1, v_rel=0.1), t=1.25).leadOne.vLead == 0.0
  assert update(guard, radar_state(distance=5.55, v_lead=v_lead, v_rel=v_rel), t=1.3).leadOne.vLead == 0.0
  for t in (1.35, 1.4):
    assert update(guard, radar_state(distance=5.6, v_lead=0.1, v_rel=0.1), t=t).leadOne.vLead == 0.0
  assert update(guard, radar_state(distance=5.6, v_lead=0.1, v_rel=0.1), t=1.45).leadOne.vLead > 0.0


def test_pre_threshold_time_credit_only_counts_newer_role_measurements():
  guard = StoppingLeadFilter(reuse_departure_evidence=True)
  establish_stop(guard)
  for t in (1.25, 1.25, 1.2, 1.25):
    assert update(guard, radar_state(distance=5.55, v_lead=0.1, v_rel=0.1), t=t).leadOne.vLead == 0.0
  assert update(guard, radar_state(distance=5.6, v_lead=0.1, v_rel=0.1), t=1.3).leadOne.vLead == 0.0
  assert update(guard, radar_state(distance=5.6, v_lead=0.1, v_rel=0.1), t=1.35).leadOne.vLead > 0.0


def test_refreshing_one_role_does_not_confirm_old_measurements_of_the_other():
  guard = StoppingLeadFilter(reuse_departure_evidence=True)

  def observe(t, one, two, two_time=None):
    state = radar_state(distance=one, v_lead=0.1, v_rel=0.1)
    state.leadTwo = radar_state(distance=two, v_lead=0.1, v_rel=0.1, track_id=43, role='leadTwo').leadTwo
    return update(guard, state, t=t, lead_mono_times={
      'leadOne': round(t * 1e9), 'leadTwo': round((t if two_time is None else two_time) * 1e9),
    })

  for index in range(5):
    observe(1.0 + index * 0.05, 5.5, 5.5)
  observe(1.25, 5.55, 5.55, 1.2)
  observe(1.3, 5.55, 5.6, 1.2)
  output = observe(1.35, 5.6, 5.6, 1.25)
  assert output.leadOne.vLead > 0.0
  assert output.leadTwo.vLead == 0.0
  assert guard.snapshot()['leads']['leadTwo']['evidence']['first_motion_mono_ns'] == 1_250_000_000
  for t, two_time in ((1.4, 1.25), (1.45, 1.3)):
    assert observe(t, 5.65, 5.6, two_time).leadTwo.vLead == 0.0
  assert observe(1.5, 5.7, 5.6, 1.35).leadTwo.vLead > 0.0


def test_new_track_does_not_inherit_confirmed_movement():
  guard = StoppingLeadFilter(reuse_departure_evidence=True)
  establish_stop(guard)
  update(guard, radar_state(distance=5.55, v_lead=0.1, v_rel=0.1), t=1.25)
  replacement = radar_state(distance=5.6, v_lead=0.1, v_rel=0.1, track_id=43)
  assert update(guard, replacement, t=1.3) is replacement
  assert guard.snapshot()['leads']['leadOne']['evidence']['first_motion_mono_ns'] == 0


def test_cold_filter_does_not_hold_an_already_opening_gap():
  guard = StoppingLeadFilter(reuse_departure_evidence=True)
  for index in range(6):
    source = radar_state(distance=5.5 + index * 0.05, v_lead=0.1, v_rel=0.1)
    assert update(guard, source, t=1.0 + index * 0.05) is source


def test_default_filter_keeps_full_confirmation_after_ten_centimeters_despite_prior_movement():
  guard = StoppingLeadFilter()
  establish_stop(guard)
  for t, distance in ((1.25, 5.55), (1.3, 5.55), (1.35, 5.6), (1.4, 5.6)):
    assert update(guard, radar_state(distance=distance, v_lead=0.1, v_rel=0.1), t=t).leadOne.vLead == 0.0
  assert update(guard, radar_state(distance=5.6, v_lead=0.1, v_rel=0.1), t=1.45).leadOne.vLead > 0.0
  assert guard.snapshot()['leads']['leadOne']['last_release']['release_reason'] == 'gapConfirmed'


def test_default_filter_preserves_existing_above_threshold_range_confirmation():
  guard = StoppingLeadFilter()
  establish_stop(guard)
  for t, distance in ((1.25, 5.7), (1.3, 5.65)):
    assert update(guard, radar_state(distance=distance, v_lead=0.1, v_rel=0.1), t=t).leadOne.vLead == 0.0
  assert update(guard, radar_state(distance=5.65, v_lead=0.1, v_rel=0.1), t=1.35).leadOne.vLead > 0.0
