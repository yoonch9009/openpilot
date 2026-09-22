import unittest
from tools.casper_stop_observer import summarize


def row(t, **changes):
  return dict({'t': t, 'route': 'test', 'cs_age': 0, 'plan_age': 0,
               'v': 0, 'brake': False, 'gas': False, 'active': True,
               'should_stop': True, 'state': 'stopping', 'request': -.5}, **changes)


class StopObserverTest(unittest.TestCase):
  def stationary(self, seconds=30):
    return [row(0, v=1)] + [row(i / 10) for i in range(1, seconds * 10 + 1)]

  def request(self, t=30.1, **kw):
    return row(t, should_stop=False, state='pid', request=.3, **kw)

  def test_long_stop_automatic_motion(self):
    d = summarize(self.stationary() + [self.request(), self.request(30.2, v=.4)])[0]
    self.assertEqual(d['outcome'], 'motion_without_pedal_input')
    self.assertAlmostEqual(d['stationary_s'], 30)
    self.assertFalse(d['stationary_duration_is_lower_bound'])

  def test_driver_pedal_must_not_count_as_automatic_success(self):
    d = summarize(self.stationary() + [self.request(), self.request(30.2, gas=True, v=.4)])[0]
    self.assertEqual(d['outcome'], 'driver_intervened')

  def test_missing_segment_does_not_count_as_long_hold(self):
    d = summarize([row(0, v=1), row(.1)] + [row(29 + i / 10) for i in range(11)] +
                  [self.request(), self.request(30.2, v=.4)])[0]
    self.assertAlmostEqual(d['stationary_s'], 1.1)
    self.assertTrue(d['stationary_duration_is_lower_bound'])

  def test_stale_plan_cannot_trigger_departure(self):
    self.assertEqual(summarize(self.stationary() + [self.request(plan_age=1)]), [])

  def test_recording_end_is_not_a_proven_vehicle_failure(self):
    d = summarize(self.stationary() + [self.request()])[0]
    self.assertEqual(d['outcome'], 'recording_ended')

  def test_new_stop_request_ends_attempt(self):
    d = summarize(self.stationary() + [self.request(), row(30.2)])[0]
    self.assertEqual(d['outcome'], 'stop_requested_again')

  def test_new_route_does_not_inherit_stationary_timer(self):
    self.assertEqual(summarize(self.stationary() + [self.request(route='other')]), [])

  def test_pedal_release_before_motion_does_not_become_auto_departure(self):
    ds = summarize(self.stationary() + [self.request(), self.request(30.2, gas=True),
                   self.request(30.3), self.request(30.4, v=.4)])
    self.assertEqual([d['outcome'] for d in ds], ['driver_intervened'])


if __name__ == '__main__':
  unittest.main()
