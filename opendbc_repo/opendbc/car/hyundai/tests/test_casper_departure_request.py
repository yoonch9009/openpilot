"""Keep the withdrawn StopReq departure trial out of ordinary Casper control."""
import unittest

from opendbc.car.hyundai.tests.test_casper_stopping import baseline, candidate, decode, encode, make_state


class TestCasperDepartureRequest(unittest.TestCase):
  def test_departure_does_not_assert_stopreq_when_speed_falls_again(self):
    cs = make_state()
    for _ in range(3):
      cs.out.vEgo = 0.0
      self.assertEqual(decode(encode(candidate, cs, stopping=True, accel=-.5))['StopReq'], 0)
      # Cross the removed 0.3 m/s threshold in both directions and remain stopped.
      for idx, v in enumerate([0.0] * 250 + [.31, .29, .1, 0.0] * 20):
        cs.out.vEgo = v
        args = dict(stopping=False, accel=.15, idx=idx)
        actual = encode(candidate, cs, **args)
        self.assertEqual(actual, encode(baseline, cs, **args))
        self.assertEqual(decode(actual)['StopReq'], 0)
        packet = next(m[1] for m in actual if m[0] == 1057)
        self.assertEqual(sum((v >> 4) + (v & 15) for v in packet) % 16, 0)
      self.assertEqual(decode(encode(candidate, cs, stopping=True, accel=-.5))['StopReq'], 0)


if __name__ == '__main__':
  unittest.main()
