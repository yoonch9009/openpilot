"""CAN encoding/isolation tests, not an ECU or hydraulic simulation."""
import copy
import unittest

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.hyundai.values import CAR
from opendbc.car.hyundai.tests.test_casper_stopping import baseline, candidate, encode, make_state


def scc14(messages):
  parser = CANParser('hyundai_kia_generic', [('SCC14', 50)], 0)
  parser.update([1_000_000_000, messages])
  return dict(parser.vl['SCC14'])


class TestCasperDepartureBands(unittest.TestCase):
  def state(self):
    cs = make_state()
    cs.out.vEgo = 0.0
    return cs

  def pair(self, cs, **kwargs):
    args = dict(stopping=False, accel=.15)
    args.update(kwargs)
    return encode(baseline, cs, **args), encode(candidate, cs, **args)

  def test_only_comfort_fields_change_during_departure(self):
    for v in (-.01, 0, .01, .299):
      for accel in (.01, .15, 1.0, 2.3):
        with self.subTest(v=v, accel=accel):
          cs = self.state()
          cs.out.vEgo = v
          old, new = self.pair(cs, accel=accel)
          a, b = scc14(old), scc14(new)
          self.assertEqual((b['ComfortBandUpper'], b['ComfortBandLower']), (0, 0))
          self.assertGreater(a['ComfortBandUpper'], 0)
          self.assertEqual({k: val for k, val in a.items() if not k.startswith('ComfortBand')},
                           {k: val for k, val in b.items() if not k.startswith('ComfortBand')})
          self.assertEqual([m for m in old if m[0] != 905], [m for m in new if m[0] != 905])

  def test_driver_hold_validity_gear_and_speed_gates(self):
    cases = [('brakePressed', True), ('gasPressed', True), ('parkingBrake', True),
             ('brakeHoldActive', True), ('canValid', False),
             ('gearShifter', structs.CarState.GearShifter.park),
             ('gearShifter', structs.CarState.GearShifter.reverse),
             ('vEgo', .3), ('vEgo', -.3), ('vEgo', 5.0), ('vEgo', float('nan'))]
    for attr, value in cases:
      with self.subTest(attr=attr, value=value):
        cs = self.state()
        setattr(cs.out, attr, value)
        self.assertEqual(*self.pair(cs))
    for kwargs in (dict(enabled=False), dict(stopping=True), dict(long_override=True),
                   dict(accel=0), dict(accel=-.5)):
      with self.subTest(kwargs=kwargs):
        self.assertEqual(*self.pair(self.state(), **kwargs))
    for mode in (0, 1, 2):
      for hold in (1, 2):
        cs = self.state()
        cs.softHoldActive = hold
        self.assertEqual(*self.pair(cs, soft_hold_mode=mode))

  def test_cruise_and_message_availability(self):
    for attr in ('scc12', 'scc14'):
      cs = self.state()
      setattr(cs, attr, None)
      self.assertEqual(*self.pair(cs))
    cs = self.state()
    cs.CP.openpilotLongitudinalControl = False
    self.assertEqual(*self.pair(cs))
    cs = self.state()
    cs.out.cruiseState.available = False
    self.assertEqual(*self.pair(cs))
    cs = self.state()
    cs.paddle_button_prev = 1
    self.assertEqual(*self.pair(cs))

  def test_other_platforms_identical_at_standstill(self):
    for platform in CAR:
      if platform != CAR.HYUNDAI_CASPER:
        with self.subTest(platform=platform):
          cs = self.state()
          cs.CP.carFingerprint = platform
          self.assertEqual(*self.pair(cs))

  def test_repeated_departure_then_motion_brake_and_restop(self):
    cs = self.state()
    for _ in range(5):
      before = copy.deepcopy(cs)
      for _ in range(100):
        self.assertEqual(scc14(self.pair(cs)[1])['ComfortBandUpper'], 0)
      self.assertEqual(cs, before)
      cs.out.vEgo = 1.0
      self.assertEqual(*self.pair(cs))
      cs.out.vEgo = 0.0
      cs.out.brakePressed = True
      self.assertEqual(*self.pair(cs))
      cs.out.brakePressed = False
      stop = encode(candidate, cs, stopping=True, accel=-.5)
      self.assertGreater(scc14(stop)['ComfortBandUpper'], 0)


if __name__ == '__main__':
  unittest.main()
