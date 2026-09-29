"""Execute the production timestamp and controller blocks without native IPC."""
import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest


ROOT = Path(__file__).resolve().parents[5]
spec = importlib.util.spec_from_file_location('casper_launch_jerk_inputs', Path(__file__).parents[1] / 'casper_launch_jerk.py')
helper_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper_module)


def compiled_block(path, predicate, prefix=0):
  tree = ast.parse(path.read_text(encoding='utf-8'))
  for parent in ast.walk(tree):
    body = getattr(parent, 'body', None)
    if not isinstance(body, list):
      continue
    for index, node in enumerate(body):
      if isinstance(node, ast.If) and ast.unparse(node.test) == predicate:
        nodes = body[index - prefix:index + 1]
        return compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])), str(path), 'exec')
  raise AssertionError(f'Production block not found: {predicate}')


class TestCasperLaunchInputFreshness(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.marker_code = compiled_block(ROOT / 'openpilot/selfdrive/controls/controlsd.py',
                                     'self.LoC.casper_launch is not None', prefix=2)
    cls.controller_code = compiled_block(Path(__file__).parents[1] / 'carcontroller.py',
                                         'self.casper_launch_jerk is not None')

  def marker(self, timestamps, *, valid=True, casper=True, now=10_000_000_000):
    sm = NS(all_checks=lambda services: valid, logMonoTime=timestamps)
    cc = NS(casperLaunchInputsMonoTime=123)  # Must overwrite a previous good marker.
    scope = dict(self=NS(sm=sm, LoC=NS(casper_launch=object() if casper else None)),
                 CC=cc, time=NS(monotonic_ns=lambda: now))
    exec(self.marker_code, scope)
    return cc.casperLaunchInputsMonoTime, scope['launch_inputs_valid']

  def test_marker_is_oldest_valid_source(self):
    self.assertEqual(self.marker(dict(carState=9_990_000_000, longitudinalPlan=9_900_000_000,
                                      radarState=9_800_000_000)), (9_800_000_000, True))

  def test_stale_future_and_invalid_sources_clear_previous_marker(self):
    for source in ('carState', 'longitudinalPlan', 'radarState'):
      for bad_timestamp in (0, 9_699_999_999, 10_000_000_001):
        with self.subTest(source=source, timestamp=bad_timestamp):
          timestamps = dict.fromkeys(('carState', 'longitudinalPlan', 'radarState'), 10_000_000_000)
          timestamps[source] = bad_timestamp
          self.assertEqual(self.marker(timestamps), (0, False))
    timestamps = dict.fromkeys(('carState', 'longitudinalPlan', 'radarState'), 10_000_000_000)
    self.assertEqual(self.marker(timestamps, valid=False), (0, False))
    self.assertEqual(self.marker(timestamps, casper=False), (0, True))

  def controller_fixture(self):
    helper = helper_module.CasperLaunchJerk()
    jerk = NS(carrot_cruise=0, jerk_u=.5)
    cs = NS(casper_tcs13_mono_ns=10_000_000_000, casper_brake_control_active=False,
            scc12={}, scc14={}, softHoldActive=0,
            out=NS(canValid=True, canTimeout=False, accFaulted=False, gearShifter='drive',
                   cruiseState=NS(available=True), brakePressed=False, gasPressed=False,
                   parkingBrake=False, brakeHoldActive=False, vEgo=0., aEgo=.1))
    cc = NS(enabled=True, longActive=True, cruiseControl=NS(override=False))
    return dict(self=NS(casper_launch_jerk=helper, hyundai_jerk=jerk), CS=cs, CC=cc,
                structs=NS(CarState=NS(GearShifter=NS(drive='drive'))), now_nanos=10_000_000_000,
                stopping=True, hud_control=NS(leadVisible=True, leadDistance=5., leadRelSpeed=.5),
                actuators=NS(aTarget=.8, jerk=.1, longControlState='pid'), LongCtrlState=NS(pid='pid'), accel=.6)

  def run_controller(self, scope, marker_age=0, tcs_age=0, missing=False):
    scope['now_nanos'] += 20_000_000
    scope['CS'].casper_tcs13_mono_ns = scope['now_nanos'] - tcs_age if tcs_age is not None else 0
    if missing:
      if hasattr(scope['CC'], 'casperLaunchInputsMonoTime'):
        del scope['CC'].casperLaunchInputsMonoTime
    else:
      scope['CC'].casperLaunchInputsMonoTime = scope['now_nanos'] - marker_age
    exec(self.controller_code, scope)
    return scope['launch_jerk_upper']

  def armed_controller(self):
    scope = self.controller_fixture()
    self.run_controller(scope)
    scope['CC'].longActive = False
    self.run_controller(scope)
    scope['CC'].longActive = True
    self.run_controller(scope)
    scope['CS'].out.vEgo = .5
    scope['stopping'] = False
    self.assertGreater(self.run_controller(scope), .5)
    return scope

  def test_controller_clears_assist_for_missing_stale_future_marker(self):
    for args in (dict(missing=True), dict(marker_age=300_000_001), dict(marker_age=-1)):
      with self.subTest(args=args):
        scope = self.armed_controller()
        self.assertEqual(self.run_controller(scope, **args), .5)
        self.assertEqual(self.run_controller(scope), .5)

  def test_controller_tcs_freshness_and_exact_marker_age_boundary(self):
    scope = self.armed_controller()
    self.assertGreater(self.run_controller(scope, marker_age=300_000_000), .5)
    for tcs_age in (None, 100_000_001, -1):
      with self.subTest(tcs_age=tcs_age):
        scope = self.armed_controller()
        self.assertEqual(self.run_controller(scope, tcs_age=tcs_age), .5)
        self.assertEqual(self.run_controller(scope), .5)


if __name__ == '__main__':
  unittest.main()
