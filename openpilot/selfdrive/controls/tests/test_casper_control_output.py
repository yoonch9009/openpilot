"""Execute the longitudinal publication block with the real LongControl."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from openpilot.cereal import car, log
from opendbc.car.hyundai.values import CAR as HYUNDAI_CAR
from openpilot.selfdrive.controls.tests.test_casper_stop_hold import control, inputs


@pytest.mark.parametrize('active,should_stop,expected', [
  (True, False, 'pid'), (True, True, 'stopping'), (False, False, 'off')])
def test_casper_publishes_state_matching_same_tick_acceleration(control, active, should_stop, expected):
  # Execute the actual publication statements without starting lateral control,
  # IPC or hardware. This detects the original before-update state copy.
  path = Path(__file__).parents[1] / 'controlsd.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Controls')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'state_control')
  def assigns(node, name):
    return isinstance(node, ast.Assign) and any(ast.unparse(t) == name for t in node.targets)
  start = next(i for i, n in enumerate(method.body) if assigns(n, 'actuators'))
  end = next(i for i, n in enumerate(method.body) if assigns(n, 'actuators.jerk'))
  program = compile(ast.Module(body=method.body[start:end+1], type_ignores=[]), str(path), 'exec')
  cs, plan, radar = inputs(should_stop=should_stop)
  cc = car.CarControl.new_message(longActive=active)
  class SM(dict):
    frame = 0
    recv_frame = {'longitudinalPlan': 0}
  self = NS(LoC=control, LaC=NS(reset=lambda: None), CP=control.CP,
            CI=NS(get_pid_accel_limits=lambda *_: (-3.5, 2.0)), sm=SM(radarState=radar))
  namespace = dict(self=self, CS=cs, CC=cc, long_plan=plan, DT_CTRL=.01,
                   CV=NS(KPH_TO_MS=1/3.6), HYUNDAI_CAR=HYUNDAI_CAR,
                   model_v2=NS(meta=NS(laneChangeState=log.LaneChangeState.off)),
                   LaneChangeState=log.LaneChangeState)
  exec(program, namespace)
  assert str(cc.actuators.longControlState) == expected
  assert cc.actuators.longControlState == control.long_control_state
  if expected == 'pid':
    assert cc.actuators.accel > 0
  elif expected == 'stopping':
    assert cc.actuators.accel <= -.5
  else:
    assert cc.actuators.accel == 0
