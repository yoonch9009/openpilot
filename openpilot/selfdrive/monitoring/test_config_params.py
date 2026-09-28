"""Exercise boot migration with the native typed Params implementation."""
import pytest

from openpilot.selfdrive.monitoring.config import configure_monitoring, monitoring_enabled


@pytest.mark.parametrize('legacy', [None, 0, 1, 2])
def test_boot_configuration_with_native_params(tmp_path, legacy):
  native = pytest.importorskip('openpilot.common.params_pyx')
  params = native.Params(str(tmp_path / 'params'))
  if legacy is not None:
    params.put_int('DisableDM', legacy)
  env = {}
  configure_monitoring(params, env)
  assert type(params.get('DriverMonitoringMode')) is int
  assert params.get('DriverMonitoringMode') == 0
  assert params.get('CarrotVisionEnabled') is (legacy == 2)
  assert 'CARROT_DM_MODE' not in env
  params.put_int('DriverMonitoringMode', 1)
  params.put_bool('CarrotVisionEnabled', False)
  configure_monitoring(params, env)
  assert 'CARROT_DM_MODE' not in env
  assert params.get('CarrotVisionEnabled') is False


class DictParams:
  def __init__(self, legacy):
    self.values = {'DisableDM': legacy, 'DriverTooDistracted': True}

  def get(self, key):
    return self.values.get(key)

  def get_int(self, key):
    return int(self.values.get(key, 0))

  def put_int(self, key, value):
    self.values[key] = value

  def put_bool(self, key, value):
    self.values[key] = bool(value)


@pytest.mark.parametrize('legacy', [0, 1, 2])
def test_independent_enable_and_standard_mode_migration(legacy):
  params, env = DictParams(legacy), {}
  configure_monitoring(params, env)
  assert monitoring_enabled(params, env) is (legacy == 0)
  assert params.get('DriverMonitoringMode') == 0
  assert params.get('CarrotVisionEnabled') is (legacy == 2)
  assert params.get('DriverTooDistracted') is True
  params.put_int('DriverMonitoringMode', 1)
  assert monitoring_enabled(params, env) is (legacy == 0)


def test_enable_change_waits_for_restart_and_preserves_lockout():
  params, env = DictParams(1), {}
  configure_monitoring(params, env)
  params.put_int('DisableDM', 0)
  assert not monitoring_enabled(params, env)
  configure_monitoring(params, env)
  assert monitoring_enabled(params, env)
  params.put_int('DisableDM', 1)
  assert monitoring_enabled(params, env)
  assert params.get('DriverTooDistracted') is True


def test_force_deceleration_preserves_non_dm_soft_disabling():
  # Execute the real forceDecel assignment without importing native controls.
  import ast
  from pathlib import Path
  from types import SimpleNamespace as NS
  source = Path(__file__).parents[1] / 'controls' / 'controlsd.py'
  tree = ast.parse(source.read_text(encoding='utf-8'))
  node = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign) and
              any(isinstance(t, ast.Attribute) and t.attr == 'forceDecel' for t in n.targets))
  code = compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), str(source), 'exec')
  for enabled in (False, True):
    for dm_alert in (0, 3):
      for state in ('enabled', 'softDisabling'):
        cs = NS()
        sm = {'driverMonitoringState': NS(alertLevel=dm_alert), 'selfdriveState': NS(state=state)}
        namespace = dict(self=NS(dm_enabled=enabled, sm=sm), cs=cs,
                         log=NS(DriverMonitoringState=NS(AlertLevel=NS(three=3))),
                         State=NS(softDisabling='softDisabling'))
        exec(code, namespace)
        assert cs.forceDecel is ((enabled and dm_alert == 3) or state == 'softDisabling')


@pytest.mark.parametrize('disabled', [False, True])
def test_manager_process_gate_matches_boot_state(monkeypatch, disabled):
  import ast
  from pathlib import Path
  from types import SimpleNamespace as NS
  from openpilot.selfdrive.monitoring.config import monitoring_enabled
  monkeypatch.setenv('CARROT_DM_DISABLED', '1' if disabled else '0')
  source = Path(__file__).parents[2] / 'system' / 'manager' / 'process_config.py'
  node = next(n for n in ast.parse(source.read_text(encoding='utf-8')).body
              if isinstance(n, ast.FunctionDef) and n.name == 'enable_dm')
  for arg in node.args.args:
    arg.annotation = None
  node.returns = None
  namespace = {'monitoring_enabled': monitoring_enabled}
  exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), str(source), 'exec'), namespace)
  for started in (False, True):
    for preview in (False, True):
      params = NS(get_bool=lambda key: preview)
      assert namespace['enable_dm'](started, params, None) is (not disabled and (started or preview))
