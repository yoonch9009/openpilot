"""Live DM mode, independent boot-latched on/off, and legacy streaming migration."""
import os


def experimental_mode(params):
  return params.get_int("DriverMonitoringMode") == 1


def monitoring_enabled(params, environ=None):
  env = os.environ if environ is None else environ
  if "CARROT_DM_DISABLED" in env:
    return env["CARROT_DM_DISABLED"] == "0"
  return params.get_int("DisableDM") == 0


def configure_monitoring(params, environ=None):
  env = os.environ if environ is None else environ
  # Latch on/off for all child processes together; changing it requires restart.
  env["CARROT_DM_DISABLED"] = "0" if params.get_int("DisableDM") == 0 else "1"
  # Old value 2 also enabled road streaming; preserve it independently.
  if params.get("DriverMonitoringMode") is None:
    if params.get("CarrotVisionEnabled") is None:
      params.put_bool("CarrotVisionEnabled", params.get_int("DisableDM") == 2)
    params.put_int("DriverMonitoringMode", 0)
  env.pop("CARROT_DM_MODE", None)  # Retired startup latch must not override live Params.
