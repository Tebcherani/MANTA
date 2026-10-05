import importlib.util
import sys

from paths import ROOT


def run_monte_carlo_sweep(config: dict) -> int:
    """Run the sweep script selected by mission configuration."""

    settings = config.get("mission_generation")
    if not isinstance(settings, dict):
        raise ValueError("Configuration is missing mission_generation.")

    script_setting = settings.get("sweep")
    if not isinstance(script_setting, str) or not script_setting.strip():
        raise ValueError("Configuration is missing mission_generation.sweep.")
    
    script = (ROOT / script_setting).resolve()

    if not script.is_relative_to(ROOT.resolve()):
        raise ValueError("Sweep script must be inside MANTA.")
    if not script.is_file():
        raise ValueError(f"Sweep script does not exist: {script}")

    module_name = "_manta_mission_sweep"
    spec = importlib.util.spec_from_file_location(module_name, script)

    if spec is None or spec.loader is None:
        raise ValueError(f"Cannot load sweep script: {script}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)

    run = getattr(module, "run_monte_carlo_sweep", None)
    if not callable(run):
        raise ValueError(
            f"Sweep script must define run_monte_carlo_sweep(config): {script}"
        )

    return run(config)