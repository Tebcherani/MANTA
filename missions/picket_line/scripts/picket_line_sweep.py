import argparse
import copy
import csv
import sys
import yaml

from dataclasses import asdict
from pathlib import Path
from threading import Event

ROOT = Path(__file__).resolve().parents[3]

sys.path.insert(0, str(ROOT / "components" / "manta_core" / "scripts"))

from manta_dataclasses import MonteCarloConfig, MonteCarloResult, SweepConfig, SweepResult
from manta_monte_carlo import compute_monte_carlo_statistics, load_completed_results, run_monte_carlo
from manta_missions import generate_missions
from manta_load import load_yaml


# -----------------------------------------------------
# Helper functions
# -----------------------------------------------------

def generate_fleet_config(sim_config: dict, fleet_size: int, config_dir: Path) -> tuple[dict, Path]:
    """Create one fleet configuration by cloning the first vehicle plugin."""

    if not sim_config.get("vehicles"):
        raise ValueError("The sweep requires at least one vehicle template.")

    scenario_type = sim_config["scenario"]["type"]
    mission_directory = ROOT / "output" / "picket_line" / "monte_carlo_sweep" / "missions" / f"vehicles_{fleet_size:02d}"
    fleet_config = copy.deepcopy(sim_config)
    template = copy.deepcopy(sim_config["vehicles"][0])
    vehicles = []

    for index in range(1, fleet_size + 1):
        vehicle = copy.deepcopy(template)
        vehicle["id"] = f"auv_{index}"
        vehicle["mission"] = f"./{(mission_directory / f'auv_{index}.mi').relative_to(ROOT)}"
        vehicles.append(vehicle)

    fleet_config["vehicles"] = vehicles
    fleet_config_path = config_dir / f"simulation_vehicles_{fleet_size:02d}.yaml"
    fleet_config_path.write_text(yaml.safe_dump(fleet_config, sort_keys=False), encoding="utf-8")

    return fleet_config, fleet_config_path


def load_completed_sweep_results(summary_path: Path) -> dict[int, SweepResult]:
    """Load completed fleet results as SweepResult instances."""

    if not summary_path.is_file(): return {}

    with summary_path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        results = [SweepResult.from_csv_row(row) for row in reader]

    return {result.fleet_size: result for result in results}


def prepare_fleet_experiment(sim_config: dict, fleet_size: int, config_dir: Path, sweep_output_dir: Path) -> tuple[dict, Path]:
    """Generate the configuration, missions, and output path for one fleet size."""

    scenario_type = sim_config["scenario"]["type"]
    mission_directory = ROOT / "output" / "picket_line" / "monte_carlo_sweep" / "missions" / f"vehicles_{fleet_size:02d}"
    fleet_config, fleet_config_path = generate_fleet_config(sim_config, fleet_size, config_dir)
    generate_missions(fleet_config, fleet_config_path, mission_directory)
    fleet_directory = sweep_output_dir / f"vehicles_{fleet_size:02d}"

    return fleet_config, fleet_directory


def prepare_monte_carlo_sweep(sim_config: dict, config_dir: Path, sweep_output_dir: Path, summary_path: Path) -> tuple[SweepConfig, MonteCarloConfig, dict[int, SweepResult]]:
    """Validate sweep settings, prepare output directories, and load completed results."""

    config_dir.mkdir(parents=True, exist_ok=True)
    sweep_output_dir.mkdir(parents=True, exist_ok=True)

    sweep_config = SweepConfig.from_dict(sim_config.get("sweep", {}))
    monte_carlo_config = MonteCarloConfig.from_dict(sim_config.get("monte_carlo", {}))
    completed_results = load_completed_sweep_results(summary_path)

    return sweep_config, monte_carlo_config, completed_results


def record_fleet_result(fleet_size: int, fleet_dir: Path, monte_carlo_config: MonteCarloConfig, completed_results: dict[int, SweepResult], summary_path: Path) -> SweepResult:
    """Summarize one completed fleet experiment and update the sweep summary."""

    experiment_results = list(load_completed_results(fleet_dir / "summary.csv", monte_carlo_config.random_seed).values()) 
    sweep_result = summarize_fleet_results(fleet_size, experiment_results, monte_carlo_config.confidence_level)
    completed_results[fleet_size] = sweep_result
    sweep_rows = [asdict(completed_results[fleet_size]) for fleet_size in sorted(completed_results)]
    write_sweep_summary(summary_path, sweep_rows)

    return sweep_result


def summarize_fleet_results(fleet_size: int, results: list[MonteCarloResult], confidence_level: float) -> SweepResult:
    """Calculate aggregate statistics for one fleet-size experiment."""

    if not results: raise ValueError(f"Fleet size {fleet_size} has no Monte Carlo results.")

    whale_count = results[0].whale_count
    if any(result.whale_count != whale_count for result in results[1:]):
        raise ValueError(f"Fleet size {fleet_size} contains inconsistent whale counts.")

    fractions = [result.detection_fraction for result in results]
    masses = [result.total_mass for result in results]
    energies = [result.total_energy for result in results]
    speeds = [result.mean_vehicle_speed for result in results]

    mean_detection, detection_half_width = compute_monte_carlo_statistics(fractions, confidence_level)
    mean_mass, mass_half_width = compute_monte_carlo_statistics(masses, confidence_level)
    mean_energy, energy_half_width = compute_monte_carlo_statistics(energies, confidence_level)
    mean_speed, speed_half_width = compute_monte_carlo_statistics(speeds, confidence_level)

    return SweepResult(
        fleet_size=fleet_size,
        realizations=len(results),
        whale_count=whale_count,
        mean_detected_whales=mean_detection * whale_count,
        mean_detection=mean_detection,
        ci_lower=max(0.0, mean_detection - detection_half_width),
        ci_upper=min(1.0, mean_detection + detection_half_width),
        ci_half_width=detection_half_width,
        confidence_level=confidence_level,
        mean_total_mass=mean_mass,
        mass_ci_lower=max(0.0, mean_mass - mass_half_width),
        mass_ci_upper=mean_mass + mass_half_width,
        mass_ci_half_width=mass_half_width,
        mean_total_energy=mean_energy,
        energy_ci_lower=max(0.0, mean_energy - energy_half_width),
        energy_ci_upper=mean_energy + energy_half_width,
        energy_ci_half_width=energy_half_width,
        mean_vehicle_speed=mean_speed,
        speed_ci_lower=max(0.0, mean_speed - speed_half_width),
        speed_ci_upper=mean_speed + speed_half_width,
        speed_ci_half_width=speed_half_width,
    )


def write_sweep_summary(path: Path, rows: list[dict]) -> None:
    """Save fleet-level means and confidence intervals."""

    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=(
                "fleet_size",
                "realizations",
                "whale_count",
                "mean_detected_whales",
                "mean_detection",
                "ci_lower",
                "ci_upper",
                "ci_half_width",
                "confidence_level",
                "mean_total_mass",
                "mass_ci_lower",
                "mass_ci_upper",
                "mass_ci_half_width",
                "mean_total_energy",
                "energy_ci_lower",
                "energy_ci_upper",
                "energy_ci_half_width",
                "mean_vehicle_speed",
                "speed_ci_lower",
                "speed_ci_upper",
                "speed_ci_half_width",
            ),
        )
        writer.writeheader()
        writer.writerows(sorted(rows, key=lambda row: row["fleet_size"]))


# -----------------------------------------------------
# Main monte carlo sweep function
# -----------------------------------------------------

def run_monte_carlo_sweep(sim_config: dict) -> int:
    """Run adaptive Monte Carlo experiments for each fleet size."""

    scenario_type = sim_config.get("scenario", {}).get("type")

    if not scenario_type:
        raise ValueError("Configuration is missing scenario.type.")

    sweep_output_dir = ROOT / "output" / scenario_type / "monte_carlo_sweep"
    config_dir = sweep_output_dir / "configs"
    summary_path = sweep_output_dir / "sweep_summary.csv"

    # Prepare the scenario-specific Monte Carlo sweep.
    stop_requested = Event()
    sweep_config, monte_carlo_config, completed_results = prepare_monte_carlo_sweep(sim_config, config_dir, sweep_output_dir, summary_path)

    # Run one Monte Carlo experiment for every requested fleet size.
    for fleet_size in range(sweep_config.min_vehicles, sweep_config.max_vehicles + 1):
        if stop_requested.is_set(): return 130

        print(f"\n=== {scenario_type} Monte Carlo fleet sweep: {fleet_size} vehicle(s) ===")

        # Preserve fleet-size experiments that have already completed.
        if fleet_size in completed_results:
            print(f"Skipping {fleet_size} vehicle(s); sweep results already exist.")
            continue

        # Generate the fleet configuration and run its Monte Carlo experiment.
        fleet_config, fleet_dir = prepare_fleet_experiment(sim_config, fleet_size, config_dir, sweep_output_dir)
        result = run_monte_carlo(sim_config=fleet_config, output_dir=fleet_dir, project_offset=fleet_size * 10000)

        if result != 0: return result

        # Add the completed fleet-size statistics to the sweep summary.
        sweep_result = record_fleet_result(fleet_size, fleet_dir, monte_carlo_config, completed_results, summary_path)
        print(f"Fleet size {fleet_size}: mean {sweep_result.mean_detection:.2%}, CI half-width {sweep_result.ci_half_width:.2%} from {sweep_result.realizations} runs.")

    print(f"Sweep data: {summary_path}")
    print("Run './picket_line review sweep' to generate the sweep plots.")

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Sweep picket-line fleet size.")
    parser.add_argument("config_path", type=Path)
    args = parser.parse_args()

    config = load_yaml(args.config_path.resolve())

    if config["scenario"]["type"] != "picket_line":
        raise ValueError("Expected a picket_line scenario.")

    return run_monte_carlo_sweep(config)


if __name__ == "__main__":
    raise SystemExit(main())