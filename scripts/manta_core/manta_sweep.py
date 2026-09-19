import copy
import csv
import yaml

from manta_core.manta_dataclasses import MonteCarloConfig, SweepConfig
from manta_core.manta_monte_carlo import run_monte_carlo
from manta_core.manta_run import generate_missions
from pathlib import Path
from paths import ROOT, SWEEP_GENERATED_DIR, SWEEP_SUMMARY_PATH, SWEEP_OUTPUT_DIR
from threading import Event

from manta_runner_old import write_sweep_summary
from manta_runner_old import load_monte_carlo_summary, calculate_monte_carlo_statistics
    

def generate_fleet_config(sim_config: dict, fleet_size: int) -> tuple[dict, Path]:
    """Create one fleet configuration by cloning the first vehicle plugin."""

    if not sim_config.get("vehicles"): raise ValueError("The sweep requires at least one vehicle template.")

    fleet_config = copy.deepcopy(sim_config)
    template = copy.deepcopy(sim_config["vehicles"][0])
    vehicles = []

    for index in range(1, fleet_size + 1):
        vehicle = copy.deepcopy(template)
        vehicle["id"] = f"auv_{index}"
        vehicle["mission"] = (f"./generated/missions/picket_line_auv_{index}.mi")
        vehicles.append(vehicle)

    fleet_config["vehicles"] = vehicles
    fleet_config_path = SWEEP_GENERATED_DIR / f"simulation_vehicles_{fleet_size:02d}.yaml"
    fleet_config_path.write_text(yaml.safe_dump(fleet_config, sort_keys=False), encoding="utf-8")
    
    return fleet_config, fleet_config_path


def load_completed_sweep_results() -> dict[int, dict[str, str | int]]:
    """Load existing sweep results, keyed by completed fleet size."""

    if not SWEEP_SUMMARY_PATH.is_file(): return {}
    with SWEEP_SUMMARY_PATH.open("r", encoding="utf-8", newline="") as file: rows = list(csv.DictReader(file))

    for row in rows:
        row["fleet_size"] = int(row["fleet_size"])
        row["realizations"] = int(row["realizations"])

    return {row["fleet_size"]: row for row in rows}


def run_monte_carlo_sweep(config: dict) -> int:
    """Run adaptive Monte Carlo experiments for each fleet size."""

    stop_requested = Event()

    monte_carlo = MonteCarloConfig.from_dict(config.get("monte_carlo", {}))
    sweep = SweepConfig.from_dict(config.get("sweep", {}))

    SWEEP_GENERATED_DIR.mkdir(parents=True, exist_ok=True)
    SWEEP_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    completed_sweep_results = load_completed_sweep_results()

    for fleet_size in range(sweep.min_vehicles, sweep.max_vehicles+1):
        if stop_requested.is_set(): return 130
        if fleet_size in completed_sweep_results:
            print(f"Skipping {fleet_size} vehicle(s); sweep results already exist.")
            continue

        print(f"\n=== Monte Carlo fleet sweep: {fleet_size} vehicle(s) ===")

        fleet_config, fleet_config_path = generate_fleet_config(config, fleet_size)
        generate_missions(fleet_config, fleet_config_path)

        fleet_directory = SWEEP_OUTPUT_DIR / f"vehicles_{fleet_size:02d}"
        result = run_monte_carlo(fleet_config, output_directory=fleet_directory, project_offset=fleet_size*10000)

        if result != 0:
            return result

        rows = load_monte_carlo_summary(
            fleet_directory / "summary.csv",
            monte_carlo.random_seed,
        )
        fractions = [row["detection_fraction"] for row in rows]
        masses = [row["total_mass_kg"] for row in rows]
        energies = [row["total_energy_wh"] for row in rows]
        speeds = [row["mean_vehicle_speed_mps"] for row in rows]
        whale_count = int(config["scenario"]["whales"]["count"])
        sample_mean, half_width = calculate_monte_carlo_statistics(
            fractions,
            monte_carlo.confidence_level,
        )
        mean_mass, mass_half_width = calculate_monte_carlo_statistics(
            masses,
            monte_carlo.confidence_level,
        )
        mean_energy, energy_half_width = calculate_monte_carlo_statistics(
            energies,
            monte_carlo.confidence_level,
        )
        mean_speed, speed_half_width = calculate_monte_carlo_statistics(
            speeds,
            monte_carlo.confidence_level,
        )

        completed_sweep_results[fleet_size] = {
            "fleet_size": fleet_size,
            "realizations": len(rows),
            "whale_count": whale_count,
            "mean_detected_whales": sample_mean * whale_count,
            "mean_detection": sample_mean,
            "ci_lower": max(0.0, sample_mean - half_width),
            "ci_upper": min(1.0, sample_mean + half_width),
            "ci_half_width": half_width,
            "confidence_level": monte_carlo.confidence_level,
            "mean_total_mass_kg": mean_mass,
            "mass_ci_lower_kg": max(0.0, mean_mass - mass_half_width),
            "mass_ci_upper_kg": mean_mass + mass_half_width,
            "mass_ci_half_width_kg": mass_half_width,
            "mean_total_energy_wh": mean_energy,
            "energy_ci_lower_wh": max(
                0.0,
                mean_energy - energy_half_width,
            ),
            "energy_ci_upper_wh": mean_energy + energy_half_width,
            "energy_ci_half_width_wh": energy_half_width,
            "mean_vehicle_speed_mps": mean_speed,
            "speed_ci_lower_mps": max(
                0.0,
                mean_speed - speed_half_width,
            ),
            "speed_ci_upper_mps": mean_speed + speed_half_width,
            "speed_ci_half_width_mps": speed_half_width,
        }
        sweep_rows = list(completed_sweep_results.values())
        write_sweep_summary(SWEEP_SUMMARY_PATH, sweep_rows)

        print(
            f"Fleet size {fleet_size}: mean {sample_mean:.2%}, "
            f"CI half-width {half_width:.2%} from {len(rows)} runs."
        )

    print(f"Sweep data: {SWEEP_SUMMARY_PATH}")
    print("Run './manta review sweep' to generate the sweep plots.")
    return 0