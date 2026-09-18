import csv
import yaml

from paths import ROOT

from manta_runner_old import make_sweep_config, generate_scenario_missions, validate_config, write_sweep_summary
from manta_runner_old import run_monte_carlo, load_monte_carlo_summary, validate_monte_carlo_config, calculate_monte_carlo_statistics

def run_monte_carlo_sweep(
    stop_requested,
    config: dict,
    minimum_vehicles: int = 1,
    maximum_vehicles: int = 10,
) -> int:
    """Run adaptive Monte Carlo experiments for each fleet size."""

    sweep_directory = ROOT / "output" / "monte_carlo_sweep"
    generated_directory = ROOT / "generated" / "monte_carlo_sweep"
    summary_path = sweep_directory / "sweep_summary.csv"
    generated_directory.mkdir(parents=True, exist_ok=True)

    sweep_rows = []

    if summary_path.is_file():
        with summary_path.open("r", encoding="utf-8", newline="") as file:
            sweep_rows = list(csv.DictReader(file))

        for row in sweep_rows:
            row["vehicle_count"] = int(row["vehicle_count"])
            row["realizations"] = int(row["realizations"])

    completed_counts = {
        int(row["vehicle_count"]): row
        for row in sweep_rows
    }

    for vehicle_count in range(minimum_vehicles, maximum_vehicles + 1):
        if stop_requested.is_set():
            return 130

        print(
            f"\n=== Monte Carlo fleet sweep: "
            f"{vehicle_count} vehicle(s) ==="
        )

        config = make_sweep_config(config, vehicle_count)
        fleet_config_path = (
            generated_directory
            / f"simulation_vehicles_{vehicle_count:02d}.yaml"
        )
        fleet_config_path.write_text(
            yaml.safe_dump(config, sort_keys=False),
            encoding="utf-8",
        )

        generate_scenario_missions(
            config,
            fleet_config_path,
            ROOT,
        )
        validate_config(config, ROOT)

        fleet_directory = (
            sweep_directory
            / f"vehicles_{vehicle_count:02d}"
        )
        result = run_monte_carlo(
            config,
            ROOT,
            output_directory=fleet_directory,
            project_offset=vehicle_count * 10000,
        )

        if result != 0:
            return result

        monte_carlo = validate_monte_carlo_config(config)
        rows = load_monte_carlo_summary(
            fleet_directory / "summary.csv",
            monte_carlo["random_seed"],
        )
        fractions = [row["detection_fraction"] for row in rows]
        masses = [row["total_mass_kg"] for row in rows]
        energies = [row["total_energy_wh"] for row in rows]
        speeds = [row["mean_vehicle_speed_mps"] for row in rows]
        whale_count = int(config["scenario"]["whales"]["count"])
        confidence_level = float(monte_carlo["confidence_level"])
        sample_mean, half_width = calculate_monte_carlo_statistics(
            fractions,
            confidence_level,
        )
        mean_mass, mass_half_width = calculate_monte_carlo_statistics(
            masses,
            confidence_level,
        )
        mean_energy, energy_half_width = calculate_monte_carlo_statistics(
            energies,
            confidence_level,
        )
        mean_speed, speed_half_width = calculate_monte_carlo_statistics(
            speeds,
            confidence_level,
        )

        completed_counts[vehicle_count] = {
            "vehicle_count": vehicle_count,
            "realizations": len(rows),
            "whale_count": whale_count,
            "mean_detected_whales": sample_mean * whale_count,
            "mean_detection": sample_mean,
            "ci_lower": max(0.0, sample_mean - half_width),
            "ci_upper": min(1.0, sample_mean + half_width),
            "ci_half_width": half_width,
            "confidence_level": monte_carlo["confidence_level"],
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
        sweep_rows = list(completed_counts.values())
        write_sweep_summary(summary_path, sweep_rows)

        print(
            f"Fleet size {vehicle_count}: mean {sample_mean:.2%}, "
            f"CI half-width {half_width:.2%} from {len(rows)} runs."
        )

    print(f"Sweep data: {summary_path}")
    print("Run './manta review sweep' to generate the sweep plots.")
    return 0