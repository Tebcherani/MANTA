import csv
import os
import math
import signal
import statistics
import subprocess
import sys

from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict
from manta_dataclasses import MonteCarloConfig, MonteCarloResult
from manta_realization import stop_requested, run_monte_carlo_realization
from pathlib import Path
from paths import ROOT, CLEANUP_PATH


# -----------------------------------------------------
# Helper functions
# -----------------------------------------------------

def check_monte_carlo_convergence(fractions: list[float], monte_carlo_config: MonteCarloConfig) -> tuple[bool, float, float, float]:
    """Calculate convergence using the current and preceding cumulative means."""

    if not fractions: return False, math.nan, math.inf, math.inf

    sample_mean, half_width = compute_monte_carlo_statistics(fractions, monte_carlo_config.confidence_level)
    previous_count = len(fractions) - monte_carlo_config.batch_size
    previous_mean = statistics.mean(fractions[:previous_count]) if previous_count > 0 else None
    mean_change = abs(sample_mean - previous_mean) if previous_mean is not None else math.inf

    converged = (
        len(fractions) >= monte_carlo_config.min_realizations
        and half_width <= monte_carlo_config.target_half_width
        and mean_change <= monte_carlo_config.stability_tolerance
    )

    return converged, sample_mean, half_width, mean_change

def get_convergence_values(completed: dict[int, MonteCarloResult], scenario_type: str) -> tuple[list[float], str, str]:
    """Select the realization metric used to test Monte Carlo convergence."""

    results = [completed[index] for index in sorted(completed)]

    if scenario_type == "trailing":
        if any(result.mean_distance_km is None for result in results):
            raise ValueError("Trailing Monte Carlo results are missing mean distances.")

        values = [result.mean_distance_km for result in results]
        return values, "mean distance", "km"

    values = [result.detection_fraction for result in results]
    return values, "mean detection", "%"

def check_saved_result_convergence(completed: dict[int, MonteCarloResult], monte_carlo_config: MonteCarloConfig, summary_path: Path, scenario_type: str) -> bool:
    """Report whether previously completed realizations already satisfy convergence."""

    values, metric_name, unit = get_convergence_values(completed, scenario_type)
    if not values:
        return False

    converged, sample_mean, half_width, mean_change = check_monte_carlo_convergence(values, monte_carlo_config)

    if unit == "%":
        print(f"Current {len(values)} realizations: {metric_name} {sample_mean:.2%}, CI half-width {half_width:.2%}, mean change {mean_change:.2%}.")
    else:
        print(f"Current {len(values)} realizations: {metric_name} {sample_mean:.3f} {unit}, CI half-width {half_width:.3f} {unit}, mean change {mean_change:.3f} {unit}.")

    if converged:
        print(f"Saved results already satisfy convergence after {len(values)} realizations.")
        print(f"Summary: {summary_path}")

    return converged


def cleanup_monte_carlo_projects() -> None:
    """Remove all MANTA Monte Carlo containers and networks."""

    if not CLEANUP_PATH.is_file(): raise ValueError(f"Monte Carlo cleanup script is missing: {CLEANUP_PATH}")
    result = subprocess.run(["bash", str(CLEANUP_PATH)], cwd=ROOT, check=False)
    if result.returncode != 0: raise ValueError("Failed to clean up MANTA Monte Carlo Docker projects.")
    

def compute_monte_carlo_statistics(detection_fractions: list[float], confidence_level: float) -> tuple[float, float]:
    """Return the sample mean and normal-approximation confidence-interval half-width."""

    if not detection_fractions: raise ValueError("At least one realization is required.")

    sample_mean = statistics.mean(detection_fractions)
    if len(detection_fractions) < 2: return sample_mean, math.inf

    standard_error = statistics.stdev(detection_fractions) / math.sqrt(len(detection_fractions))
    z_score = statistics.NormalDist().inv_cdf(0.5 + confidence_level / 2.0)
    half_width = z_score * standard_error

    return sample_mean, half_width


def compute_runs_to_next_checkpoint(completed_count: int, monte_carlo_config: MonteCarloConfig) -> int:
    """Return the number of realizations needed to reach the next checkpoint."""

    remaining_runs = monte_carlo_config.max_realizations - completed_count
    if remaining_runs <= 0: return 0

    remainder = completed_count % monte_carlo_config.batch_size
    checkpoint_runs = monte_carlo_config.batch_size if remainder == 0 else monte_carlo_config.batch_size - remainder

    return min(checkpoint_runs, remaining_runs)


def get_next_batch(next_realization: int, runs_needed: int, completed_results: dict, max_realizations: int) -> list[int]:
    """Return the next incomplete realization numbers to run."""

    batch = []
    candidate = next_realization

    while len(batch) < runs_needed and candidate <= max_realizations:
        if candidate not in completed_results: batch.append(candidate)
        candidate += 1

    return batch


def get_worker_count(sim_config: dict, monte_carlo_config: MonteCarloConfig) -> int:
    """Choose a safe number of parallel realizations for the fleet size."""

    cpu_count = os.cpu_count() or 1
    default_workers = max(1, cpu_count // max(1, len(sim_config["vehicles"])))
    requested_workers = monte_carlo_config.workers if monte_carlo_config.workers is not None else default_workers
    return max(1, min(requested_workers, monte_carlo_config.batch_size, default_workers, 200))


def handle_interrupt(signal_number, frame) -> None:
    """Stop every active Monte Carlo project and suppress retries."""

    if stop_requested.is_set(): return
    print("\nStopping MANTA Monte Carlo...", file=sys.stderr)
    stop_requested.set()
    subprocess.run(["bash", str(CLEANUP_PATH)], check=False)


def load_completed_results(path: Path, base_seed: int) -> dict[int, MonteCarloResult]:
    """Load completed Monte Carlo realizations keyed by realization number."""

    if not path.is_file(): return {}

    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        required = {"realization", "seed", "detected_whales", "whale_count", "detection_fraction", "total_mass", "total_energy", "mean_vehicle_speed"}
        if not reader.fieldnames or not required.issubset(reader.fieldnames): raise ValueError(f"Monte Carlo summary has an invalid header: {path}")

        results = [MonteCarloResult.from_csv_row(row, base_seed) for row in reader]

    return {result.realization: result for result in sorted(results, key=lambda result: result.realization)}


def prepare_monte_carlo_experiment(sim_config: dict, output_dir: Path) -> tuple[MonteCarloConfig, int, dict[int, dict]]:
    """Load Monte Carlo settings, select workers, and restore completed realizations."""

    # Get worker count
    monte_carlo_config = MonteCarloConfig.from_dict(sim_config.get("monte_carlo", {}))
    workers = get_worker_count(sim_config, monte_carlo_config)

    # Get summary path
    summary_path = output_dir / "summary.csv"

    # Load completed results
    completed_results = load_completed_results(summary_path, monte_carlo_config.random_seed)
    if completed_results: print(f"Resuming Monte Carlo experiment with {len(completed_results)} completed realizations.")

    # Get next realization
    next_realization = 1
    while next_realization in completed_results:
        next_realization += 1

    print(f"Monte Carlo settings: minimum {monte_carlo_config.min_realizations}, maximum {monte_carlo_config.max_realizations}, batch size {monte_carlo_config.batch_size}, workers {workers}.")

    return monte_carlo_config, workers, completed_results, next_realization, summary_path


def recover_failed_batch(failures: list[str], consecutive_recoveries: int, monte_carlo_config: MonteCarloConfig, completed_results: dict[int, MonteCarloResult]) -> tuple[int, int]:
    """Clean up failed Docker projects and return the recovery count and next missing realization."""

    consecutive_recoveries += 1
    
    print("One or more realizations exhausted their local retries:", file=sys.stderr)
    for failure in failures: print(f"  {failure}", file=sys.stderr)
    
    if consecutive_recoveries > monte_carlo_config.recovery_cycles:
        raise ValueError(f"Monte Carlo exceeded the configured global recovery limit of {monte_carlo_config.recovery_cycles}.\n" + "\n".join(failures))
    
    print(f"Stopping all MANTA Monte Carlo Docker projects before recovery {consecutive_recoveries}/{monte_carlo_config.recovery_cycles}...", file=sys.stderr)
    cleanup_monte_carlo_projects()
    
    next_realization = 1
    
    while next_realization in completed_results:
        next_realization += 1

    return next_realization


def report_monte_carlo_checkpoint(completed_results: dict[int, MonteCarloResult], monte_carlo_config: MonteCarloConfig, scenario_type: str) -> bool:
    """Evaluate and report convergence at the current realization checkpoint."""

    values, metric_name, unit = get_convergence_values(completed_results, scenario_type)
    converged, sample_mean, half_width, mean_change = check_monte_carlo_convergence(values, monte_carlo_config)
    confidence_percent = 100.0 * monte_carlo_config.confidence_level

    if unit == "%":
        print(f"After {len(values)} realizations: {metric_name} {sample_mean:.2%}, {confidence_percent:.1f}% CI half-width {half_width:.2%}, mean change {mean_change:.2%}.")
    else:
        print(f"After {len(values)} realizations: {metric_name} {sample_mean:.3f} {unit}, {confidence_percent:.1f}% CI half-width {half_width:.3f} {unit}, mean change {mean_change:.3f} {unit}.")

    return converged


def run_batch(sim_config: dict, batch: list[int], workers: int, output_dir: Path, summary_path: Path, project_offset: int, completed_results: dict[int, dict]) -> list[str]:
    """Run one batch of simulations concurrently and save successful results."""

    failures = []
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {
            executor.submit(run_monte_carlo_realization, sim_config, ROOT, simulation, output_dir, project_offset): simulation
            for simulation in batch
        }
    
        for future in as_completed(futures):
            simulation = futures[future]
    
            try:
                row = future.result()
            except InterruptedError:
                continue
            except Exception as error:
                failures.append(f"Realization {simulation} failed: {error}")
                continue
    
            completed_results[simulation] = row
            rows = [completed_results[index] for index in sorted(completed_results)]
            write_monte_carlo_summary(summary_path, rows)

    return failures


def write_monte_carlo_summary(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    # Open file
    with path.open("w", encoding="utf-8", newline="") as file:
        # Write header
        writer = csv.DictWriter(
            file,
            fieldnames=(
                "realization",
                "seed",
                "detected_whales",
                "whale_count",
                "detection_fraction",
                "total_mass",
                "total_energy",
                "mean_vehicle_speed",
                "mean_distance_km",
            ),
        )
        writer.writeheader()

        # Write rows
        for result in sorted(rows, key=lambda item: item.realization): 
            writer.writerow(asdict(result))

# -----------------------------------------------------
# Main monte carlo experiment function
# -----------------------------------------------------

def run_monte_carlo(sim_config: dict, output_dir: Path, project_offset: int=0) -> int:
    scenario_type = sim_config["scenario"]["type"]
    
    # Handle ctrl+c interrupt
    stop_requested.clear()
    signal.signal(signal.SIGINT, handle_interrupt)
    signal.signal(signal.SIGTERM, handle_interrupt)

    monte_carlo_config, workers, completed_results, next_realization, summary_path = prepare_monte_carlo_experiment(sim_config, output_dir)
    if check_saved_result_convergence(completed_results, monte_carlo_config, summary_path, scenario_type): return 0

    consecutive_recoveries = 0
    while len(completed_results) < monte_carlo_config.max_realizations:
        
        # Compute the number of runs needed before the next checkpoint
        runs_needed = compute_runs_to_next_checkpoint(len(completed_results), monte_carlo_config)

        # Get next batch
        batch = get_next_batch(next_realization, runs_needed, completed_results, monte_carlo_config.max_realizations)
        if not batch: break
        print(f"Running {len(batch)} realization(s) to reach checkpoint {len(completed_results) + len(batch)}.")

        # Run the batch
        failures = run_batch(sim_config, batch, workers, output_dir, summary_path, project_offset, completed_results)

        # Handle ctrl+c interrupts
        if stop_requested.is_set():
            print("Monte Carlo stopped.")
            return 130

        # Handle failures
        if failures:
            consecutive_recoveries = recover_failed_batch(failures, consecutive_recoveries, monte_carlo_config, completed_results)
            continue

        # Check for confidence interval convergence
        consecutive_recoveries = 0
        converged = report_monte_carlo_checkpoint(completed_results, monte_carlo_config, scenario_type)
        if converged:
            print(f"Monte Carlo convergence reached after {len(completed_results)} realizations.\nSummary: {summary_path}")
            return 0

        # Get next realization
        next_realization = 1
        while next_realization in completed_results:
            next_realization += 1

    # Monte carlo results after reaching max simulations
    fractions = [completed_results[index].detection_fraction for index in sorted(completed_results)]
    sample_mean, half_width = compute_monte_carlo_statistics(fractions, monte_carlo_config.confidence_level,)
    print(f"Reached maximum of {len(completed_results)} realizations. Mean detection: {sample_mean:.2%}; CI half-width: {half_width:.2%}.\nSummary: {summary_path}")

    return 0