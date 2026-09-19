import csv
import os
import math
import signal
import statistics
import subprocess
import sys

from concurrent.futures import ThreadPoolExecutor, as_completed
from manta_core.manta_dataclasses import MonteCarloConfig, MonteCarloResult
from pathlib import Path
from paths import ROOT, CLEANUP_PATH, MONTE_CARLO_SUMMARY_PATH

from manta_runner_old import stop_requested, run_monte_carlo_realization, write_monte_carlo_summary, cleanup_monte_carlo_projects


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


def check_saved_result_convergence(completed: dict[int, dict], monte_carlo_config: MonteCarloConfig, summary_path: Path) -> bool:
    """Report whether previously completed realizations already satisfy convergence."""

    # Get detection fractions for all monte carlo runs
    detection_fractions = [completed[index]["detection_fraction"] for index in sorted(completed)]
    if not detection_fractions: return False

    # Check if the confidence intervals have converged
    converged, sample_mean, half_width, mean_change = check_monte_carlo_convergence(detection_fractions, monte_carlo_config)
    print(f"Current {len(detection_fractions)} realizations: mean detection {sample_mean:.2%}, CI half-width {half_width:.2%}, mean change {mean_change:.2%}.")

    if converged:
        print(f"Saved results already satisfy convergence after {len(detection_fractions)} realizations.")
        print(f"Summary: {summary_path}")

    return converged


def compute_monte_carlo_statistics(detection_fractions: list[float], confidence_level: float) -> tuple[float, float]:
    """Return the sample mean and normal-approximation confidence-interval half-width."""

    if not detection_fractions: raise ValueError("At least one realization is required.")

    sample_mean = statistics.mean(detection_fractions)
    if len(detection_fractions) < 2: return sample_mean, math.inf

    standard_error = statistics.stdev(detection_fractions) / math.sqrt(len(detection_fractions))
    z_score = statistics.NormalDist().inv_cdf(0.5 + confidence_level / 2.0)
    half_width = z_score * standard_error

    return sample_mean, half_width


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
        required = {"realization", "seed", "detected_whales", "whale_count", "detection_fraction", "total_mass_kg", "total_energy_wh", "mean_vehicle_speed_mps"}
        if not reader.fieldnames or not required.issubset(reader.fieldnames): raise ValueError(f"Monte Carlo summary has an invalid header: {path}")

        results = [MonteCarloResult.from_csv_row(row, base_seed) for row in reader]

    return {result.realization: result for result in sorted(results, key=lambda result: result.realization)}


def prepare_monte_carlo_experiment(sim_config: dict, output_dir: Path) -> tuple[MonteCarloConfig, int, dict[int, dict]]:
    """Load Monte Carlo settings, select workers, and restore completed realizations."""

    # Get worker count
    monte_carlo_config = MonteCarloConfig.from_dict(sim_config.get("monte_carlo", {}))
    workers = get_worker_count(sim_config, monte_carlo_config)

    # Load completed results
    summary_path = output_dir / "summary.csv"
    completed_results = load_completed_results(summary_path, monte_carlo_config.random_seed)
    if completed_results: print(f"Resuming Monte Carlo experiment with {len(completed_results)} completed realizations.")

    # Check if the existing results have already converged
    check_saved_result_convergence(completed_results, monte_carlo_config, summary_path)

    # Get next realization
    next_realization = 1
    while next_realization in completed_results:
        next_realization += 1

    print(f"Monte Carlo settings: minimum {monte_carlo_config.min_realizations}, maximum {monte_carlo_config.max_realizations}, batch size {monte_carlo_config.batch_size}, workers {workers}.")

    return monte_carlo_config, workers, completed_results, next_realization


# -----------------------------------------------------
# Main monte carlo experiment function
# -----------------------------------------------------

def run_monte_carlo(sim_config: dict, output_dir: Path, project_offset: int=0) -> int:
    # Handle ctrl+c interrupt
    stop_requested.clear()
    signal.signal(signal.SIGINT, handle_interrupt)
    signal.signal(signal.SIGTERM, handle_interrupt)

    monte_carlo, workers, completed_results, next_realization = prepare_monte_carlo_experiment(sim_config, output_dir)

    consecutive_recoveries = 0

    while len(completed_results) < monte_carlo.max_realizations:
        remainder = len(completed_results) % monte_carlo.batch_size
        runs_needed = monte_carlo.batch_size if remainder == 0 else monte_carlo.batch_size - remainder
        runs_needed = min(runs_needed, monte_carlo.max_realizations - len(completed_results))

        batch = []
        candidate = next_realization

        while (
            len(batch) < runs_needed
            and candidate <= monte_carlo.max_realizations
        ):
            if candidate not in completed_results:
                batch.append(candidate)
            candidate += 1

        if not batch:
            break

        print(
            f"Running {len(batch)} realization(s) to reach checkpoint "
            f"{len(completed_results) + len(batch)}."
        )

        failures = []
        with ThreadPoolExecutor(max_workers=workers) as executor:
            futures = {
                executor.submit(
                    run_monte_carlo_realization,
                    sim_config,
                    ROOT,
                    realization,
                    output_dir,
                    project_offset,
                ): realization
                for realization in batch
            }

            for future in as_completed(futures):
                realization = futures[future]

                try:
                    row = future.result()
                except InterruptedError:
                    continue
                except Exception as error:
                    failures.append(
                        f"Realization {realization} failed: {error}"
                    )
                    continue

                completed_results[realization] = row
                rows = [
                    completed_results[index]
                    for index in sorted(completed_results)
                ]
                write_monte_carlo_summary(MONTE_CARLO_SUMMARY_PATH, rows)

        if stop_requested.is_set():
            print("Monte Carlo stopped.")
            return 130

        if failures:
            consecutive_recoveries += 1

            print(
                "One or more realizations exhausted their local retries:",
                file=sys.stderr,
            )

            for failure in failures:
                print(f"  {failure}", file=sys.stderr)

            if consecutive_recoveries > monte_carlo.recovery_cycles:
                raise ValueError(
                    "Monte Carlo exceeded the configured global recovery "
                    f"limit of {monte_carlo.recovery_cycles}.\n"
                    + "\n".join(failures)
                )

            print(
                "Stopping all MANTA Monte Carlo Docker projects before "
                f"recovery {consecutive_recoveries}/{monte_carlo.recovery_cycles}...",
                file=sys.stderr,
            )
            cleanup_monte_carlo_projects(ROOT)

            next_realization = 1

            while next_realization in completed_results:
                next_realization += 1

            continue

        consecutive_recoveries = 0

        fractions = [
            completed_results[index]["detection_fraction"]
            for index in sorted(completed_results)
        ]

        converged, sample_mean, half_width, mean_change = (
            check_monte_carlo_convergence(
                fractions,
                monte_carlo,
            )
        )

        confidence_percent = 100.0 * monte_carlo.confidence_level

        print(
            f"After {len(fractions)} realizations: "
            f"mean detection {sample_mean:.2%}, "
            f"{confidence_percent:.1f}% CI half-width "
            f"{half_width:.2%}, "
            f"mean change {mean_change:.2%}."
        )

        if converged:
            print(
                f"Monte Carlo convergence reached after "
                f"{len(fractions)} realizations."
            )
            print(f"Summary: {MONTE_CARLO_SUMMARY_PATH}")
            return 0

        next_realization = 1

        while next_realization in completed_results:
            next_realization += 1

    fractions = [
        completed_results[index]["detection_fraction"]
        for index in sorted(completed_results)
    ]

    sample_mean, half_width = compute_monte_carlo_statistics(
        fractions,
        monte_carlo.confidence_level,
    )

    print(
        f"Reached maximum of {len(fractions)} realizations. "
        f"Mean detection: {sample_mean:.2%}; "
        f"CI half-width: {half_width:.2%}."
    )
    print(f"Summary: {MONTE_CARLO_SUMMARY_PATH}")

    return 0