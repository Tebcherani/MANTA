import csv
import os
import math
import signal
import statistics
import subprocess
import sys

from concurrent.futures import ThreadPoolExecutor, as_completed
from manta_core.manta_dataclasses import MonteCarloConfig
from pathlib import Path
from paths import ROOT, CLEANUP_PATH, MONTE_CARLO_SUMMARY_PATH

from manta_runner_old import stop_requested, run_monte_carlo_realization, write_monte_carlo_summary, cleanup_monte_carlo_projects


def monte_carlo_convergence(
    fractions: list[float],
    batch_size: int,
    min_realizations: int,
    confidence_level: float,
    target_half_width: float,
    stability_tolerance: float,
) -> tuple[bool, float, float, float]:
    """Calculate convergence using the current and preceding cumulative means."""

    if not fractions:
        return False, math.nan, math.inf, math.inf

    sample_mean, half_width = calculate_monte_carlo_statistics(
        fractions,
        confidence_level,
    )

    previous_count = len(fractions) - batch_size
    previous_mean = (
        statistics.mean(fractions[:previous_count])
        if previous_count > 0
        else None
    )
    mean_change = (
        abs(sample_mean - previous_mean)
        if previous_mean is not None
        else math.inf
    )

    converged = (
        len(fractions) >= min_realizations
        and half_width <= target_half_width
        and mean_change <= stability_tolerance
    )

    return converged, sample_mean, half_width, mean_change


def calculate_monte_carlo_statistics(detection_fractions: list[float], confidence_level: float) -> tuple[float, float]:
    """Return the sample mean and normal-approximation confidence-interval half-width."""

    if not detection_fractions: raise ValueError("At least one realization is required.")

    sample_mean = statistics.mean(detection_fractions)
    if len(detection_fractions) < 2: return sample_mean, math.inf

    standard_error = statistics.stdev(detection_fractions) / math.sqrt(len(detection_fractions))
    z_score = statistics.NormalDist().inv_cdf(0.5 + confidence_level / 2.0)
    half_width = z_score * standard_error

    return sample_mean, half_width

def calculate_worker_count(config: dict, monte_carlo: MonteCarloConfig) -> int:
    """Choose a safe number of parallel realizations for the fleet size."""

    cpu_count = os.cpu_count() or 1
    fleet_size = len(config["vehicles"])
    default_workers = max(1, cpu_count // max(1, fleet_size))
    requested_workers = monte_carlo.workers if monte_carlo.workers is not None else default_workers

    return max(1, min(requested_workers, monte_carlo.batch_size, default_workers, 200))


def handle_interrupt(signal_number, frame) -> None:
    """Stop every active Monte Carlo project and suppress retries."""

    if stop_requested.is_set(): return
    print("\nStopping MANTA Monte Carlo...", file=sys.stderr)
    stop_requested.set()
    subprocess.run(["bash", str(CLEANUP_PATH)], check=False)


def load_monte_carlo_summary(path: Path, base_seed: int) -> list[dict]:
    if not path.is_file():
        return []

    rows = []

    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)

        required = {
            "realization",
            "seed",
            "detected_whales",
            "whale_count",
            "detection_fraction",
            "total_mass_kg",
            "total_energy_wh",
            "mean_vehicle_speed_mps",
        }

        if not reader.fieldnames or not required.issubset(
            reader.fieldnames
        ):
            raise ValueError(
                f"Monte Carlo summary has an invalid header: {path}"
            )

        for row in reader:
            realization = int(row["realization"])
            seed = int(row["seed"])
            expected_seed = base_seed + realization - 1

            if seed != expected_seed:
                raise ValueError(
                    f"Existing realization {realization} uses seed {seed}, "
                    f"but the current configuration expects {expected_seed}."
                )

            rows.append(
                {
                    "realization": realization,
                    "seed": seed,
                    "detected_whales": int(row["detected_whales"]),
                    "whale_count": int(row["whale_count"]),
                    "detection_fraction": float(
                        row["detection_fraction"]
                    ),
                    "total_mass_kg": float(row["total_mass_kg"]),
                    "total_energy_wh": float(row["total_energy_wh"]),
                    "mean_vehicle_speed_mps": float(
                        row["mean_vehicle_speed_mps"]
                    ),
                }
            )

    rows.sort(key=lambda row: row["realization"])
    return rows


def run_monte_carlo(config: dict, output_directory: Path, project_offset: int=0) -> int:
    stop_requested.clear()
    signal.signal(signal.SIGINT, handle_interrupt)
    signal.signal(signal.SIGTERM, handle_interrupt)

    monte_carlo = MonteCarloConfig.from_dict(config.get("monte_carlo", {}))
    workers = calculate_worker_count(config, monte_carlo)

    rows = load_monte_carlo_summary(MONTE_CARLO_SUMMARY_PATH, monte_carlo.random_seed)

    completed = {row["realization"]: row for row in rows}
    if rows: print(f"Resuming Monte Carlo experiment with {len(rows)} completed realizations.")

    print(
        f"Monte Carlo settings: minimum {monte_carlo.min_realizations}, "
        f"maximum {monte_carlo.max_realizations}, batch size {monte_carlo.batch_size}, "
        f"workers {workers}."
    )

    fractions = [completed[index]["detection_fraction"] for index in sorted(completed)]

    if fractions:
        converged, sample_mean, half_width, mean_change = (
            monte_carlo_convergence(
                fractions,
                monte_carlo.batch_size,
                monte_carlo.min_realizations,
                monte_carlo.confidence_level,
                monte_carlo.target_half_width,
                monte_carlo.stability_tolerance,
            )
        )

        print(
            f"Current {len(fractions)} realizations: mean detection "
            f"{sample_mean:.2%}, CI half-width {half_width:.2%}, "
            f"mean change {mean_change:.2%}."
        )

        if converged:
            print(
                f"Saved results already satisfy convergence after "
                f"{len(fractions)} realizations."
            )
            print(f"Summary: {MONTE_CARLO_SUMMARY_PATH}")
            return 0

    next_realization = 1

    while next_realization in completed:
        next_realization += 1

    consecutive_recoveries = 0

    while len(completed) < monte_carlo.max_realizations:
        remainder = len(completed) % monte_carlo.batch_size
        runs_needed = monte_carlo.batch_size if remainder == 0 else monte_carlo.batch_size - remainder
        runs_needed = min(runs_needed, monte_carlo.max_realizations - len(completed))

        batch = []
        candidate = next_realization

        while (
            len(batch) < runs_needed
            and candidate <= monte_carlo.max_realizations
        ):
            if candidate not in completed:
                batch.append(candidate)
            candidate += 1

        if not batch:
            break

        print(
            f"Running {len(batch)} realization(s) to reach checkpoint "
            f"{len(completed) + len(batch)}."
        )

        failures = []
        with ThreadPoolExecutor(max_workers=workers) as executor:
            futures = {
                executor.submit(
                    run_monte_carlo_realization,
                    config,
                    ROOT,
                    realization,
                    output_directory,
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

                completed[realization] = row
                rows = [
                    completed[index]
                    for index in sorted(completed)
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

            while next_realization in completed:
                next_realization += 1

            continue

        consecutive_recoveries = 0

        fractions = [
            completed[index]["detection_fraction"]
            for index in sorted(completed)
        ]

        converged, sample_mean, half_width, mean_change = (
            monte_carlo_convergence(
                fractions,
                monte_carlo.batch_size,
                monte_carlo.min_realizations,
                monte_carlo.confidence_level,
                monte_carlo.target_half_width,
                monte_carlo.stability_tolerance,
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

        while next_realization in completed:
            next_realization += 1

    fractions = [
        completed[index]["detection_fraction"]
        for index in sorted(completed)
    ]

    sample_mean, half_width = calculate_monte_carlo_statistics(
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