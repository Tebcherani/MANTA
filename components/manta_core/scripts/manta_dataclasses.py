from dataclasses import dataclass


@dataclass(frozen=True)
class MonteCarloConfig:
    """Settings controlling a repeated Monte Carlo simulation experiment."""

    min_realizations: int
    max_realizations: int
    batch_size: int
    random_seed: int
    confidence_level: float
    target_half_width: float
    stability_tolerance: float
    workers: int | None = None
    realization_timeout: int = 60
    max_retries: int = 2
    recovery_cycles: int = 3

    def __post_init__(self) -> None:
        """Validate Monte Carlo settings immediately after loading."""

        if self.min_realizations < 2:
            raise ValueError("'monte_carlo.min_realizations' must be at least 2.")
        if self.max_realizations < self.min_realizations:
            raise ValueError("'monte_carlo.max_realizations' must be greater than or equal to min_realizations.")
        if self.batch_size < 1:
            raise ValueError("'monte_carlo.batch_size' must be at least 1.")
        if self.workers is not None and self.workers < 1:
            raise ValueError("'monte_carlo.workers' must be a positive integer.")
        if self.realization_timeout < 1:
            raise ValueError("'monte_carlo.realization_timeout' must be a positive integer.")
        if self.max_retries < 0:
            raise ValueError("'monte_carlo.max_retries' cannot be negative.")
        if not 0.0 < self.confidence_level < 1.0:
            raise ValueError("'monte_carlo.confidence_level' must be between 0 and 1.")
        if self.target_half_width <= 0.0:
            raise ValueError("'monte_carlo.target_half_width' must be greater than zero.")
        if self.stability_tolerance < 0.0:
            raise ValueError("'monte_carlo.stability_tolerance' cannot be negative.")
        if self.recovery_cycles < 0:
            raise ValueError("'monte_carlo.recovery_cycles' cannot be negative.")

    @classmethod
    def from_dict(cls, data: dict) -> "MonteCarloConfig":
        """Convert the YAML monte_carlo mapping into validated settings."""

        if not isinstance(data, dict):
            raise ValueError("'monte_carlo' must be a YAML mapping.")

        return cls(
            min_realizations = int(data["min_realizations"]),
            max_realizations = int(data["max_realizations"]),
            batch_size = int(data["batch_size"]),
            random_seed = int(data["random_seed"]),
            confidence_level = float(data["confidence_level"]),
            target_half_width = float(data["target_half_width"]),
            stability_tolerance = float(data["stability_tolerance"]),
            workers = int(data["workers"]) if data.get("workers") is not None else None,
            realization_timeout = int(data.get("realization_timeout", 60)),
            max_retries = int(data.get("max_retries", 2)),
            recovery_cycles=int(data.get("recovery_cycles", 3)),
        )


@dataclass(frozen=True)
class MonteCarloResult:
    """Results from one Monte Carlo realization."""

    realization: int
    seed: int
    total_mass: float
    total_energy: float
    mean_vehicle_speed: float
    mean_distance_km: float | None = None

    @classmethod
    def from_csv_row(cls, row: dict[str, str], base_seed: int) -> "MonteCarloResult":
        """Convert and validate one Monte Carlo summary row."""

        realization = int(row["realization"])
        seed = int(row["seed"])
        expected_seed = base_seed + realization - 1

        if seed != expected_seed:
            raise ValueError(f"Existing realization {realization} uses seed {seed}, but the current configuration expects {expected_seed}.")

        return cls(
            realization=realization,
            seed=seed,
            total_mass=float(row["total_mass"]),
            total_energy=float(row["total_energy"]),
            mean_vehicle_speed=float(row["mean_vehicle_speed"]),
            mean_distance_km=float(row["mean_distance_km"]) if row.get("mean_distance_km", "").strip() else None,
        )


@dataclass(frozen=True)
class TrailingSweepConfig:
    """Whale-call waypoint-update intervals evaluated by the trailing sweep."""

    call_update_intervals: tuple[float, ...]

    def __post_init__(self) -> None:
        """Validate the configured call-update intervals."""

        if not self.call_update_intervals:
            raise ValueError("'sweep.call_update_intervals' must contain at least one value.")
        if any(interval < 0.0 for interval in self.call_update_intervals):
            raise ValueError("'sweep.call_update_intervals' cannot contain negative values.")
        if len(self.call_update_intervals) != len(set(self.call_update_intervals)):
            raise ValueError("'sweep.call_update_intervals' cannot contain duplicate values.")
        if tuple(sorted(self.call_update_intervals)) != self.call_update_intervals:
            raise ValueError("'sweep.call_update_intervals' must be ordered from smallest to largest.")

    @classmethod
    def from_dict(cls, data: dict) -> "TrailingSweepConfig":
        """Convert the YAML trailing-sweep mapping into validated settings."""

        if not isinstance(data, dict):
            raise ValueError("'sweep' must be a YAML mapping.")

        intervals = data.get("communication_intervals")

        if not isinstance(intervals, list):
            raise ValueError("'sweep.communication_intervals' must be a YAML list.")
        
        return cls(call_update_intervals=tuple(float(interval) for interval in intervals))


@dataclass(frozen=True)
class TrailingSweepResult:
    """Aggregate tracking results for one call-update interval."""

    call_update_interval: float
    realizations: int
    mean_distance_km: float
    distance_ci_lower_km: float
    distance_ci_upper_km: float
    distance_ci_half_width_km: float
    confidence_level: float

    def __post_init__(self) -> None:
        """Validate a completed trailing-sweep result."""

        if self.call_update_interval < 0.0:
            raise ValueError("Call-update interval cannot be negative.")
        if self.realizations < 1:
            raise ValueError("Trailing sweep result must contain at least one realization.")
        if self.mean_distance_km < 0.0:
            raise ValueError("Mean distance cannot be negative.")
        if not 0.0 < self.confidence_level < 1.0:
            raise ValueError("Confidence level must be between zero and one.")

    @classmethod
    def from_csv_row(cls, row: dict[str, str]) -> "TrailingSweepResult":
        """Convert one trailing-sweep summary row into a typed result."""

        return cls(
            call_update_interval=float(row["call_update_interval"]),
            realizations=int(row["realizations"]),
            mean_distance_km=float(row["mean_distance_km"]),
            distance_ci_lower_km=float(row["distance_ci_lower_km"]),
            distance_ci_upper_km=float(row["distance_ci_upper_km"]),
            distance_ci_half_width_km=float(row["distance_ci_half_width_km"]),
            confidence_level=float(row["confidence_level"]),
        )


@dataclass(frozen=True)
class SweepConfig:
    """Fleet sizes evaluated by the Monte Carlo sweep."""

    min_vehicles: int
    max_vehicles: int

    def __post_init__(self) -> None:
        if self.min_vehicles < 1:
            raise ValueError("'sweep.min_vehicles' must be at least 1.")
        if self.max_vehicles < self.min_vehicles:
            raise ValueError("'sweep.max_vehicles' must be greater than or equal to min_vehicles.")

    @classmethod
    def from_dict(cls, data: dict) -> "SweepConfig":
        """Convert the YAML sweep mapping into validated settings."""

        if not isinstance(data, dict):
            raise ValueError("'sweep' must be a YAML mapping.")

        return cls(min_vehicles=int(data.get("min_vehicles", 1)), max_vehicles=int(data.get("max_vehicles", 10)))