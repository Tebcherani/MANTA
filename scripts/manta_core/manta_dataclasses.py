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
    detected_whales: int
    whale_count: int
    detection_fraction: float
    total_mass: float
    total_energy: float
    mean_vehicle_speed: float

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
            detected_whales=int(row["detected_whales"]),
            whale_count=int(row["whale_count"]),
            detection_fraction=float(row["detection_fraction"]),
            total_mass=float(row["total_mass_kg"]),
            total_energy=float(row["total_energy_wh"]),
            mean_vehicle_speed=float(row["mean_vehicle_speed_mps"]),
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


@dataclass(frozen=True)
class SweepResult:
    """Aggregate Monte Carlo results for one fleet size."""

    fleet_size: int
    realizations: int
    whale_count: int
    mean_detected_whales: float
    mean_detection: float
    ci_lower: float
    ci_upper: float
    ci_half_width: float
    confidence_level: float
    mean_total_mass: float
    mass_ci_lower: float
    mass_ci_upper: float
    mass_ci_half_width: float
    mean_total_energy: float
    energy_ci_lower: float
    energy_ci_upper: float
    energy_ci_half_width: float
    mean_vehicle_speed: float
    speed_ci_lower: float
    speed_ci_upper: float
    speed_ci_half_width: float

    def __post_init__(self) -> None:
        """Validate a completed fleet-size result."""

        if self.fleet_size < 1:
            raise ValueError("Sweep-result fleet size must be at least 1.")
        if self.realizations < 1:
            raise ValueError("Sweep result must contain at least one realization.")
        if self.whale_count < 1:
            raise ValueError("Sweep-result whale count must be at least 1.")
        if not 0.0 <= self.mean_detection <= 1.0:
            raise ValueError("Mean detection must be between 0 and 1.")
        if not 0.0 <= self.ci_lower <= self.ci_upper <= 1.0:
            raise ValueError("Detection confidence interval must remain between 0 and 1.")
        if not 0.0 < self.confidence_level < 1.0:
            raise ValueError("Confidence level must be between 0 and 1.")

    @classmethod
    def from_csv_row(cls, row: dict[str, str]) -> "SweepResult":
        """Convert one sweep-summary CSV row into a typed result."""

        return cls(
            fleet_size=int(row["fleet_size"]),
            realizations=int(row["realizations"]),
            whale_count=int(row["whale_count"]),
            mean_detected_whales=float(row["mean_detected_whales"]),
            mean_detection=float(row["mean_detection"]),
            ci_lower=float(row["ci_lower"]),
            ci_upper=float(row["ci_upper"]),
            ci_half_width=float(row["ci_half_width"]),
            confidence_level=float(row["confidence_level"]),
            mean_total_mass=float(row["mean_total_mass"]),
            mass_ci_lower=float(row["mass_ci_lower"]),
            mass_ci_upper=float(row["mass_ci_upper"]),
            mass_ci_half_width=float(row["mass_ci_half_width"]),
            mean_total_energy=float(row["mean_total_energy"]),
            energy_ci_lower=float(row["energy_ci_lower"]),
            energy_ci_upper=float(row["energy_ci_upper"]),
            energy_ci_half_width=float(row["energy_ci_half_width"]),
            mean_vehicle_speed=float(row["mean_vehicle_speed"]),
            speed_ci_lower=float(row["speed_ci_lower"]),
            speed_ci_upper=float(row["speed_ci_upper"]),
            speed_ci_half_width=float(row["speed_ci_half_width"]),
        )