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