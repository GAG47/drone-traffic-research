"""SI parameters: official supplement Table S2; engineering choices marked below."""

from dataclasses import dataclass, fields
import math


@dataclass(frozen=True, slots=True)
class TrafficParameters:
    # S2 simulation column, supplement p. 3.
    cruise_speed: float = 8.0
    repulsion_radius: float = 10.5
    repulsion_gain: float = 1.3
    anisotropy: float = 0.5  # Main text p. 35 instead says 0.42.
    friction_radius: float = 10.7
    friction_slack: float = 0.5
    friction_slope: float = 1.0
    friction_acceleration: float = 3.0
    avoidance_radius: float = 13.0
    avoidance_slope: float = 0.5
    avoidance_acceleration: float = 3.0
    # Explicit engineering assumptions, NOT S2 values.
    friction_gain: float = 1.0
    max_iterations: int = 32
    tolerance: float = 1e-9
    arrival_radius: float = 0.5
    arrival_speed: float = 0.2

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{field.name} must be finite and nonnegative")
        for name in ("cruise_speed", "friction_slope", "friction_acceleration",
                     "avoidance_radius", "avoidance_slope", "avoidance_acceleration",
                     "tolerance", "arrival_radius", "arrival_speed"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if self.anisotropy > 1:
            raise ValueError("require 0 <= anisotropy <= 1")
        if isinstance(self.max_iterations, bool) or not isinstance(self.max_iterations, int) or self.max_iterations < 1:
            raise ValueError("max_iterations must be a positive integer")
