"""Terrain-preview contract shared by truth, noisy and perception inputs."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TerrainPreview:
    leading_edge_m: float
    trailing_edge_m: float
    lateral_min_m: float
    lateral_max_m: float
    depth_m: float
    friction: float
    confidence: float = 1.0
    timestamp_s: float = 0.0
    source: str = "ground_truth"

    @classmethod
    def from_scenario(cls, scenario, timestamp_s: float = 0.0) -> "TerrainPreview":
        return cls(
            leading_edge_m=float(scenario.leading_edge_m),
            trailing_edge_m=float(scenario.trailing_edge_m),
            lateral_min_m=float(scenario.lateral_min_m),
            lateral_max_m=float(scenario.lateral_max_m),
            depth_m=float(scenario.depth_m),
            friction=float(scenario.friction),
            confidence=1.0,
            timestamp_s=float(timestamp_s),
            source="ground_truth",
        )

    def distance_to_entry(self, wheel_station_m: float) -> float:
        return self.leading_edge_m - float(wheel_station_m)

    def contains_wheel(self, station_m: float, lateral_m: float) -> bool:
        return (
            self.leading_edge_m <= float(station_m) <= self.trailing_edge_m
            and self.lateral_min_m <= float(lateral_m) <= self.lateral_max_m
        )

    def as_dict(self) -> dict:
        return {
            "leading_edge_m": self.leading_edge_m,
            "trailing_edge_m": self.trailing_edge_m,
            "lateral_min_m": self.lateral_min_m,
            "lateral_max_m": self.lateral_max_m,
            "depth_m": self.depth_m,
            "friction": self.friction,
            "confidence": self.confidence,
            "timestamp_s": self.timestamp_s,
            "source": self.source,
        }
