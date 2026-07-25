from dataclasses import dataclass
from typing import Protocol, Sequence, Tuple

import numpy as np


class PathConstraint(Protocol):
    name: str

    def evaluate(
        self,
        points: np.ndarray,
        normalized_distance: np.ndarray,
    ) -> Tuple[float, float]:
        """返回（适应度罚分，最大超限量）。"""


@dataclass(frozen=True)
class ForbiddenPolygonConstraint:
    vertices: np.ndarray
    weight: float = 1e6
    name: str = "禁止多边形区域"

    def evaluate(self, points, normalized_distance):
        vertices = np.asarray(self.vertices, dtype=float)
        if vertices.ndim != 2 or vertices.shape[1] != 2 or len(vertices) < 3:
            raise ValueError("禁止多边形区域至少需要三个顶点")
        inside = _points_in_polygon(np.asarray(points, dtype=float), vertices)
        violation = float(np.mean(inside))
        return self.weight * violation, violation


@dataclass(frozen=True)
class ParallelBandConstraint:
    normal: np.ndarray
    lower: float
    upper: float
    start_fraction: float = 0.0
    end_fraction: float = 1.0
    weight: float = 1e5
    name: str = "平行线限制带"

    def evaluate(self, points, normalized_distance):
        normal = np.asarray(self.normal, dtype=float)
        norm = np.linalg.norm(normal)
        if norm <= 1e-12 or self.lower >= self.upper:
            raise ValueError("平行线限制带的几何参数无效")
        normal /= norm
        mask = (
            (normalized_distance >= self.start_fraction)
            & (normalized_distance <= self.end_fraction)
        )
        if not np.any(mask):
            return self.weight, 1.0
        projected = np.asarray(points, dtype=float)[mask] @ normal
        violation = np.maximum(self.lower - projected, 0.0)
        violation += np.maximum(projected - self.upper, 0.0)
        maximum = float(np.max(violation))
        return self.weight * float(np.mean(violation**2)), maximum


@dataclass(frozen=True)
class DirectedRectanglePassageConstraint:
    """要求路径从矩形两个短边的中部进入和离开，避免横穿长边。"""

    short_edge_center_a: np.ndarray
    short_edge_center_b: np.ndarray
    half_width: float
    center_fraction: float = 0.5
    weight: float = 1e5
    name: str = "定向矩形通过区"

    def evaluate(self, points, normalized_distance):
        points = np.asarray(points, dtype=float)
        start = np.asarray(self.short_edge_center_a, dtype=float)
        end = np.asarray(self.short_edge_center_b, dtype=float)
        axis = end - start
        length = np.linalg.norm(axis)
        if length <= 1e-12 or self.half_width <= 0:
            raise ValueError("定向矩形通过区的几何参数无效")
        axis /= length
        normal = np.array([-axis[1], axis[0]])
        local = points - start
        longitudinal = local @ axis
        lateral = local @ normal

        entry_error = max(np.min(np.linalg.norm(points - start, axis=1)) - self.half_width, 0.0)
        exit_error = max(np.min(np.linalg.norm(points - end, axis=1)) - self.half_width, 0.0)
        slab = (longitudinal >= 0.0) & (longitudinal <= length)
        if not np.any(slab):
            return self.weight, length

        allowed_lateral = self.half_width * self.center_fraction
        side_error = np.maximum(np.abs(lateral[slab]) - allowed_lateral, 0.0)
        maximum = float(max(entry_error, exit_error, np.max(side_error)))
        penalty = entry_error**2 + exit_error**2 + float(np.mean(side_error**2))
        return self.weight * penalty, maximum

    def polygon(self) -> np.ndarray:
        start = np.asarray(self.short_edge_center_a, dtype=float)
        end = np.asarray(self.short_edge_center_b, dtype=float)
        axis = end - start
        axis /= np.linalg.norm(axis)
        normal = np.array([-axis[1], axis[0]]) * self.half_width
        return np.array([start + normal, end + normal, end - normal, start - normal])


def evaluate_constraints(
    constraints: Sequence[PathConstraint],
    points: np.ndarray,
    normalized_distance: np.ndarray,
) -> Tuple[float, float]:
    total_penalty = 0.0
    maximum_violation = 0.0
    for constraint in constraints:
        penalty, violation = constraint.evaluate(points, normalized_distance)
        total_penalty += penalty
        maximum_violation = max(maximum_violation, violation)
    return total_penalty, maximum_violation


def _points_in_polygon(points: np.ndarray, vertices: np.ndarray) -> np.ndarray:
    x = points[:, 0]
    y = points[:, 1]
    inside = np.zeros(len(points), dtype=bool)
    x1, y1 = vertices[-1]
    for x2, y2 in vertices:
        crosses = (y1 > y) != (y2 > y)
        x_intersection = (x2 - x1) * (y - y1) / (y2 - y1 + 1e-300) + x1
        inside ^= crosses & (x < x_intersection)
        x1, y1 = x2, y2
    return inside
