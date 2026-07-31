"""Five hole-free non-convex gate families."""

from __future__ import annotations

from typing import Any

import numpy as np
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

from .dynamic_gate import DynamicGate, SplineSeries
from .geometry import polygonal_parts, validate_simple_polygon


class LShapeGate(DynamicGate):
    shape_kind = "l_shape"

    def _unit_polygon(self, deformation: float) -> Polygon:
        width = 2.6
        leg = float(np.clip(0.80 * (1.0 + 0.35 * deformation), 0.45, 1.15))
        horizontal = box(-width / 2, -width / 2, width / 2, -width / 2 + leg)
        vertical = box(-width / 2, -width / 2, -width / 2 + leg, width / 2)
        result = unary_union((horizontal, vertical))
        assert isinstance(result, Polygon)
        return result


class UShapeGate(DynamicGate):
    shape_kind = "u_shape"

    def _unit_polygon(self, deformation: float) -> Polygon:
        width, height = 2.8, 2.6
        leg = float(np.clip(0.62 * (1.0 + 0.30 * deformation), 0.42, 0.90))
        bottom = float(np.clip(0.68 * (1.0 - 0.25 * deformation), 0.45, 0.92))
        pieces = (
            box(-width / 2, -height / 2, width / 2, -height / 2 + bottom),
            box(-width / 2, -height / 2, -width / 2 + leg, height / 2),
            box(width / 2 - leg, -height / 2, width / 2, height / 2),
        )
        result = unary_union(pieces)
        assert isinstance(result, Polygon)
        return result


class StarGate(DynamicGate):
    shape_kind = "star"

    def _unit_polygon(self, deformation: float) -> Polygon:
        point_count = 5
        outer = 1.45
        inner = float(np.clip(0.72 * (1.0 + 0.35 * deformation), 0.45, 0.98))
        angles = np.linspace(0.0, 2.0 * np.pi, 2 * point_count, endpoint=False)
        radii = np.where(np.arange(2 * point_count) % 2 == 0, outer, inner)
        vertices = np.column_stack((radii * np.cos(angles), radii * np.sin(angles)))
        return Polygon(vertices)


class CrescentGate(DynamicGate):
    shape_kind = "crescent"

    def _unit_polygon(self, deformation: float) -> Polygon:
        resolution = max(32, self.boundary_samples // 4)
        outer_radius = 1.48
        inner_radius = float(np.clip(1.02 * (1.0 + 0.18 * deformation), 0.86, 1.18))
        offset = float(np.clip(0.66 * (1.0 - 0.20 * deformation), 0.52, 0.82))
        outer = Point(0.0, 0.0).buffer(outer_radius, quad_segs=resolution)
        cut = Point(offset, 0.0).buffer(inner_radius, quad_segs=resolution)
        result = outer.difference(cut)
        parts = polygonal_parts(result)
        if len(parts) != 1:
            raise ValueError("crescent parameters did not produce one simple component")
        validate_simple_polygon(parts[0])
        return parts[0]


class WaveGate(DynamicGate):
    shape_kind = "wave"

    def _unit_polygon(self, deformation: float) -> Polygon:
        count = max(32, self.boundary_samples // 2)
        x = np.linspace(-1.45, 1.45, count)
        amplitude = float(np.clip(0.30 * (1.0 + 0.45 * deformation), 0.14, 0.48))
        phase = 0.35 * deformation
        top = 1.02 + amplitude * np.sin(2.0 * np.pi * x / 2.9 + phase)
        bottom = -1.02 + 0.75 * amplitude * np.sin(
            2.0 * np.pi * x / 2.9 + phase + 1.15
        )
        vertices = np.vstack(
            (np.column_stack((x, bottom)), np.column_stack((x[::-1], top[::-1])))
        )
        return Polygon(vertices)


GATE_TYPES: dict[str, type[DynamicGate]] = {
    cls.shape_kind: cls
    for cls in (LShapeGate, UShapeGate, StarGate, CrescentGate, WaveGate)
}


def gate_from_dict(data: dict[str, Any]) -> DynamicGate:
    kind = str(data["shape_kind"])
    try:
        gate_type = GATE_TYPES[kind]
    except KeyError as exc:
        raise ValueError(f"unknown gate shape {kind!r}") from exc
    return gate_type(
        gate_id=int(data["gate_id"]),
        name=str(data["name"]),
        center_profile=SplineSeries.from_dict(data["center_profile"]),
        rpy_profile=SplineSeries.from_dict(data["rpy_profile"]),
        scale_profile=SplineSeries.from_dict(data["scale_profile"]),
        deformation_profile=SplineSeries.from_dict(data["deformation_profile"]),
        boundary_samples=int(data.get("boundary_samples", 256)),
    )


__all__ = [
    "CrescentGate",
    "GATE_TYPES",
    "LShapeGate",
    "StarGate",
    "UShapeGate",
    "WaveGate",
    "gate_from_dict",
]
