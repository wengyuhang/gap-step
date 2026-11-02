"""Numerical aperture erosion for initialization, not a safety certificate."""
from dataclasses import dataclass

import numpy as np
from shapely.geometry import Polygon
from shapely.ops import polylabel


@dataclass(frozen=True)
class ComponentSeed:
    index: int
    region: Polygon
    point: np.ndarray
    clearance: float

    def record(self):
        return dict(index=self.index, point=self.point.tolist(),
                    area_m2=float(self.region.area), bounds=list(self.region.bounds),
                    original_boundary_distance_m=self.clearance,
                    eroded_region_geojson=self.region.__geo_interface__)


def component_seeds(vertices, radius, *, quad_segs=64, tolerance=1e-6):
    """Round Euclidean inward offset; one pole-of-inaccessibility per component.

    GEOS approximates offset arcs by chords. It is used only to seed the
    unchanged true finite-edge constraints, not to certify dynamic passage.
    No component is discarded by area or selected by a left/right label.
    """
    if not np.isfinite(radius) or radius < 0 or tolerance <= 0:
        raise ValueError('Nonnegative finite radius and positive tolerance required')
    if quad_segs < 1:
        raise ValueError('quad_segs must be positive')
    polygon = Polygon(np.asarray(vertices, float))
    if not polygon.is_valid or polygon.area <= 0 or polygon.interiors:
        raise ValueError('Expected a valid simple polygon without holes')
    eroded = polygon.buffer(-radius, quad_segs=quad_segs, join_style='round')
    if eroded.is_empty:
        return []
    regions = [eroded] if isinstance(eroded, Polygon) else list(eroded.geoms)
    regions.sort(key=lambda p: tuple(p.bounds))  # Deterministic geometry-only order.
    seeds = []
    for i, region in enumerate(regions):
        point = polylabel(region, tolerance=tolerance)
        clearance = float(polygon.boundary.distance(point))
        if not region.covers(point) or not polygon.covers(point) or clearance < radius - 1e-9:
            raise RuntimeError('Numerical erosion produced an invalid representative')
        seeds.append(ComponentSeed(i, region, np.asarray(point.coords[0]), clearance))
    return seeds
