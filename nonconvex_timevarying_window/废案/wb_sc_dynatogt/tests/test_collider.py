from __future__ import annotations

import numpy as np
from shapely.geometry import MultiPoint, Point, Polygon

from nonconvex_timevarying_window.wb_sc_dynatogt.collider import ColliderConfig, CuboidCollider


def _projection_inside(points: np.ndarray, polygon: Polygon, margin: float) -> bool:
    return polygon.buffer(-margin, join_style="mitre").covers(MultiPoint(points).convex_hull)


def test_default_cuboid_dimensions_and_manifest() -> None:
    collider = CuboidCollider()
    assert collider.corners.shape == (8, 3)
    assert collider.config.geometric_margin == 0.005
    assert collider.config.numerical_margin == 0.010
    assert np.isclose(collider.maximum_radius, 0.300, atol=1.0e-12)
    manifest = collider.manifest()
    assert manifest["model"] == "oriented_cuboid"
    assert manifest["corner_count"] == 8
    assert collider.edge_points().shape == (92, 3)
    assert manifest["hard_constraint_edge_points"] == 92


def test_rotation_changes_projected_cuboid_extent() -> None:
    points = CuboidCollider().corners
    identity_extent = np.ptp(points[:, :2], axis=0)
    angle = np.pi / 2.0
    rotation_y = np.array(
        [[np.cos(angle), 0.0, np.sin(angle)], [0.0, 1.0, 0.0], [-np.sin(angle), 0.0, np.cos(angle)]]
    )
    tilted_extent = np.ptp((points @ rotation_y.T)[:, :2], axis=0)
    assert tilted_extent[0] < identity_extent[0]


def test_legacy_sphere_radius_remains_0315() -> None:
    assert ColliderConfig().legacy_sphere_radius == 0.315


def test_sphere_and_every_yaw_infeasible_but_pitch_rotated_cuboid_feasible() -> None:
    aperture = Polygon([(-0.085, -0.30), (0.085, -0.30), (0.085, 0.30), (-0.085, 0.30)])
    collider = CuboidCollider()
    yaw_only_feasible = []
    for yaw in np.linspace(-np.pi, np.pi, 361):
        rotation_z = np.array(
            [
                [np.cos(yaw), -np.sin(yaw), 0.0],
                [np.sin(yaw), np.cos(yaw), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        yaw_only_feasible.append(
            _projection_inside(
                (collider.corners @ rotation_z.T)[:, :2],
                aperture,
                collider.config.clearance,
            )
        )
    angle = np.pi / 2.0
    rotation_y = np.array(
        [[np.cos(angle), 0.0, np.sin(angle)], [0.0, 1.0, 0.0], [-np.sin(angle), 0.0, np.cos(angle)]]
    )
    projected = (collider.corners @ rotation_y.T)[:, :2]
    assert aperture.boundary.distance(Point(0.0, 0.0)) < collider.config.legacy_sphere_radius
    assert not any(yaw_only_feasible)
    assert _projection_inside(projected, aperture, collider.config.clearance)
    assert not _projection_inside(collider.corners[:, :2], aperture, collider.config.clearance)


def test_center_legal_while_cuboid_collides() -> None:
    aperture = Polygon([(-0.20, -0.20), (0.20, -0.20), (0.20, 0.20), (-0.20, 0.20)])
    collider = CuboidCollider()
    assert aperture.covers(Point(0.0, 0.0))
    assert not _projection_inside(collider.corners[:, :2], aperture, collider.config.clearance)
