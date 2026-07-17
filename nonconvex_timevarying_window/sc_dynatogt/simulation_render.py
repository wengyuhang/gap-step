"""AirSim-style offline rendering for SC-DynaTOGT trajectories.

This module is deliberately separate from the optimizer and the Matplotlib
diagnostic plots.  It consumes the same physical window poses and degree-7
MINCO trajectory, then renders a small simulated environment through an
off-screen OpenGL context.  It is a cinematic visualization, not a second
physics engine and not an AirSim integration.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from functools import lru_cache
import json
import os
from pathlib import Path
from typing import Any, Sequence

import imageio.v2 as imageio
import numpy as np
from numpy.typing import ArrayLike, NDArray
from PIL import Image, ImageDraw, ImageEnhance, ImageFont

from .environment import SCDynamicWindow, SCWindowTrack
from .minco import BoundaryState, MincoSnap
from .optimizer import OptimizationResult
from .visualization import _quadrotor_basis, _trajectory


FloatArray = NDArray[np.float64]
UInt8Image = NDArray[np.uint8]
TrajectoryInput = MincoSnap | OptimizationResult


@dataclass(frozen=True)
class SimulationRenderConfig:
    """Quality and camera settings for the OpenGL scene renderer."""

    width: int = 960
    height: int = 540
    frame_count: int = 144
    fps: float = 12.0
    gate_tube_radius: float = 0.105
    drone_arm_length: float = 0.38
    maximum_gate_segments: int = 96
    camera_distance: float = 5.2
    camera_height: float = 2.0
    field_of_view_degrees: float = 62.0
    shadows: bool = True
    render_video: bool = True

    def __post_init__(self) -> None:
        if self.width < 320 or self.height < 240:
            raise ValueError("render size must be at least 320 x 240")
        if self.frame_count < 2:
            raise ValueError("frame_count must be at least two")
        if self.fps <= 0.0:
            raise ValueError("fps must be positive")
        if self.gate_tube_radius <= 0.0 or self.drone_arm_length <= 0.0:
            raise ValueError("gate and drone dimensions must be positive")
        if self.maximum_gate_segments < 12:
            raise ValueError("maximum_gate_segments must be at least 12")
        if self.camera_distance <= 0.0 or self.camera_height < 0.0:
            raise ValueError("camera distance must be positive and height nonnegative")
        if not (25.0 <= self.field_of_view_degrees <= 110.0):
            raise ValueError("field_of_view_degrees must be in [25, 110]")


@lru_cache(maxsize=1)
def _renderer_modules() -> tuple[Any, Any]:
    """Load optional rendering dependencies only for simulation output."""

    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
    try:
        import pyrender
        import trimesh
    except ImportError as exc:  # pragma: no cover - depends on optional extras
        raise ImportError(
            "AirSim-style rendering requires the optional packages in "
            "requirements-render.txt"
        ) from exc
    return pyrender, trimesh


def _closed_without_duplicate(boundary: ArrayLike) -> FloatArray:
    points = np.asarray(boundary, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3:
        raise ValueError("boundary must have shape (n, 2), n >= 3")
    if not np.all(np.isfinite(points)):
        raise ValueError("boundary must contain finite values")
    if np.linalg.norm(points[0] - points[-1]) <= 1.0e-12:
        points = points[:-1]
    if len(points) < 3:
        raise ValueError("boundary degenerates after removing its duplicate endpoint")
    return points


def _resample_closed_boundary(boundary: ArrayLike, maximum_segments: int) -> FloatArray:
    """Reduce dense display curves while retaining visually sharp vertices."""

    points = _closed_without_duplicate(boundary)
    if len(points) <= maximum_segments:
        return points.copy()

    edges = np.roll(points, -1, axis=0) - points
    lengths = np.linalg.norm(edges, axis=1)
    if float(lengths.min()) <= 1.0e-12:
        raise ValueError("boundary contains a zero-length edge")
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    perimeter = float(cumulative[-1])

    incoming = points - np.roll(points, 1, axis=0)
    outgoing = np.roll(points, -1, axis=0) - points
    incoming /= np.linalg.norm(incoming, axis=1)[:, None]
    outgoing /= np.linalg.norm(outgoing, axis=1)[:, None]
    turn = np.arccos(np.clip(np.einsum("ij,ij->i", incoming, outgoing), -1.0, 1.0))
    corner_indices = np.flatnonzero(turn > np.deg2rad(9.0))
    if len(corner_indices) > maximum_segments - 3:
        strongest = np.argsort(turn[corner_indices])[-(maximum_segments - 3) :]
        corner_indices = np.sort(corner_indices[strongest])

    uniform_count = max(3, maximum_segments - len(corner_indices))
    sample_distances = np.linspace(0.0, perimeter, uniform_count, endpoint=False)
    forced_distances = cumulative[corner_indices]
    distances = np.unique(np.round(np.concatenate((sample_distances, forced_distances)), 12))
    distances.sort()
    if len(distances) > maximum_segments:
        # True corners take priority; only thin the uniform candidates.
        forced_keys = set(np.round(forced_distances, 12).tolist())
        uniform = np.asarray([value for value in distances if value not in forced_keys])
        keep = maximum_segments - len(forced_distances)
        indices = np.linspace(0, max(0, len(uniform) - 1), max(0, keep), dtype=int)
        distances = np.sort(np.concatenate((forced_distances, uniform[indices])))

    segment = np.searchsorted(cumulative[1:], distances, side="right")
    local = (distances - cumulative[segment]) / lengths[segment]
    return points[segment] + local[:, None] * edges[segment]


def _look_at(eye: ArrayLike, target: ArrayLike, up: ArrayLike = (0.0, 0.0, 1.0)) -> FloatArray:
    """Return an OpenGL camera pose whose local -z axis faces ``target``."""

    eye_array = np.asarray(eye, dtype=float)
    target_array = np.asarray(target, dtype=float)
    up_array = np.asarray(up, dtype=float)
    if eye_array.shape != (3,) or target_array.shape != (3,) or up_array.shape != (3,):
        raise ValueError("eye, target, and up must have shape (3,)")
    forward = target_array - eye_array
    forward_norm = float(np.linalg.norm(forward))
    if forward_norm <= 1.0e-9:
        raise ValueError("eye and target must differ")
    forward /= forward_norm
    right = np.cross(forward, up_array)
    if float(np.linalg.norm(right)) <= 1.0e-9:
        axes = np.eye(3)
        up_array = axes[int(np.argmin(np.abs(axes @ forward)))]
        right = np.cross(forward, up_array)
    right /= np.linalg.norm(right)
    camera_up = np.cross(right, forward)
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack((right, camera_up, -forward))
    pose[:3, 3] = eye_array
    return pose


def _pose(rotation: ArrayLike, translation: ArrayLike, scale: float = 1.0) -> FloatArray:
    matrix = np.eye(4)
    matrix[:3, :3] = scale * np.asarray(rotation, dtype=float)
    matrix[:3, 3] = np.asarray(translation, dtype=float)
    return matrix


def _window_pose(window: SCDynamicWindow, time: float) -> FloatArray:
    center, basis, scale, *_ = window.state_at(float(time))
    normal = np.cross(basis[:, 0], basis[:, 1])
    rotation = np.column_stack((basis[:, 0], basis[:, 1], normal))
    return _pose(rotation, center, scale)


def _cylinder_between(trimesh: Any, start: ArrayLike, finish: ArrayLike, radius: float, *, sections: int = 12) -> Any:
    start_array = np.asarray(start, dtype=float)
    finish_array = np.asarray(finish, dtype=float)
    direction = finish_array - start_array
    length = float(np.linalg.norm(direction))
    if length <= 1.0e-10:
        raise ValueError("cylinder endpoints must differ")
    transform = trimesh.geometry.align_vectors((0.0, 0.0, 1.0), direction)
    transform[:3, 3] = 0.5 * (start_array + finish_array)
    return trimesh.creation.cylinder(
        radius=radius,
        height=length,
        sections=sections,
        transform=transform,
    )


def _tube_mesh(trimesh: Any, points: ArrayLike, radius: float, *, closed: bool, sections: int = 12) -> Any:
    vertices = np.asarray(points, dtype=float)
    segments = list(zip(vertices[:-1], vertices[1:]))
    if closed:
        segments.append((vertices[-1], vertices[0]))
    parts = [
        _cylinder_between(trimesh, start, finish, radius, sections=sections)
        for start, finish in segments
        if np.linalg.norm(finish - start) > 1.0e-10
    ]
    if not parts:
        raise ValueError("tube path must contain a nonzero segment")
    return trimesh.util.concatenate(parts)


def _material(pyrender: Any, color: Sequence[float], *, metallic: float, roughness: float, emissive: Sequence[float] | None = None, alpha_mode: str = "OPAQUE") -> Any:
    rgba = tuple(float(value) for value in color)
    if len(rgba) == 3:
        rgba = (*rgba, 1.0)
    return pyrender.MetallicRoughnessMaterial(
        baseColorFactor=rgba,
        metallicFactor=float(metallic),
        roughnessFactor=float(roughness),
        emissiveFactor=None if emissive is None else tuple(emissive),
        alphaMode=alpha_mode,
    )


def _build_gate_mesh(pyrender: Any, trimesh: Any, boundary: FloatArray, config: SimulationRenderConfig) -> Any:
    sampled = _resample_closed_boundary(boundary, config.maximum_gate_segments)
    points = np.column_stack((sampled, np.zeros(len(sampled))))
    tube = _tube_mesh(
        trimesh,
        points,
        config.gate_tube_radius,
        closed=True,
        sections=14,
    )
    orange = _material(
        pyrender,
        (0.98, 0.20, 0.025, 1.0),
        metallic=0.18,
        roughness=0.27,
        emissive=(0.16, 0.018, 0.0),
    )
    return pyrender.Mesh.from_trimesh(tube, material=orange, smooth=False)


def _translated(mesh: Any, translation: Sequence[float]) -> Any:
    mesh.apply_translation(np.asarray(translation, dtype=float))
    return mesh


def _build_drone_meshes(pyrender: Any, trimesh: Any, arm_length: float) -> list[Any]:
    diagonal = arm_length / np.sqrt(2.0)
    rotor_centers = np.asarray(
        (
            (diagonal, diagonal, 0.035),
            (diagonal, -diagonal, 0.035),
            (-diagonal, -diagonal, 0.035),
            (-diagonal, diagonal, 0.035),
        )
    )

    frame_parts: list[Any] = []
    for center in rotor_centers:
        frame_parts.append(
            _cylinder_between(trimesh, (0.0, 0.0, 0.0), center, 0.027, sections=12)
        )
        frame_parts.append(
            _translated(
                trimesh.creation.cylinder(radius=0.052, height=0.075, sections=18),
                center,
            )
        )
    frame_parts.append(trimesh.creation.box(extents=(0.31, 0.19, 0.09)))
    canopy = trimesh.creation.icosphere(subdivisions=2, radius=0.115)
    canopy.apply_scale((1.18, 0.78, 0.48))
    canopy.apply_translation((0.015, 0.0, 0.07))
    frame_parts.append(canopy)
    frame = trimesh.util.concatenate(frame_parts)

    propeller_parts: list[Any] = []
    blade_length = 0.31 * arm_length / 0.38
    for index, center in enumerate(rotor_centers):
        angle = np.deg2rad(18.0 + 24.0 * index)
        for extra in (0.0, 0.5 * np.pi):
            blade = trimesh.creation.box(extents=(blade_length, 0.018, 0.008))
            transform = trimesh.transformations.rotation_matrix(angle + extra, (0.0, 0.0, 1.0))
            transform[:3, 3] = center + np.asarray((0.0, 0.0, 0.065))
            blade.apply_transform(transform)
            propeller_parts.append(blade)
    propellers = trimesh.util.concatenate(propeller_parts)

    nose = trimesh.creation.cone(radius=0.065, height=0.15, sections=20)
    nose.apply_transform(trimesh.transformations.rotation_matrix(0.5 * np.pi, (0.0, 1.0, 0.0)))
    nose.apply_translation((0.22, 0.0, 0.025))

    left_light = _translated(trimesh.creation.icosphere(subdivisions=2, radius=0.025), (0.20, 0.075, 0.05))
    right_light = _translated(trimesh.creation.icosphere(subdivisions=2, radius=0.025), (0.20, -0.075, 0.05))

    return [
        pyrender.Mesh.from_trimesh(
            frame,
            material=_material(pyrender, (0.035, 0.045, 0.055, 1.0), metallic=0.72, roughness=0.24),
            smooth=False,
        ),
        pyrender.Mesh.from_trimesh(
            propellers,
            material=_material(pyrender, (0.09, 0.10, 0.11, 1.0), metallic=0.48, roughness=0.20),
            smooth=False,
        ),
        pyrender.Mesh.from_trimesh(
            nose,
            material=_material(pyrender, (0.90, 0.035, 0.025, 1.0), metallic=0.15, roughness=0.28, emissive=(0.08, 0.0, 0.0)),
            smooth=False,
        ),
        pyrender.Mesh.from_trimesh(
            left_light,
            material=_material(pyrender, (0.03, 0.95, 0.23, 1.0), metallic=0.0, roughness=0.15, emissive=(0.0, 0.8, 0.08)),
            smooth=True,
        ),
        pyrender.Mesh.from_trimesh(
            right_light,
            material=_material(pyrender, (0.95, 0.03, 0.02, 1.0), metallic=0.0, roughness=0.15, emissive=(0.8, 0.0, 0.0)),
            smooth=True,
        ),
    ]


def _add_landscape(scene: Any, pyrender: Any, trimesh: Any, points: FloatArray) -> tuple[FloatArray, FloatArray]:
    lower = points.min(axis=0)
    upper = points.max(axis=0)
    span = np.maximum(upper - lower, 1.0)
    center = 0.5 * (lower + upper)
    ground_extent = float(max(span[0], span[1]) + 55.0)

    ground = trimesh.creation.box(extents=(ground_extent, ground_extent, 0.24))
    ground.apply_translation((center[0], center[1], -0.13))
    scene.add(
        pyrender.Mesh.from_trimesh(
            ground,
            material=_material(pyrender, (0.20, 0.33, 0.18, 1.0), metallic=0.0, roughness=0.94),
        )
    )

    road_parts = []
    road_x = trimesh.creation.box(extents=(ground_extent, 5.8, 0.055))
    road_x.apply_translation((center[0], center[1], 0.015))
    road_parts.append(road_x)
    road_y = trimesh.creation.box(extents=(5.8, ground_extent, 0.055))
    road_y.apply_translation((center[0] + 4.0, center[1], 0.018))
    road_parts.append(road_y)
    scene.add(
        pyrender.Mesh.from_trimesh(
            trimesh.util.concatenate(road_parts),
            material=_material(pyrender, (0.075, 0.085, 0.09, 1.0), metallic=0.0, roughness=0.82),
        )
    )

    rng = np.random.default_rng(731)
    building_groups: list[list[Any]] = [[], [], []]
    roof_parts: list[Any] = []
    window_parts: list[Any] = []
    trunk_parts: list[Any] = []
    canopy_parts: list[Any] = []
    x_positions = np.linspace(lower[0] - 10.0, upper[0] + 10.0, 10)
    y_positions = np.linspace(lower[1] - 10.0, upper[1] + 10.0, 8)
    locations: list[tuple[float, float]] = []
    locations.extend((float(x), float(lower[1] - 11.0)) for x in x_positions)
    locations.extend((float(x), float(upper[1] + 11.0)) for x in x_positions)
    locations.extend((float(lower[0] - 12.0), float(y)) for y in y_positions[1:-1])
    locations.extend((float(upper[0] + 12.0), float(y)) for y in y_positions[1:-1])
    for building_index, (x, y) in enumerate(locations):
        width = float(rng.uniform(3.2, 6.2))
        depth = float(rng.uniform(3.0, 5.8))
        height = float(rng.uniform(4.0, 12.0))
        x += float(rng.uniform(-0.8, 0.8))
        y += float(rng.uniform(-0.8, 0.8))
        building = trimesh.creation.box(extents=(width, depth, height))
        building.apply_translation((x, y, 0.5 * height))
        building_groups[building_index % len(building_groups)].append(building)

        roof = trimesh.creation.box(extents=(width + 0.12, depth + 0.12, 0.14))
        roof.apply_translation((x, y, height + 0.07))
        roof_parts.append(roof)

        toward_center = center[:2] - np.asarray((x, y))
        if abs(toward_center[0]) >= abs(toward_center[1]):
            face_sign = 1.0 if toward_center[0] >= 0.0 else -1.0
            glass = trimesh.creation.box(extents=(0.055, 0.68 * depth, 0.56 * height))
            glass.apply_translation((x + face_sign * (0.5 * width + 0.032), y, 0.56 * height))
        else:
            face_sign = 1.0 if toward_center[1] >= 0.0 else -1.0
            glass = trimesh.creation.box(extents=(0.68 * width, 0.055, 0.56 * height))
            glass.apply_translation((x, y + face_sign * (0.5 * depth + 0.032), 0.56 * height))
        window_parts.append(glass)

        tree_xy = np.asarray((x, y)) + 0.25 * toward_center
        trunk_height = float(rng.uniform(1.4, 2.2))
        trunk = trimesh.creation.cylinder(radius=0.12, height=trunk_height, sections=10)
        trunk.apply_translation((tree_xy[0], tree_xy[1], 0.5 * trunk_height))
        trunk_parts.append(trunk)
        canopy = trimesh.creation.icosphere(subdivisions=2, radius=float(rng.uniform(0.65, 1.05)))
        canopy.apply_scale((1.0, 1.0, 1.18))
        canopy.apply_translation((tree_xy[0], tree_xy[1], trunk_height + 0.55))
        canopy_parts.append(canopy)

    building_colors = (
        (0.46, 0.50, 0.53, 1.0),
        (0.53, 0.49, 0.43, 1.0),
        (0.38, 0.43, 0.48, 1.0),
    )
    for group, color in zip(building_groups, building_colors):
        scene.add(
            pyrender.Mesh.from_trimesh(
                trimesh.util.concatenate(group),
                material=_material(pyrender, color, metallic=0.06, roughness=0.64),
            )
        )
    scene.add(
        pyrender.Mesh.from_trimesh(
            trimesh.util.concatenate(roof_parts),
            material=_material(pyrender, (0.16, 0.18, 0.20, 1.0), metallic=0.20, roughness=0.58),
        )
    )
    scene.add(
        pyrender.Mesh.from_trimesh(
            trimesh.util.concatenate(window_parts),
            material=_material(
                pyrender,
                (0.09, 0.25, 0.34, 1.0),
                metallic=0.55,
                roughness=0.16,
                emissive=(0.005, 0.028, 0.045),
            ),
        )
    )
    scene.add(
        pyrender.Mesh.from_trimesh(
            trimesh.util.concatenate(trunk_parts),
            material=_material(pyrender, (0.22, 0.10, 0.04, 1.0), metallic=0.0, roughness=0.92),
        )
    )
    scene.add(
        pyrender.Mesh.from_trimesh(
            trimesh.util.concatenate(canopy_parts),
            material=_material(pyrender, (0.09, 0.30, 0.11, 1.0), metallic=0.0, roughness=0.88),
            smooth=True,
        )
    )

    return lower, upper


def _trajectory_mesh(pyrender: Any, trimesh: Any, trajectory: MincoSnap) -> Any:
    positions = np.asarray(np.real(trajectory.sample(num_samples=220).position), dtype=float)
    mesh = _tube_mesh(trimesh, positions, 0.032, closed=False, sections=8)
    material = _material(
        pyrender,
        (0.025, 0.43, 0.92, 1.0),
        metallic=0.12,
        roughness=0.24,
        emissive=(0.0, 0.16, 0.55),
    )
    return pyrender.Mesh.from_trimesh(mesh, material=material, smooth=False)


def _gate_crossing_times(value: TrajectoryInput, trajectory: MincoSnap, count: int) -> FloatArray:
    supplied = getattr(value, "traversal_times", None)
    times = np.cumsum(np.real(trajectory.durations))[:-1] if supplied is None else np.asarray(supplied, dtype=float)
    if times.shape != (count,):
        raise ValueError(f"expected {count} crossing times, got {times.shape}")
    return np.asarray(times, dtype=float)


def _apply_atmosphere(color: UInt8Image, depth: FloatArray) -> UInt8Image:
    height, width = depth.shape
    vertical = np.linspace(0.0, 1.0, height)[:, None, None]
    sky_top = np.asarray((72.0, 145.0, 205.0))[None, None, :]
    sky_horizon = np.asarray((190.0, 218.0, 233.0))[None, None, :]
    sky_image = (1.0 - vertical) * sky_top + vertical * sky_horizon
    sky_image = np.broadcast_to(sky_image, (height, width, 3)).copy()
    y_grid, x_grid = np.mgrid[:height, :width]
    sun_x, sun_y = 0.82 * width, 0.15 * height
    sun_distance = np.sqrt((x_grid - sun_x) ** 2 + (y_grid - sun_y) ** 2)
    sun_glow = np.clip(1.0 - sun_distance / (0.24 * height), 0.0, 1.0) ** 2
    sun_color = np.asarray((255.0, 235.0, 184.0))
    sky_image = (1.0 - 0.46 * sun_glow[..., None]) * sky_image + 0.46 * sun_glow[..., None] * sun_color

    working = np.asarray(color, dtype=float).copy()
    background = depth <= 0.0
    working[background] = sky_image[background]
    fog_color = np.asarray((151.0, 190.0, 219.0))
    factor = np.clip((depth - 24.0) / 70.0, 0.0, 0.58)
    factor[depth <= 0.0] = 0.0
    blended = (1.0 - factor[..., None]) * working + factor[..., None] * fog_color
    image = Image.fromarray(np.asarray(np.clip(blended, 0.0, 255.0), dtype=np.uint8))
    image = ImageEnhance.Contrast(image).enhance(1.06)
    image = ImageEnhance.Color(image).enhance(1.08)
    return np.asarray(image, dtype=np.uint8)


@lru_cache(maxsize=4)
def _font(size: int) -> ImageFont.ImageFont:
    paths = (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    )
    for path in paths:
        if Path(path).is_file():
            return ImageFont.truetype(path, size=size)
    return ImageFont.load_default()


def _add_hud(frame: UInt8Image, *, time: float, speed: float, next_gate: str, mode: str) -> UInt8Image:
    image = Image.fromarray(frame)
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    width, height = image.size
    draw.rounded_rectangle((20, 18, 324, 75), radius=10, fill=(8, 18, 27, 178), outline=(76, 219, 235, 205), width=2)
    draw.text((36, 29), "SC-DynaTOGT", font=_font(20), fill=(235, 250, 252, 255))
    draw.text((193, 34), mode, font=_font(13), fill=(81, 223, 235, 255))
    draw.rounded_rectangle((20, height - 104, 287, height - 20), radius=10, fill=(8, 18, 27, 175))
    draw.text((36, height - 91), f"TIME       {time:05.2f} s", font=_font(16), fill=(240, 245, 248, 255))
    draw.text((36, height - 67), f"SPEED      {speed:05.2f} m/s", font=_font(16), fill=(240, 245, 248, 255))
    draw.text((36, height - 43), f"NEXT       {next_gate}", font=_font(16), fill=(255, 173, 51, 255))
    cross_x, cross_y = width // 2, height // 2
    draw.line((cross_x - 10, cross_y, cross_x - 3, cross_y), fill=(235, 248, 250, 150), width=1)
    draw.line((cross_x + 3, cross_y, cross_x + 10, cross_y), fill=(235, 248, 250, 150), width=1)
    draw.line((cross_x, cross_y - 10, cross_x, cross_y - 3), fill=(235, 248, 250, 150), width=1)
    draw.line((cross_x, cross_y + 3, cross_x, cross_y + 10), fill=(235, 248, 250, 150), width=1)
    image = Image.alpha_composite(image.convert("RGBA"), overlay).convert("RGB")
    return np.asarray(image, dtype=np.uint8)


def _next_gate_label(crossings: FloatArray, time: float) -> str:
    next_index = int(np.searchsorted(crossings, float(time), side="right"))
    return "FINISH" if next_index >= len(crossings) else f"GATE {next_index + 1}"


def _drone_pose(trajectory: MincoSnap, time: float) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray]:
    position = np.asarray(np.real(trajectory.evaluate(time)), dtype=float)
    velocity = np.asarray(np.real(trajectory.evaluate(time, derivative=1)), dtype=float)
    acceleration = np.asarray(np.real(trajectory.evaluate(time, derivative=2)), dtype=float)
    basis = _quadrotor_basis(velocity, acceleration)
    return _pose(basis, position), position, velocity, acceleration


def _camera_target(
    trajectory: MincoSnap,
    time: float,
    previous_forward: FloatArray,
    config: SimulationRenderConfig,
) -> tuple[FloatArray, FloatArray, FloatArray]:
    position = np.asarray(np.real(trajectory.evaluate(time)), dtype=float)
    velocity = np.asarray(np.real(trajectory.evaluate(time, derivative=1)), dtype=float)
    speed = float(np.linalg.norm(velocity))
    if speed > 0.35:
        forward = velocity / speed
    else:
        future_time = min(float(np.real(trajectory.total_time)), float(time) + 0.15)
        delta = np.asarray(np.real(trajectory.evaluate(future_time)), dtype=float) - position
        forward = delta / np.linalg.norm(delta) if np.linalg.norm(delta) > 1.0e-8 else previous_forward
    horizontal = np.asarray((forward[0], forward[1], 0.0))
    if np.linalg.norm(horizontal) <= 1.0e-8:
        horizontal = previous_forward
    else:
        horizontal /= np.linalg.norm(horizontal)
    eye = position - config.camera_distance * horizontal + np.asarray((0.0, 0.0, config.camera_height))
    target = position + 2.8 * forward + np.asarray((0.0, 0.0, 0.28))
    return eye, target, horizontal


def render_simulation_scene(
    track: SCWindowTrack,
    trajectory_or_result: TrajectoryInput,
    output_dir: str | Path,
    *,
    config: SimulationRenderConfig = SimulationRenderConfig(),
) -> dict[str, str]:
    """Render overview/chase PNGs and an optional chase-camera MP4."""

    pyrender, trimesh = _renderer_modules()
    trajectory = _trajectory(trajectory_or_result)
    crossings = _gate_crossing_times(trajectory_or_result, trajectory, len(track.order))
    root = Path(output_dir).expanduser()
    root.mkdir(parents=True, exist_ok=True)

    total_time = float(np.real(trajectory.total_time))
    course_positions = np.asarray(
        np.real(trajectory.sample(num_samples=320).position), dtype=float
    )
    scene = pyrender.Scene(
        bg_color=np.asarray((0.55, 0.73, 0.88, 1.0)),
        ambient_light=np.asarray((0.25, 0.27, 0.30)),
    )
    lower, upper = _add_landscape(scene, pyrender, trimesh, course_positions)

    gate_nodes = []
    for window in track.windows:
        physical = window.physical_boundary
        if physical is None:
            raise ValueError(
                f"window {window.name!r} has no physical boundary for simulation rendering"
            )
        mesh = _build_gate_mesh(pyrender, trimesh, physical, config)
        gate_nodes.append(scene.add(mesh, pose=np.eye(4)))

    drone_nodes = [
        scene.add(mesh, pose=np.eye(4))
        for mesh in _build_drone_meshes(
            pyrender, trimesh, config.drone_arm_length
        )
    ]
    path_node = scene.add(_trajectory_mesh(pyrender, trimesh, trajectory), pose=np.eye(4))

    camera = pyrender.PerspectiveCamera(
        yfov=np.deg2rad(config.field_of_view_degrees),
        aspectRatio=config.width / config.height,
        znear=0.08,
        zfar=180.0,
    )
    camera_node = scene.add(camera, pose=np.eye(4))
    center = 0.5 * (lower + upper)
    sun_eye = center + np.asarray((-28.0, -34.0, 52.0))
    sun_pose = _look_at(sun_eye, center)
    scene.add(
        pyrender.DirectionalLight(color=np.asarray((1.0, 0.94, 0.84)), intensity=4.2),
        pose=sun_pose,
    )
    scene.add(
        pyrender.PointLight(color=np.asarray((0.62, 0.74, 1.0)), intensity=48.0),
        pose=_pose(np.eye(3), center + np.asarray((0.0, 0.0, 13.0))),
    )

    flags = pyrender.RenderFlags.SHADOWS_DIRECTIONAL if config.shadows else pyrender.RenderFlags.NONE
    renderer = pyrender.OffscreenRenderer(config.width, config.height)
    output: dict[str, str] = {}
    writer: Any | None = None
    try:
        still_time = 0.48 * total_time
        for node, window in zip(gate_nodes, track.windows):
            scene.set_pose(node, _window_pose(window, still_time))
        drone_pose, drone_position, drone_velocity, _ = _drone_pose(trajectory, still_time)
        for node in drone_nodes:
            scene.set_pose(node, drone_pose)

        span = np.maximum(upper - lower, 1.0)
        overview_eye = center + np.asarray(
            (-0.50 * span[0] - 7.0, -0.58 * span[1] - 7.0, 0.48 * max(span[:2]) + 9.0)
        )
        overview_target = center + np.asarray((0.0, 0.0, 3.0))
        scene.set_pose(camera_node, _look_at(overview_eye, overview_target))
        color, depth = renderer.render(scene, flags=flags)
        overview = _add_hud(
            _apply_atmosphere(color, depth),
            time=still_time,
            speed=float(np.linalg.norm(drone_velocity)),
            next_gate=_next_gate_label(crossings, still_time),
            mode="OVERVIEW CAM",
        )
        overview_path = root / "airsim_overview.png"
        imageio.imwrite(overview_path, overview)
        output["overview_png"] = str(overview_path)

        scene.remove_node(path_node)
        initial_forward = np.asarray((1.0, 0.0, 0.0))
        chase_eye, chase_target, initial_forward = _camera_target(
            trajectory, still_time, initial_forward, config
        )
        scene.set_pose(camera_node, _look_at(chase_eye, chase_target))
        color, depth = renderer.render(scene, flags=flags)
        chase = _add_hud(
            _apply_atmosphere(color, depth),
            time=still_time,
            speed=float(np.linalg.norm(drone_velocity)),
            next_gate=_next_gate_label(crossings, still_time),
            mode="CHASE CAM",
        )
        chase_path = root / "airsim_chase.png"
        imageio.imwrite(chase_path, chase)
        output["chase_png"] = str(chase_path)

        if config.render_video:
            video_path = root / "airsim_chase.mp4"
            try:
                writer = imageio.get_writer(
                    video_path,
                    fps=config.fps,
                    codec="libx264",
                    quality=8,
                    macro_block_size=None,
                    ffmpeg_log_level="warning",
                )
            except (ImportError, RuntimeError) as exc:  # pragma: no cover - optional binary
                raise ImportError(
                    "MP4 output requires imageio-ffmpeg from requirements-render.txt"
                ) from exc

            smoothed_eye: FloatArray | None = None
            smoothed_target: FloatArray | None = None
            previous_forward = np.asarray((1.0, 0.0, 0.0))
            for time in np.linspace(0.0, total_time, config.frame_count):
                for node, window in zip(gate_nodes, track.windows):
                    scene.set_pose(node, _window_pose(window, float(time)))
                current_pose, _, velocity, _ = _drone_pose(trajectory, float(time))
                for node in drone_nodes:
                    scene.set_pose(node, current_pose)

                desired_eye, desired_target, previous_forward = _camera_target(
                    trajectory, float(time), previous_forward, config
                )
                if smoothed_eye is None:
                    smoothed_eye = desired_eye
                    smoothed_target = desired_target
                else:
                    smoothed_eye = 0.78 * smoothed_eye + 0.22 * desired_eye
                    assert smoothed_target is not None
                    smoothed_target = 0.72 * smoothed_target + 0.28 * desired_target
                assert smoothed_target is not None
                scene.set_pose(camera_node, _look_at(smoothed_eye, smoothed_target))
                color, depth = renderer.render(scene, flags=flags)
                frame = _add_hud(
                    _apply_atmosphere(color, depth),
                    time=float(time),
                    speed=float(np.linalg.norm(velocity)),
                    next_gate=_next_gate_label(crossings, float(time)),
                    mode="CHASE CAM",
                )
                writer.append_data(frame)
            writer.close()
            writer = None
            output["chase_mp4"] = str(video_path)
    finally:
        if writer is not None:
            writer.close()
        renderer.delete()
    return output


def render_diverse_summary(
    summary_path: str | Path,
    output_dir: str | Path,
    *,
    config: SimulationRenderConfig = SimulationRenderConfig(),
) -> dict[str, str]:
    """Reconstruct and render a saved ``diverse_demo`` optimization result."""

    from .experiments import ExperimentSettings, _preprocessing_config
    from .scenarios import build_diverse_scenario

    source = Path(summary_path).expanduser()
    payload = json.loads(source.read_text(encoding="utf-8"))
    result = payload["result"]
    scenario = build_diverse_scenario(
        mode=str(payload["mode"]),
        preprocessing_config=_preprocessing_config(
            ExperimentSettings(suite=str(payload["quality"]))
        ),
        layout=str(payload["layout"]),
        motion_scale=float(payload["motion_scale"]),
    )
    trajectory = MincoSnap(
        BoundaryState(scenario.track.start),
        BoundaryState(scenario.track.goal),
        np.asarray(result["waypoints"], dtype=float),
        np.asarray(result["durations"], dtype=float),
    )
    return render_simulation_scene(
        scenario.track,
        trajectory,
        output_dir,
        config=config,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Render an AirSim-style offline scene from a saved diverse demo"
    )
    parser.add_argument(
        "--summary",
        type=Path,
        default=Path(
            "nonconvex_timevarying_window/sc_dynatogt/results/"
            "diverse_paper_irregular_closed/summary.json"
        ),
    )
    parser.add_argument(
        "--outdir",
        type=Path,
        default=Path(
            "nonconvex_timevarying_window/sc_dynatogt/results/"
            "diverse_paper_irregular_closed_airsim_style"
        ),
    )
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--frames", type=int, default=144)
    parser.add_argument("--fps", type=float, default=12.0)
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args(argv)
    output = render_diverse_summary(
        args.summary,
        args.outdir,
        config=SimulationRenderConfig(
            width=args.width,
            height=args.height,
            frame_count=args.frames,
            fps=args.fps,
            render_video=not args.no_video,
        ),
    )
    print(json.dumps(output, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "SimulationRenderConfig",
    "main",
    "render_diverse_summary",
    "render_simulation_scene",
]
