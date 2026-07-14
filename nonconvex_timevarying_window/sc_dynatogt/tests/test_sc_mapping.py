from __future__ import annotations

import numpy as np
import pytest
from shapely import contains_xy
from shapely.geometry import Point, Polygon

from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import (
    B,
    SCCrowdingError,
    SCDiskMap,
    SCMappingError,
    jacobian_B,
    polygon_interior_angles,
    polylabel_point,
)


L_SHAPE = np.asarray(
    [[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [1.0, 1.0], [1.0, 2.0], [0.0, 2.0]],
    dtype=np.float64,
)

U_SHAPE = np.asarray(
    [
        [-2.0, -2.0],
        [2.0, -2.0],
        [2.0, 2.0],
        [0.75, 2.0],
        [0.75, -0.5],
        [-0.75, -0.5],
        [-0.75, 2.0],
        [-2.0, 2.0],
    ],
    dtype=np.float64,
)


@pytest.fixture(scope="module")
def l_map() -> SCDiskMap:
    return SCDiskMap.fit(L_SHAPE, quadrature_order=64)


def _central_jacobian(function, x: np.ndarray, h: float = 1e-6) -> np.ndarray:
    columns = []
    for j in range(2):
        step = np.zeros(2)
        step[j] = h
        columns.append((function(x + step) - function(x - step)) / (2.0 * h))
    return np.column_stack(columns)


def test_B_value_open_disk_and_analytic_jacobian() -> None:
    d = np.asarray([1.25, -0.7])
    expected = d / np.sqrt(1.0 + d @ d)
    assert np.allclose(B(d), expected, rtol=0.0, atol=1e-15)
    assert np.linalg.norm(B(np.asarray([1e8, -2e8]))) < 1.0
    numeric = _central_jacobian(B, d, h=1e-6)
    assert np.allclose(jacobian_B(d), numeric, rtol=2e-9, atol=2e-10)


def test_B_and_jacobian_are_finite_for_extreme_finite_inputs() -> None:
    for d in [
        np.asarray([1e155, -2e155]),
        np.asarray([np.finfo(float).max / 2.0, np.finfo(float).max / 3.0]),
        np.asarray([-np.finfo(float).max, 0.0]),
    ]:
        point = B(d)
        jacobian = jacobian_B(d)
        assert np.all(np.isfinite(point))
        assert np.hypot(point[0], point[1]) < 1.0
        assert np.all(np.isfinite(jacobian))
        assert np.allclose(jacobian, jacobian.T, rtol=0.0, atol=0.0)


def test_polygon_angles_include_reflex_vertex_and_polylabel_is_strictly_inside() -> None:
    beta = polygon_interior_angles(L_SHAPE)
    assert np.allclose(beta / np.pi, [0.5, 0.5, 0.5, 1.5, 0.5, 0.5], atol=1e-14)
    assert np.isclose(beta.sum(), (len(L_SHAPE) - 2) * np.pi)
    q0 = polylabel_point(L_SHAPE, tolerance=1e-8)
    assert Polygon(L_SHAPE).contains(Point(q0))
    assert np.isclose(q0[0], q0[1], atol=2e-6)


def test_polygon_validation_rejects_self_intersection() -> None:
    bow_tie = np.asarray([[0, 0], [2, 2], [0, 2], [2, 0]], dtype=np.float64)
    with pytest.raises(ValueError, match="simple"):
        polygon_interior_angles(bow_tie)


def test_fit_reconstructs_l_shape_vertices_and_normalizes_center(l_map: SCDiskMap) -> None:
    assert l_map.diagnostics is not None
    assert l_map.diagnostics.parameter_residual_inf < 2e-7
    assert l_map.diagnostics.vertex_reconstruction_inf < 2e-5
    assert np.allclose(l_map.evaluate([0.0, 0.0]), l_map.q0, rtol=0.0, atol=3e-7)


def test_sc_analytic_jacobian_matches_center_difference_h_1e_6(l_map: SCDiskMap) -> None:
    z = np.asarray([0.17, -0.23], dtype=np.float64)
    numeric = _central_jacobian(l_map.evaluate, z, h=1e-6)
    analytic = l_map.jacobian(z)
    assert np.allclose(analytic, numeric, rtol=2e-7, atol=2e-8)
    # A holomorphic map's real Jacobian has the required Cauchy--Riemann form.
    assert np.isclose(analytic[0, 0], analytic[1, 1], atol=1e-14)
    assert np.isclose(analytic[0, 1], -analytic[1, 0], atol=1e-14)


def test_unconstrained_composed_jacobian_matches_center_difference(l_map: SCDiskMap) -> None:
    d = np.asarray([0.55, -0.8], dtype=np.float64)
    numeric = _central_jacobian(l_map.map_unconstrained, d, h=1e-6)
    assert np.allclose(l_map.jacobian_unconstrained(d), numeric, rtol=3e-7, atol=3e-8)


def test_mapped_disk_samples_are_strictly_inside_nonconvex_polygon(l_map: SCDiskMap) -> None:
    polygon = Polygon(L_SHAPE)
    samples = [np.asarray([0.0, 0.0])]
    for radius in [0.2, 0.55, 0.82, 0.95]:
        for theta in np.linspace(0.0, 2.0 * np.pi, 29, endpoint=False):
            samples.append(radius * np.asarray([np.cos(theta), np.sin(theta)]))
    for z in samples:
        point = l_map.evaluate(z)
        assert polygon.contains(Point(point)), (z, point)


def test_inverse_round_trip_and_save_load(tmp_path, l_map: SCDiskMap) -> None:
    for z in [np.asarray([0.0, 0.0]), np.asarray([0.2, -0.3]), np.asarray([-0.55, 0.15])]:
        assert np.allclose(l_map.inverse(l_map.evaluate(z)), z, atol=2e-7, rtol=0.0)
    path = tmp_path / "l_shape_sc.npz"
    l_map.save(path)
    loaded = SCDiskMap.load(path)
    assert np.allclose(loaded.prevertices, l_map.prevertices)
    for z in [np.asarray([0.0, 0.0]), np.asarray([0.31, 0.27])]:
        assert np.allclose(loaded.evaluate(z), l_map.evaluate(z), atol=2e-12, rtol=0.0)
        assert np.allclose(loaded.jacobian(z), l_map.jacobian(z), atol=2e-12, rtol=0.0)


def test_bulk_evaluation_and_jacobians_match_scalar_paths(l_map: SCDiskMap) -> None:
    points = np.asarray(
        [[0.0, 0.0], [0.2, -0.3], [-0.55, 0.15], [(1.0 - 1e-12), 0.0]],
        dtype=np.float64,
    )
    bulk_values = l_map.evaluate_many(points, batch_size=2)
    bulk_jacobians = l_map.jacobian_many(points, batch_size=2)
    assert bulk_values.shape == (len(points), 2)
    assert bulk_jacobians.shape == (len(points), 2, 2)
    for index, point in enumerate(points):
        assert np.allclose(bulk_values[index], l_map.evaluate(point), rtol=0.0, atol=2e-12)
        assert np.allclose(
            bulk_jacobians[index], l_map.jacobian(point), rtol=2e-13, atol=2e-13
        )
    complex_values = points[:, 0] + 1j * points[:, 1]
    assert np.allclose(l_map.evaluate_many(complex_values), bulk_values, atol=2e-12)


def test_near_boundary_values_and_derivatives_remain_consistent(l_map: SCDiskMap) -> None:
    rng = np.random.default_rng(20260713)
    for distance in [1e-3, 1e-6, 1e-9, 1e-12]:
        for _ in range(4):
            angle = rng.uniform(0.0, 2.0 * np.pi)
            z = (1.0 - distance) * np.exp(1j * angle)
            # Tangential central differences stay inside the disk even for
            # the smallest tested radial clearance.
            dz = 1e-7j * np.exp(1j * angle)
            numeric = (l_map.map_complex(z + dz) - l_map.map_complex(z - dz)) / (2.0 * dz)
            analytic = l_map.derivative(z)
            assert np.isfinite(l_map.map_complex(z))
            assert np.isfinite(analytic)
            assert abs(numeric - analytic) / max(1.0, abs(analytic)) < 2e-7

    # Exercise the endpoint-singularity-aware path, not only random angles.
    a = l_map.normalization
    aligned = []
    for prevertex in l_map.prevertices:
        normalized_boundary = (prevertex - a) / (1.0 - np.conj(a) * prevertex)
        aligned.append((1.0 - 1e-12) * normalized_boundary / abs(normalized_boundary))
    assert np.all(np.isfinite(l_map.evaluate_many(np.asarray(aligned))))
    assert np.all(np.isfinite(l_map.jacobian_many(np.asarray(aligned))))


def test_unconverged_highest_order_quadrature_raises(
    monkeypatch: pytest.MonkeyPatch, l_map: SCDiskMap
) -> None:
    monkeypatch.setattr(
        l_map, "_integral_to_fixed", lambda _z, order: complex(float(order), 0.0)
    )
    monkeypatch.setattr(
        l_map,
        "_integral_to_composite_fixed",
        lambda _z, order, _levels: complex(float(order), 0.0),
    )
    with pytest.raises(SCMappingError, match="clustered"):
        l_map._integral_to(0.13 + 0.07j)


def test_load_rejects_tampered_geometric_parameters(tmp_path, l_map: SCDiskMap) -> None:
    source = tmp_path / "source.npz"
    l_map.save(source)
    with np.load(source, allow_pickle=False) as archive:
        original = {name: np.array(archive[name], copy=True) for name in archive.files}

    def rejected(name: str, mutate) -> None:
        payload = {key: np.array(value, copy=True) for key, value in original.items()}
        mutate(payload)
        path = tmp_path / f"tampered_{name}.npz"
        np.savez_compressed(path, **payload)
        with pytest.raises(SCMappingError):
            SCDiskMap.load(path)

    rejected("A", lambda data: data.__setitem__("A", data["A"] + (0.02 + 0.01j)))
    rejected("C", lambda data: data.__setitem__("C", data["C"] * 1.01))

    def alter_alpha(data) -> None:
        data["alpha"][0] += 0.01

    rejected("alpha", alter_alpha)

    def reorder_prevertices(data) -> None:
        data["prevertices"][[1, 2]] = data["prevertices"][[2, 1]]

    rejected("prevertices", reorder_prevertices)

    def alter_q0(data) -> None:
        data["q0"] += np.asarray([0.03, -0.02])

    rejected("q0", alter_q0)
    rejected(
        "normalization",
        lambda data: data.__setitem__("normalization", data["normalization"] * 0.9),
    )


def test_analytic_prevertex_jacobian_matches_finite_difference() -> None:
    n = 17
    theta = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    radius = 1.0 + 0.12 * np.cos(3.0 * theta)
    polygon = np.column_stack((radius * np.cos(theta), radius * np.sin(theta)))
    exponents = polygon_interior_angles(polygon) / np.pi - 1.0
    quadrature = SCDiskMap._edge_quadrature_data(exponents, 48)
    x = 0.1 * np.random.default_rng(5).normal(size=n - 3)
    prevertices, _, theta_jacobian = SCDiskMap._decode_prevertices_with_jacobian(x, n)
    integrals, angle_jacobian = SCDiskMap._edge_integrals_with_angle_jacobian(
        prevertices, exponents, quadrature
    )
    analytic = np.real(angle_jacobian / integrals[:, None])
    analytic -= analytic.mean(axis=0, keepdims=True)
    analytic = analytic @ theta_jacobian

    def log_lengths(parameters: np.ndarray) -> np.ndarray:
        local_prevertices, _ = SCDiskMap._decode_prevertices(parameters, n)
        local_integrals = SCDiskMap._edge_integrals(
            local_prevertices, exponents, quadrature
        )
        values = np.log(np.abs(local_integrals))
        return values - values.mean()

    for column in [0, 4, n - 4]:
        step = np.zeros(n - 3)
        step[column] = 1e-6
        numeric = (log_lengths(x + step) - log_lengths(x - step)) / (2e-6)
        assert np.allclose(analytic[:, column], numeric, rtol=3e-6, atol=3e-8)


def test_64_vertex_smooth_nonconvex_fit_regression() -> None:
    n = 64
    theta = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    radius = 1.0 + 0.23 * np.cos(3.0 * theta) + 0.05 * np.sin(5.0 * theta)
    polygon = np.column_stack((radius * np.cos(theta), radius * np.sin(theta)))
    mapping = SCDiskMap.fit(polygon, quadrature_order=48, max_nfev=100)
    assert mapping.diagnostics is not None
    assert mapping.diagnostics.optimizer_nfev < 15
    assert mapping.diagnostics.parameter_residual_inf < 2e-7
    assert mapping.diagnostics.vertex_reconstruction_inf < 2e-5


def test_disk_automorphism_preserves_open_disk_for_extreme_optimizer_values(
    l_map: SCDiskMap,
) -> None:
    directions = [
        np.asarray([1.0, -2.0]),
        np.asarray([0.37, 0.91]),
    ]
    # Include directions whose normalized boundary images are prevertices;
    # these are the most demanding subsequent SC integrals.
    a = l_map.normalization
    for prevertex in l_map.prevertices:
        boundary = (prevertex - a) / (1.0 - np.conj(a) * prevertex)
        directions.append(np.asarray([boundary.real, boundary.imag]))

    scale = np.finfo(float).max / 4.0
    disk_points = []
    for direction in directions:
        d = scale * direction / np.max(np.abs(direction))
        disk = B(d)
        transformed = l_map._automorphism(complex(float(disk[0]), float(disk[1])))
        assert np.hypot(transformed.real, transformed.imag) < 1.0
        assert np.all(np.isfinite(l_map.map_unconstrained(d)))
        assert np.all(np.isfinite(l_map.jacobian_unconstrained(d)))
        disk_points.append(disk)

    bulk = np.asarray(disk_points)
    assert np.all(np.isfinite(l_map.evaluate_many(bulk)))
    assert np.all(np.isfinite(l_map.jacobian_many(bulk)))

    # Away from the one-ULP correction surface, the implementation retains
    # the exact Möbius derivative.
    z = 0.72 - 0.31j
    h = 1e-7
    numeric = (l_map._automorphism(z + h) - l_map._automorphism(z - h)) / (2.0 * h)
    assert np.isclose(numeric, l_map._automorphism_derivative(z), rtol=2e-9, atol=2e-10)


@pytest.mark.parametrize("shape", ["mixed", "limacon"])
def test_256_sample_safe_polygon_mixed_and_smooth_maps_are_legal(shape: str) -> None:
    from nonconvex_timevarying_window.sc_dynatogt.preprocessing import (
        PreprocessingConfig,
        limacon_boundary,
        line_bezier_mixed_boundary,
        preprocess_boundary,
    )

    boundary = (
        line_bezier_mixed_boundary() if shape == "mixed" else limacon_boundary()
    )
    gate = preprocess_boundary(
        boundary,
        name=shape,
        config=PreprocessingConfig(
            vertex_counts=(256,),
            sc_fit_options={"quadrature_order": 32},
        ),
    )
    if shape == "mixed":
        # The Round offset removes redundant collinear samples but must keep
        # every one of the resulting short/long safe-polygon edges.
        assert len(gate.safe_polygon) == 106
        assert gate.sc_map.diagnostics.minimum_prevertex_gap < 1e-9
    else:
        assert len(gate.safe_polygon) == 256
    assert gate.sc_map.diagnostics is not None
    assert gate.sc_map.diagnostics.parameter_residual_inf < 2e-7
    assert gate.sc_map.diagnostics.vertex_reconstruction_inf < 2e-5

    rng = np.random.default_rng(91)
    d = rng.normal(0.0, 2.0, size=(1024, 2))
    disk = d / np.sqrt(1.0 + np.einsum("ij,ij->i", d, d))[:, None]
    mapped = gate.sc_map.evaluate_many(disk)
    jacobians = gate.sc_map.jacobian_many(disk)
    polygon = Polygon(gate.safe_polygon)
    assert np.all(contains_xy(polygon, mapped[:, 0], mapped[:, 1]))
    assert np.all(np.isfinite(mapped))
    assert np.all(np.isfinite(jacobians))
    assert np.all(np.linalg.det(jacobians) > 0.0)
    if shape == "mixed":
        from nonconvex_timevarying_window.sc_dynatogt.validation import (
            validate_sc_mapping,
        )

        report = validate_sc_mapping(
            gate.sc_map,
            sample_count=1000,
            seed=0,
            batch_size=128,
        )
        assert report.passed
        assert report.inside_count == 1000


def test_u_shape_general_nonconvex_fit_and_interior_samples() -> None:
    mapping = SCDiskMap.fit(U_SHAPE, quadrature_order=64)
    polygon = Polygon(U_SHAPE)
    assert mapping.diagnostics is not None
    assert mapping.diagnostics.vertex_reconstruction_inf < 2e-5
    for radius in [0.0, 0.45, 0.8, 0.94]:
        for theta in np.linspace(0.0, 2.0 * np.pi, 17, endpoint=False):
            point = mapping.evaluate(radius * np.asarray([np.cos(theta), np.sin(theta)]))
            assert polygon.contains(Point(point))


def test_high_vertex_limit_fails_explicitly() -> None:
    theta = np.linspace(0.0, 2.0 * np.pi, 41, endpoint=False)
    many_vertices = np.column_stack([np.cos(theta), np.sin(theta)])
    with pytest.raises(SCCrowdingError, match="exceed"):
        SCDiskMap.fit(many_vertices, max_vertices=40)
