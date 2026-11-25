import json

from PIL import Image

from nonconvex_timevarying_window.sc_dynatogt.explain_figures import (
    generate_explanation_figures,
)


def _write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def _synthetic_results(root):
    _write_json(root / "E0" / "summary.json", {"relative_total_time_error": 1.0e-5})
    boundaries = [
        "l_shape",
        "u_shape",
        "five_point_star",
        "limacon",
        "wavy",
        "line_bezier_mixed",
    ]
    _write_json(
        root / "E1" / "summary.json",
        {
            "rows": [
                {"boundary": name, "max_boundary_error_m": 0.0002 * index}
                for index, name in enumerate(boundaries)
            ]
        },
    )
    _write_json(
        root / "E2" / "summary.json",
        {
            "rows": [
                {"method": "fixed_center", "total_time": 2.4},
                {"method": "convex_hull", "total_time": 2.0},
                {"method": "sc", "total_time": 2.1},
            ],
            "sc_convergence_rate": 1.0,
            "sc_legal_rate": 1.0,
        },
    )
    for name, window_error, joint_error, rotor in (
        ("E3", 2.0e-8, 7.0e-6, 5.11),
        ("E4", 3.0e-8, 1.1e-5, 5.01),
    ):
        _write_json(
            root / name / "summary.json",
            {
                "convergence_rate": 1.0,
                "designated_order_legal_rate": 1.0,
                "gradient_reports": [{"p99_relative_error": window_error}],
                "joint_objective_gradient_report": {"p99_relative_error": joint_error},
                "rows": [{"sampled_max_rotor_thrust": rotor}],
                "mapping_validation": [
                    {
                        "sample_count": 1000,
                        "inside_count": 1000,
                        "outside_count": 0,
                        "nan_count": 0,
                        "inf_count": 0,
                        "degenerate_jacobian_count": 0,
                    }
                ],
            },
        )
    _write_json(
        root / "E5" / "summary.json",
        {
            "method_rates": {
                "full_time_gradient": {
                    "convergence_rate": 1.0,
                    "designated_order_legal_rate": 1.0,
                },
                "zero_window_time_gradient": {
                    "convergence_rate": 1.0,
                    "designated_order_legal_rate": 1.0,
                },
            }
        },
    )
    image_root = root / "E4"
    preprocessing = image_root / "preprocessed_gates" / "00_L" / "preprocessing.png"
    preprocessing.parent.mkdir(parents=True, exist_ok=True)
    Image.new("RGB", (320, 100), "white").save(preprocessing)
    Image.new("RGB", (180, 180), "white").save(image_root / "trajectory.png")


def test_plain_language_figure_set_is_generated_from_results(tmp_path):
    results = tmp_path / "smoke"
    _synthetic_results(results)
    outputs = generate_explanation_figures(results, tmp_path / "figures")
    assert len(outputs) == 5
    assert [path.name for path in outputs] == [
        "01_algorithm_overview.png",
        "02_component_map.png",
        "03_dynamic_and_gradients.png",
        "04_how_to_read_outputs.png",
        "05_experiment_results.png",
    ]
    for path in outputs:
        assert path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
        assert path.stat().st_size > 10_000
