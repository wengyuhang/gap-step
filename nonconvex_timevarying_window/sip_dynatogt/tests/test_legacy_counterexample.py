import importlib


def test_preserved_exact_area_counterexample_collides_before_nominal_crossing():
    module = importlib.import_module(
        "nonconvex_timevarying_window.废案.cwb_sc_dynatogt.exact_area_sc_dynatogt.stress_case"
    )
    case = module.build_stress_case()
    assert case.collision_time < case.crossing_time
    snapshot = case.snapshot("Old-0.315", case.collision_time, executed=False)
    assert snapshot.metrics.whole_body_collision
