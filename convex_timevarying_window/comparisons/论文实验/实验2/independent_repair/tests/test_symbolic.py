import casadi as ca
import numpy as np
from ..ipopt_planner import Planner
from ..symbolic import constraints
from ...basic_group.whole_course_difficulty import scenario


def test_compressed_two_sided_constraints_preserve_original_violation():
    track, _ = scenario.build_case(scenario.Case(level='a50_f6'))
    p = Planner(track, False, pose_function=scenario.pose)
    witnesses = p.witnesses()
    witnesses[(0, 7)] = dict(kind='normalized', values=list(np.linspace(.1, .9, 21)))
    x = ca.MX.sym('test_variables', p.nvar)
    g, upper = constraints(p, x, witnesses)
    function = ca.Function('compressed_test', [x], [g, ca.jacobian(g, x)])
    indices = []
    stride = p.physical.size1_out(0)
    for piece in range(p.n):
        for node in range(p.cfg.nodes_per_piece+1):
            indices.extend((piece*stride+node*17+np.r_[0:7, 11:14]).tolist())
    cursor = p.n*stride
    for (gate, piece), item in sorted(witnesses.items()):
        batch = p.safety[gate].size1_out(0)
        per_sample = batch//p.cfg.witness_slots
        for begin in range(0, len(item['values']), p.cfg.witness_slots):
            offsets = np.resize(item['values'][begin:begin+p.cfg.witness_slots], p.cfg.witness_slots)
            for index in sorted(np.unique(offsets, return_index=True)[1]):
                indices.extend((cursor+index*per_sample+np.arange(per_sample)).tolist())
            cursor += batch
    indices.extend(range(cursor, cursor+p.m))
    for seed in p.seeds():
        original, original_derivative, _ = p.evaluate(seed, witnesses)
        actual, derivative = function(seed)
        actual = np.asarray(actual).ravel()
        np.testing.assert_allclose(actual, original[indices], rtol=1e-9, atol=1e-7)
        np.testing.assert_allclose(np.asarray(derivative), original_derivative[indices], rtol=1e-7, atol=1e-6)
        np.testing.assert_allclose(max(0., -original.min()),
            max(0., -actual.min(), float(np.max(actual-upper))), rtol=1e-8, atol=1e-8)
        assert np.isfinite(np.asarray(derivative)).all()
    assert len(upper) < len(original)*.75
