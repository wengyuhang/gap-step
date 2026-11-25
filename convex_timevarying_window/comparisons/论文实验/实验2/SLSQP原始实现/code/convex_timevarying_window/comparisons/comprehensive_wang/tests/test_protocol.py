from collections import Counter
from dataclasses import asdict
from types import SimpleNamespace
import numpy as np
import pytest
from convex_timevarying_window.comparisons.comprehensive_wang.registry import trials,HARD
from convex_timevarying_window.comparisons.comprehensive_wang.flight import variant_config
from 复现.polynomial_dynamic_racing_2023.px4_matched_x500.flight import optimizer_failed
from 复现.polynomial_dynamic_racing_2023.px4_matched_x500.adapter import Adapter
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case


def test_matrix_without_repetitions():
    rows=trials();assert len(rows)==27
    assert Counter(r.stage for r in rows)=={'main':18,'ablation':9}
    pairs=Counter(r.pair for r in rows if r.stage=='main')
    assert len(pairs)==9 and set(pairs.values())=={2}
    assert all(r.phase_degrees==0 and r.phase==0. for r in rows)
    main=[r for r in rows if r.stage=='main']
    assert [r.method for r in main[:4]]==['ours','wang','wang','ours']
    assert len({r.name for r in rows})==len(rows)


def test_only_declared_config_changes():
    original=asdict(variant_config('ours'))
    assert asdict(variant_config('center'))==original
    assert asdict(variant_config('no_dense'))==original
    changed=asdict(variant_config('no_soft'))
    assert {k for k in original if original[k]!=changed[k]}=={'soft_clearance_weight'}
    assert changed['soft_clearance_weight']==0
    with pytest.raises(ValueError):variant_config('wang')


def test_optimizer_error_not_a_valid_candidate():
    for code in (-1000,-1,1002):assert optimizer_failed(SimpleNamespace(stats={'code':code}))
    for code in (0,1,1001):assert not optimizer_failed(SimpleNamespace(stats={'code':code}))
    assert not optimizer_failed(None)


def test_projected_terminal_not_unsafe_global_spline():
    track,_=build_case(Case(.2,.3,0.));a=Adapter.__new__(Adapter);a.track=track;a.windows=track.windows
    from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import signs
    a.step=1.
    for gate in range(6):
        w=track.windows[gate];outward=-signs(track)[gate]*w.state_at(0.)[1][:,2]
        original=w.center0.copy();original[2]=-5.
        a.global_seed=SimpleNamespace(sample=lambda t,d=0:original.copy() if d==0 else -outward)
        pva=a.terminal(gate)
        assert np.all(pva[:,0]>=[-27.65,-29.65,1.]) and np.all(pva[:,0]<=[27.65,21.65,11.65])
        assert outward@(pva[:,0]-w.center0)>=2.-1e-8
        assert outward@pva[:,1]>0
        assert np.linalg.norm(pva[:,1])==pytest.approx(6.)
        assert np.all(pva[:,2]==0)
    assert np.all(a.terminal(6)[:,1:]==0)


def test_race_time_excludes_return_on_saved_valid_flight(tmp_path):
    from pathlib import Path
    import shutil
    from convex_timevarying_window.comparisons.comprehensive_wang.audit import audit
    root=Path(__file__).resolve().parents[4]
    old=root/'convex_timevarying_window/comparisons/original_fixed_plane_crossing_resolved/results/v1_20261007/f1_a2_p000_r3'
    if not old.exists():pytest.skip('Optional saved flight fixture unavailable')
    for name in ('result.json','flight.ulg','execution.npz'):shutil.copyfile(old/name,tmp_path/name)
    row=audit(tmp_path)
    assert row['race_passed'] and row['main_passed']
    assert row['race_time_s']==pytest.approx(27.964)
    assert row['flight_time_s']==pytest.approx(31.840)
