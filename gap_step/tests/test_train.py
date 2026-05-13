from __future__ import annotations

from collections import deque

from gap_step.train import AdaptiveCurriculumState, adaptive_stage_status


def test_adaptive_curriculum_promotes_on_success_threshold():
    state = AdaptiveCurriculumState(
        stage_steps=500,
        recent_successes=deque([True] * 7 + [False] * 3, maxlen=10),
        last_promotion_eval_success_rate=0.6,
    )
    status = adaptive_stage_status(
        state,
        min_steps=500,
        soft_max_steps=2000,
        hard_max_steps=5000,
        promotion_success_rate=0.70,
        promotion_eval_success_rate=0.60,
        hard_max_policy="stop",
    )
    assert status == "promoted_success"


def test_adaptive_curriculum_requires_deterministic_eval_for_promotion():
    state = AdaptiveCurriculumState(
        stage_steps=500,
        recent_successes=deque([True] * 8 + [False] * 2, maxlen=10),
        last_promotion_eval_success_rate=0.5,
    )
    status = adaptive_stage_status(
        state,
        min_steps=500,
        soft_max_steps=2000,
        hard_max_steps=5000,
        promotion_success_rate=0.70,
        promotion_eval_success_rate=0.60,
        hard_max_policy="stop",
    )
    assert status == "training"


def test_adaptive_curriculum_soft_max_warns_once_without_promotion():
    state = AdaptiveCurriculumState(stage_steps=2000, recent_successes=deque([False] * 10, maxlen=10))
    kwargs = {
        "min_steps": 500,
        "soft_max_steps": 2000,
        "hard_max_steps": 5000,
        "promotion_success_rate": 0.70,
        "hard_max_policy": "stop",
    }
    assert adaptive_stage_status(state, **kwargs) == "soft_max_warning"
    assert adaptive_stage_status(state, **kwargs) == "training"


def test_adaptive_curriculum_hard_max_stops_without_promotion():
    state = AdaptiveCurriculumState(stage_steps=5000, recent_successes=deque([False] * 10, maxlen=10))
    status = adaptive_stage_status(
        state,
        min_steps=500,
        soft_max_steps=2000,
        hard_max_steps=5000,
        promotion_success_rate=0.70,
        hard_max_policy="stop",
    )
    assert status == "hard_max_stop"
