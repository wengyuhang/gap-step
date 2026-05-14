from __future__ import annotations

from collections import deque

from gap_step.train import (
    AdaptiveCurriculumState,
    _format_training_log,
    _stage_checkpoint_path,
    _stage_metrics_path,
    adaptive_stage_status,
)


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


def test_stagewise_output_paths_use_stage_subdirectories():
    config = {"checkpoint_dir": "checkpoints", "results_dir": "results"}
    assert _stage_checkpoint_path(config, "C1").endswith("checkpoints/C1/teacher_final.pt")
    assert _stage_metrics_path(config, "C1").endswith("results/C1/train_metrics.csv")


def test_training_log_is_chinese_and_contains_live_metrics():
    row = {
        "stage": "C1",
        "update": 3,
        "stage_steps": 384,
        "episodes": 4,
        "success_rate": 0.25,
        "rolling_success_rate": 0.2,
        "collision_rate": 0.5,
        "timeout_rate": 0.25,
        "average_return": -3.5,
        "average_action_norm": 1.2,
        "entropy": 2.3,
        "approx_kl": 0.01,
        "clip_fraction": 0.125,
        "explained_variance": 0.5,
        "ppo_updates": 6.0,
    }
    text = _format_training_log(row)
    assert "课程 C1" in text
    assert "成功率 0.250" in text
    assert "碰撞率 0.500" in text
    assert "平均回报 -3.50" in text
    assert "裁剪率 0.125" in text
    assert "解释方差 0.500" in text
    assert "PPO更新 6" in text
