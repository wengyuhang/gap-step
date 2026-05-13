# Decisions

## 2026-05-11: Refactor to PPO teacher only

Decision: replace the visual student / BC / heuristic teacher project with the continuous 2D rotating time-varying window maze and train only a PPO privileged teacher.

Reasoning: the current research stage is to obtain a teacher policy in a procedurally generated continuous maze before introducing student learning.

## 2026-05-11: Keep all active code in `gap_step/`

Decision: all runnable code lives directly in the `gap_step/` package. Do not keep active code in `trainers/`, `scripts/`, or nested `gap_step/envs`, `gap_step/models`, `gap_step/teachers` packages.

Reasoning: the project should have a clean single-folder code layout without backward compatibility paths.

## 2026-05-11: Ignore generated outputs

Decision: ignore `data/`, `checkpoints/`, `logs/`, `runs/`, and `results/`.

Reasoning: these are reproducible experiment artifacts and may become large.

## 2026-05-11: Generate ordinary maze topology before continuous walls

Decision: generate each maze as a randomized grid maze, add a small number of loop openings, then convert closed edges and selected passage edges into continuous `WallSegment` and `Gate` objects.

Reasoning: this produces ordinary maze-like layouts with both horizontal and vertical corridors while keeping the implementation compact and readable.

## 2026-05-12: Add continuous-geometry progress shaping for PPO training

Decision: keep strict sparse rewards as the environment default, but enable `dynamic_geometry` progress shaping in teacher training configs.

Reasoning: sparse rewards alone collapse to timeout/collision. The shaping term uses continuous visibility geometry and future gate safety sampling without changing observations, collision semantics, or success criteria.

## 2026-05-12: Do not use grid cells for reward potentials

Decision: reward shaping potentials must be computed from continuous geometry: positions, wall rectangles, gate approach points, line-segment visibility, and future gate safety.

Reasoning: cells are topology metadata, not the continuous state. Cell-based potentials can reward misleading movement around walls and windows.

## 2026-05-12: Use squashed Gaussian PPO actions

Decision: sample from a tanh-squashed Gaussian and compute PPO log probabilities for the squashed action actually executed.

Reasoning: `Normal -> clamp(action)` stores log probabilities for a different action than the environment executes.

## 2026-05-12: Remove time-passing reward leakage from progress shaping

Decision: compare `potential(old_pos, current_time)` and `potential(new_pos, current_time)`, then clip the delta before scaling.

Reasoning: waiting near a future-opening gate may be necessary, but pure time passage should not create large dense progress reward.

## 2026-05-13: Adopt full privileged GNN teacher observation

Decision: discard the local vector observation teacher path and use graph privileged observation as the teacher contract. The graph includes full maze topology, gate nodes, wall/open/gate edge labels, current gate safety, gate timing, and gate dynamics.

Reasoning: the previous local teacher could learn C1 but failed to learn stable deterministic C2 behavior. The teacher needs enough privileged information to know which gates matter and when they open/close, while still learning a continuous-action policy rather than executing a planner.

## 2026-05-13: Use pure PyTorch message passing

Decision: implement the GNN actor-critic in pure PyTorch rather than adding PyG/DGL.

Reasoning: this keeps dependencies simple and makes graph batching explicit in `gap_step/graph.py` and `gap_step/ppo.py`.

## 2026-05-13: Split early dynamic curriculum

Decision: replace the direct C1->C2 jump with `C1`, `C1_5`, `C2A`, `C2B`, `C3`, `C4`, `C5`.

Reasoning: the failed run showed that static-gate competence does not transfer directly to waiting for dynamic gates. The bridge stages isolate high-duty-cycle gates, single-gate waiting, and then the original small dynamic maze.

## 2026-05-13: Save best deterministic teacher checkpoint

Decision: save both `teacher_final.pt` and `teacher_best.pt`. Update best from deterministic promotion evaluation.

Reasoning: PPO can pass through useful intermediate policies and later degrade. Evaluation and visualization should default to the best deterministic teacher unless explicitly testing the final state.
