# DynaTOGT Algorithm

## Relation to TOGT

The TOGT paper treats a racing gate as a geometric traversal constraint rather than a waypoint:

```text
p(t_i) in G_i
```

DynaTOGT keeps this idea and replaces every static feasible set with a time-varying feasible set:

```text
p(t_i) in G_i(t_i)
```

Each `G_i(t)` is represented by a local 2D convex shape embedded in a moving 3D plane. The plane center, orientation, and local shape scale can all vary with time.

## Decision Variables

For a task sequence with `M` traversal events, DynaTOGT optimizes:

```text
T = [T_1, ..., T_{M+1}]
Z = [z_1, ..., z_M], z_i in R^2
```

`T_i` are positive segment durations. `z_i` are unconstrained local variables. The task sequence may contain repeated windows, so multiple events can reference the same physical `G_j` at different times. A smooth bounded map converts each `z_i` into a local point inside the current dynamic window polygon, and the dynamic window transform maps it to world coordinates:

```text
local_i = phi_i(z_i, t_i)
p_i = world_i(local_i, t_i)
t_i = sum_{k<=i} T_k
```

Thus the geometric constraint is satisfied by construction.

## Objective

The optimized objective is:

```text
J =
  total_time
  + length_weight * path_length
  + acceleration_weight * max_acceleration
  + jerk_weight * mean_jerk
  + violation_weight * dynamic_feasibility_excess
  + margin_weight * gate_margin_penalty
```

The full quadrotor dynamics from the paper are not reproduced. Instead, this prototype uses sampled trajectory speed, acceleration, and jerk as dynamic feasibility proxies.

## Solver

1. Build a discrete warm start by sampling arrival times and traversal points inside each dynamic window.
2. Optionally search traversal order with a beam search over `visited_mask` for `shuffled_dynamic`.
3. Optimize continuous variables with `scipy.optimize.minimize(method="L-BFGS-B")`.
4. Generate a piecewise Hermite polynomial trajectory through start, selected window points, and goal.
5. Validate traversal against the actual dynamic windows at the optimized crossing times.

## Baselines

- `WaypointCenter`: pass through dynamic window centers only.
- `StaticTOGT`: optimize using `G_i(0)` and evaluate against true `G_i(t)`.
- `DiscreteDynamic`: dynamic warm start without continuous optimization.
- `DynaTOGT`: warm start plus continuous optimization.
