# Architecture

## High-Level Flow

```text
configs/*.yaml
    -> environment / training scripts
        -> gap_step env, teacher, model
            -> data, checkpoints, logs, rollout GIFs
```

## Environment

`gap_step/envs/gap_step_env.py` implements the Gymnasium-style environment.

Key responsibilities:

- maintain robot state: position, velocity, previous action
- apply 2D acceleration action through double-integrator dynamics
- update time-varying gate state
- compute observations: image stack and proprioception
- compute rewards and termination conditions
- report diagnostic metrics through `info`

`gap_step/envs/gate_dynamics.py` owns the sinusoidal gate-width model:

```text
d_i(t) = d_min + (d_max - d_min) / 2 * (1 + sin(omega_i * t + phi_i))
```

A gate is safe when:

```text
width > 2 * robot_radius + safe_margin
```

`gap_step/envs/renderer.py` converts environment state into grayscale images and RGB rollout frames.

## Teacher

`gap_step/teachers/heuristic_teacher.py` implements a privileged heuristic teacher.

The teacher sees true robot state and current gate widths, but not future gate states. It selects:

- the lowest-cost currently safe gate, if any exists
- otherwise the currently widest gate and a staging point before the wall

The teacher outputs:

- `teacher_acc`: 2D acceleration command
- `teacher_gate`: selected gate index

## Student Model

`gap_step/models/student_policy.py` combines:

- CNN encoder for `image_stack`
- MLP encoder for proprioception
- action head for 2D acceleration
- gate classification head
- width regression head
- safety prediction head

The action head is used for environment control. The other heads support imitation, auxiliary supervision, and diagnostics.

## Training Scripts

`trainers/generate_demos.py`

- rolls out the heuristic teacher
- stores image stacks, proprioception, teacher actions, teacher gate labels, true widths, and safety flags

`trainers/train_bc.py`

- trains BC-only and BC+Aux variants
- uses MSE for acceleration imitation
- uses CE for gate classification
- optionally adds L1 width loss and BCE safety loss

`trainers/train_ppo.py`

- loads BC+Aux initialization by default
- performs compact Gaussian continuous-action PPO
- saves best checkpoint according to deterministic evaluation

`trainers/evaluate.py`

- evaluates teacher and available checkpoints
- writes metrics such as success rate, collision rate, closed-gate attempts, width MAE, safe F1, and return

## Generated Artifacts

Generated outputs are ignored by Git:

- `data/`
- `checkpoints/`
- `logs/`
- `runs/`

This keeps the repository small while preserving full reproducibility through configs and scripts.

