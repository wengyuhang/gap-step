# GAP-Step

GAP-Step is a minimal 2D research project for dynamic multi-gate crossing. It tests whether privileged heuristic demonstrations plus visual behavior cloning, gate-state auxiliary prediction, and PPO fine-tuning can learn gate choice and crossing timing.

This implementation intentionally does not include a 3D quadrotor, full SITT proxy-student machinery, world models, future video prediction, or active camera control.

## Environment

The provided environment uses GPU PyTorch for an NVIDIA driver compatible with CUDA 12.x. For the supplied machine info, `pytorch-cuda=12.1` is used.

```bash
conda env create -f environment.yml
conda activate gap-step
```

If you already have a compatible environment, such as `isaac`, you can use it directly:

```bash
conda activate isaac
```

## Required Commands

```bash
python scripts/render_random_policy.py
python trainers/generate_demos.py
python trainers/train_bc.py
python trainers/train_ppo.py
python trainers/evaluate.py
```

## Useful Commands

```bash
python trainers/train_bc.py --mode bc_only
python trainers/train_bc.py --mode bc_aux
python trainers/train_ppo.py --init none --output checkpoints/visual_ppo.pt
python scripts/render_trained_policy.py --checkpoint checkpoints/bc_aux.pt
python scripts/render_trained_policy.py --checkpoint checkpoints/bc_aux_ppo.pt
pytest
```

## Outputs

- Demonstrations: `data/demos_e3.npz`
- Checkpoints: `checkpoints/bc_only.pt`, `checkpoints/bc_aux.pt`, `checkpoints/bc_aux_ppo.pt`
- TensorBoard logs: `runs/`
- Evaluation table: `logs/eval_results.csv`
- Rollout GIFs: `runs/random_policy.gif`, `runs/trained_policy.gif`

## Notes

`train_ppo.py` implements a compact continuous-action Gaussian PPO. The environment executes only the 2D acceleration action; the gate head is kept for imitation, diagnostics, and auxiliary interpretability. This is a deliberate MVP choice rather than a full hybrid discrete-continuous PPO implementation.
