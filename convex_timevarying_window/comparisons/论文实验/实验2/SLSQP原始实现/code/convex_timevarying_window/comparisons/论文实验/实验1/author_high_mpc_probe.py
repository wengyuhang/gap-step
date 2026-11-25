"""Headless execution of author code/weights; NOT a body-collision benchmark."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
from unittest.mock import patch
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repository', type=Path, default=ROOT/'复现/high_mpc_author')
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--episodes', type=int, default=1)
    parser.add_argument('--source-nlp', action='store_true',
        help='Use the exact author source NLP when its old packaged CasADi binary is ABI-incompatible')
    parser.add_argument('--cache-metadata-compat', action='store_true',
        help='Recompile the author cached C changing only its old upper-Hessian output name')
    parser.add_argument('--metadata-shim', action='store_true',
        help='Use a one-function metadata shim instead of recompiling the large cached C')
    args = parser.parse_args(); repo = args.repository.resolve(); out = args.outdir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    commit = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
    sources = {str(p.relative_to(repo)): hashlib.sha256(p.read_bytes()).hexdigest() for p in repo.rglob('*.py')}
    sys.path.insert(0, str(repo))
    import casadi
    import tensorflow as tf
    from high_mpc.mpc.high_mpc import High_MPC
    from high_mpc.simulation.dynamic_gap import DynamicGap
    from high_mpc.policy.deep_high_policy import Actor
    np.random.seed(args.seed); tf.random.set_seed(args.seed)
    began = time.perf_counter()
    library = repo/'high_mpc/mpc/saved/high_mpc.so'
    if args.source_nlp and args.cache_metadata_compat:
        raise ValueError('Choose only one compatibility mode')
    compatibility = dict(enabled=args.source_nlp or args.cache_metadata_compat, algorithm_parameters_changed=False)
    if args.cache_metadata_compat:
        original_c = repo/'high_mpc/mpc/saved/high_mpc.c'
        text = original_c.read_text()
        old_name, new_name = '"hess_gamma_x_x"', '"triu_hess_gamma_x_x"'
        if text.count(old_name) != 1:
            raise RuntimeError('Unexpected author cache format')
        old_hessian = casadi.external('nlp_hess_l', str(library))
        if not old_hessian.sparsity_out(0).is_triu():
            raise RuntimeError('Author Hessian is not upper triangular; renaming is invalid')
        source = out/'author_cache_metadata_compat.c'
        adapted_library = out/'author_cache_metadata_compat.so'
        if args.metadata_shim:
            source.write_text('const char* nlp_hess_l_name_out(long long int i) { return i == 0 ? "triu_hess_gamma_x_x" : 0; }\n')
            print('BUILDING_ONE_FUNCTION_METADATA_SHIM', flush=True)
            subprocess.run(['gcc', '-fPIC', '-shared', str(source), '-Wl,--no-as-needed', str(library),
                '-o', str(adapted_library)], check=True)
        else:
            source.write_text(text.replace(old_name, new_name))
            print('COMPILING_AUTHOR_CACHE_METADATA_ONLY', flush=True)
            subprocess.run(['gcc', '-O1', '-fPIC', '-shared', str(source), '-o', str(adapted_library), '-lm'], check=True)
        captured = {}
        original_nlpsol = casadi.nlpsol
        def metadata_solver(name, backend, model, options):
            if isinstance(model, dict):
                captured['graph'] = model
            if isinstance(model, str) and model == str(library):
                return original_nlpsol(name, backend, str(adapted_library), options)
            return original_nlpsol(name, backend, model, options)
        with patch.object(casadi, 'nlpsol', metadata_solver):
            controller = High_MPC(T=2., dt=.04, so_path=str(library))
        rng = np.random.default_rng(381)
        differences = []
        for name in ('nlp_f', 'nlp_g', 'nlp_grad_f', 'nlp_jac_g', 'nlp_hess_l'):
            old = casadi.external(name, str(library)); new = casadi.external(name, str(adapted_library))
            maximum = 0.
            for _ in range(3):
                values = [rng.normal(0, .1, old.size_in(k)) for k in range(old.n_in())]
                a, b = old.call(values), new.call(values)
                for av, bv in zip(a, b):
                    maximum = max(maximum, float(np.max(np.abs(np.asarray(av)-np.asarray(bv)))))
                    np.testing.assert_allclose(np.asarray(av), np.asarray(bv), rtol=1e-10, atol=1e-10)
            differences.append(dict(function=name, maximum_absolute_difference=maximum))
        compatibility.update(execution='original cached C; only upper-Hessian metadata output name changed',
            author_c_sha256=hashlib.sha256(text.encode()).hexdigest(),
            cache_function_equivalence=differences, modified_metadata_strings=1,
            author_python_source_retained=True, metadata_shim=args.metadata_shim,
            compiler_flags=['-fPIC', '-shared'] if args.metadata_shim else ['-O1', '-fPIC', '-shared'])
    elif args.source_nlp:
        original_nlpsol = casadi.nlpsol
        captured = {}
        def source_solver(name, backend, model, options):
            if isinstance(model, dict):
                captured['graph'] = model
                captured['solver'] = original_nlpsol(name, backend, model, options)
                return captured['solver']
            if str(model) == str(library):
                return captured['solver']
            return original_nlpsol(name, backend, model, options)
        with patch.object(casadi, 'nlpsol', source_solver):
            controller = High_MPC(T=2., dt=.04, so_path=str(library))
        graph = captured['graph']
        exact = casadi.Function('author_source_fg', [graph['x'], graph['p']], [graph['f'], graph['g']])
        old_f = casadi.external('nlp_f', str(library)); old_g = casadi.external('nlp_g', str(library))
        rng = np.random.default_rng(381)
        differences = []
        for _ in range(3):
            x = np.asarray(controller.nlp_w0)+rng.normal(0, .01, int(graph['x'].numel()))
            p = rng.normal(0, .1, int(graph['p'].numel()))
            f, g = exact(x, p)
            delta_f = abs(float(f)-float(old_f(x, p)))
            delta_g = float(np.max(np.abs(np.asarray(g)-np.asarray(old_g(x, p)))))
            differences.append(dict(objective_absolute_difference=delta_f, constraint_max_difference=delta_g))
            np.testing.assert_allclose(float(f), float(old_f(x, p)), rtol=1e-10, atol=1e-10)
            np.testing.assert_allclose(np.asarray(g), np.asarray(old_g(x, p)), rtol=1e-10, atol=1e-10)
        compatibility.update(old_binary_fg_comparison=differences,
            execution='author source NLP via current CasADi; bypass incompatible old cached solver ABI')
    else:
        controller = High_MPC(T=2., dt=.04, so_path=str(library))
    env = DynamicGap(controller, 2., .04)
    actor = Actor(9, 1)
    weights = repo/'high_mpc/Dataset/act_net/weights_999.h5'
    actor.load_weights(str(weights))
    build_s = time.perf_counter()-began
    metadata = dict(author_commit=commit, source_sha256=sources, weights_sha256=hashlib.sha256(weights.read_bytes()).hexdigest(),
        tensorflow=tf.__version__, casadi=casadi.__version__, numpy=np.__version__, python=sys.version,
        build_s=build_s, original_model_and_weights=True, controller_parameters_modified=False,
        cached_solver_compatibility=compatibility,
        runtime_difference='TensorFlow CPU 2.15 and current CasADi, not original pinned CasADi 3.5.1',
        platform='author Python integrator, NOT Gazebo/PX4/x500', formal_comparison=False,
        body_collision_assessed=False, collision_detected=None,
        gate_geometry=dict(width_m=env.pend.width, height_m=env.pend.height, length_m=env.pend.length,
            pivot_m=env.pivot_point.tolist(), dimensions_used_for_visualization_only=True))
    (out/'metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
    episodes = []
    for episode in range(args.episodes):
        observation = env.reset(); initial_quad = env.quad.get_state().copy(); initial_pend = env.pend.get_state().copy()
        states = [initial_quad]; pendulum = [initial_pend]; actions = []; selected_times = []; cycles = []
        failure = None
        try:
            for _ in range(env.max_episode_steps):
                started = time.perf_counter()
                act = actor(np.asarray(observation).reshape(1, -1)).numpy()[0]
                observation, _, _, info = env.step(act)
                cycles.append(1000*(time.perf_counter()-started))
                states.append(env.quad.get_state().copy()); pendulum.append(env.pend.get_state().copy())
                actions.append(np.asarray(info['quad_act']).ravel()); selected_times.append(float(act[0]))
        except Exception as exc:
            failure = repr(exc)
        states = np.asarray(states); pendulum = np.asarray(pendulum)
        times = np.arange(len(states))*env.sim_dt; events = []
        for k in np.flatnonzero((states[:-1, 0]-env.pivot_point[0])*(states[1:, 0]-env.pivot_point[0]) < 0):
            alpha = (env.pivot_point[0]-states[k, 0])/(states[k+1, 0]-states[k, 0])
            p = (1-alpha)*states[k, :3]+alpha*states[k+1, :3]
            theta = (1-alpha)*pendulum[k, 0]+alpha*pendulum[k+1, 0]
            delta = p-env.pivot_point
            local = [np.cos(theta)*delta[1]+np.sin(theta)*delta[2],
                -np.sin(theta)*delta[1]+np.cos(theta)*delta[2]+env.pend.length]
            events.append(dict(time_s=float(times[k]+alpha*env.sim_dt), forward=bool(states[k+1, 0] > states[k, 0]),
                point_local_m=list(map(float, local)), point_inside=bool(abs(local[0]) <= env.pend.width/2 and abs(local[1]) <= env.pend.height/2)))
        row = dict(episode=episode, seed=args.seed, steps=len(actions), failure=failure,
            initial_quad=initial_quad.tolist(), initial_pendulum=initial_pend.tolist(), crossings=events,
            final_goal_error_m=float(np.linalg.norm(states[-1, :3]-env.goal_point)),
            final_speed_mps=float(np.linalg.norm(states[-1, 7:10])),
            cycle_ms=dict(p50=float(np.median(cycles)), p95=float(np.percentile(cycles, 95)), maximum=float(max(cycles))) if cycles else None,
            body_collision_assessed=False, formal_success=None)
        np.savez_compressed(out/f'episode_{episode:02d}.npz', times=times, states=states, pendulum=pendulum,
            actions=np.asarray(actions), selected_traversal_time_s=selected_times, cycle_ms=cycles)
        episodes.append(row)
        print(json.dumps(row), flush=True)
    metadata['source_unchanged'] = sources == {str(p.relative_to(repo)): hashlib.sha256(p.read_bytes()).hexdigest() for p in repo.rglob('*.py')}
    (out/'result.json').write_text(json.dumps(dict(metadata=metadata, episodes=episodes), indent=2)+'\n')
    if any(r['failure'] for r in episodes) or not metadata['source_unchanged']:
        raise RuntimeError('author reproduction failed; inspect retained raw result')


if __name__ == '__main__':
    main()
