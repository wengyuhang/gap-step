// Direct, unmodified TOGT race-planner entry point.  No Python callback,
// no custom gate map, and no reconstructed MINCO objective are involved.
#include <chrono>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "drolib/race/race_params.hpp"
#include "drolib/race/race_planner.hpp"
#include "drolib/race/race_track.hpp"

namespace {

drolib::QuadState state_from_pvaj(const drolib::PVAJ& pvaj) {
  drolib::QuadState state;
  state.setZero();
  state.p = pvaj.col(0);
  state.v = pvaj.col(1);
  state.a = pvaj.col(2);
  state.j = pvaj.col(3);
  return state;
}

std::shared_ptr<drolib::RaceTrack> subtrack(const drolib::RaceTrack& full,
                                             const drolib::QuadState& head,
                                             const drolib::QuadState& tail,
                                             const size_t first_gate,
                                             const size_t gate_count) {
  auto local = std::make_shared<drolib::RaceTrack>(head, tail);
  for (size_t i = 0; i < gate_count; ++i) local->gates.push_back(full.gates.at(first_gate + i));
  return local;
}

struct BlockRecord {
  int index{0}; size_t first_gate{0}; size_t horizon{0}; size_t executed{0};
  double planning{0.}; double warm_start_planning{0.};
  double objective{NAN}; double energy{NAN}; double penalty{NAN};
  double time_cost{NAN}; double tail_prior{NAN};
};
struct ExecutedPiece { double duration; Eigen::Matrix<double, drolib::POLY_DEG + 1, 3> coefficients; };
struct Result { bool success{true}; int calls{0}; int optimizer_attempts{0}; int warm_start_calls{0}; int warm_start_failures{0}; int rescues{0}; int rescue_horizon{0}; int failure_code{0}; double failure_objective{NAN}; double failure_gradient{NAN}; double flight{0.}; double planning{0.}; double warm_start_planning{0.}; double executed_soft_penalty{NAN}; std::vector<BlockRecord> blocks; std::vector<ExecutedPiece> executed_pieces; };

double evaluate_executed_soft_penalty(const drolib::RaceParams& params,
                                      const std::vector<ExecutedPiece>& pieces) {
  drolib::QuadManifold quad(params.qp);
  drolib::ConstAngle yaw(0.0);
  double total = 0.0;
  for (const auto& piece : pieces) {
    Eigen::VectorXd durations(1);
    durations(0) = piece.duration;
    Eigen::MatrixX3d coefficients(drolib::NUM_COEFF, 3);
    // Polynomial stores coefficients in descending order; the released TOGT
    // penalty routine expects MINCO's ascending-power row layout.
    for (int row = 0; row < drolib::NUM_COEFF; ++row) {
      coefficients.row(row) = piece.coefficients.row(drolib::NUM_COEFF - 1 - row);
    }
    Eigen::MatrixX3d grad_coeff = Eigen::MatrixX3d::Zero(drolib::NUM_COEFF, 3);
    Eigen::VectorXd grad_time = Eigen::VectorXd::Zero(1);
    total += drolib::TrajSolver::addPenaltyCost(durations, coefficients, quad, &yaw,
                                                 params.tpinit, grad_coeff, grad_time);
  }
  return total;
}

Result native_replan(const drolib::RaceParams& params, const drolib::RaceTrack& full,
                     const int horizon, const bool rolling, const bool rescue = false,
                     const bool one_sided = false, const bool right_pvaj = false,
                     const std::vector<drolib::PVAJ>* tail_warm_start = nullptr,
                     const bool lookahead_warm_start = false,
                     const double tail_seed_weight = 0.0,
                     const bool robust_tail_seed = false) {
  Result output;
  drolib::QuadState head = full.initState;
  size_t next_gate = 0;
  drolib::PVAJ previous_tail_seed;
  bool has_previous_tail_seed = false;
  while (next_gate < full.gates.size()) {
    const size_t remaining = full.gates.size() - next_gate;
    const size_t nominal = std::min(static_cast<size_t>(horizon), remaining);
    std::vector<size_t> candidates{nominal};
    if (rescue) {
      for (const int candidate : {horizon - 1, horizon + 1, 1}) {
        if (candidate > 0) {
          const size_t bounded = std::min(static_cast<size_t>(candidate), remaining);
          if (std::find(candidates.begin(), candidates.end(), bounded) == candidates.end()) candidates.push_back(bounded);
        }
      }
    }
    bool ok = false; size_t local_gates = nominal; drolib::MincoSnapTrajectory trajectory;
    for (size_t attempt = 0; attempt < candidates.size(); ++attempt) {
      double block_planning = 0.0;
      double block_warm_start_planning = 0.0;
      local_gates = candidates[attempt];
      const bool final_call = remaining <= (rolling ? 1U : local_gates);
      drolib::QuadState tail = full.endState;
      size_t interior_gates = local_gates;
      drolib::Waypoint terminal_gate;
      if ((one_sided || right_pvaj) && !final_call) {
        // No artificial future anchor: terminate exactly at the last gate of
        // this horizon, with its PVAJ left to the natural one-sided MINCO map.
        tail = full.initState;
        terminal_gate = full.gates.at(next_gate + local_gates - 1)->corridor.front();
        terminal_gate.point = terminal_gate.shape->position;
        tail.p = terminal_gate.point;
        tail.v.setZero(); tail.a.setZero(); tail.j.setZero();
        interior_gates = local_gates - 1;
      }
      const auto local = subtrack(full, head, tail, next_gate, interior_gates);
      drolib::PVAJ local_terminal_seed;
      const drolib::PVAJ* terminal_seed = nullptr;
      if (right_pvaj && !final_call && tail_warm_start) {
        terminal_seed = &tail_warm_start->at(next_gate + local_gates - 1);
      }
      if (right_pvaj && !final_call && lookahead_warm_start) {
        // A one-gate look-ahead is an initializer only.  The actual local
        // problem below still ends at G_H and has no anchor segment.
        const size_t seed_gates = std::min(local_gates + 1, remaining);
        const auto seed_track = subtrack(full, head, full.endState, next_gate, seed_gates);
        drolib::RacePlanner seed_planner(params);
        const auto seed_t0 = std::chrono::steady_clock::now();
        const bool seed_ok = seed_planner.planTOGT(seed_track);
        const double seed_elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - seed_t0).count();
        output.warm_start_planning += seed_elapsed;
        block_warm_start_planning += seed_elapsed;
        ++output.warm_start_calls;
        if (seed_ok) {
          const auto seed_trajectory = seed_planner.getTrajectory();
          const double terminal_time = seed_trajectory.durations.head(static_cast<Eigen::Index>(local_gates)).sum();
          local_terminal_seed = seed_trajectory.polys.getPVAJ(terminal_time);
        } else if (has_previous_tail_seed) {
          // The preceding solved local trajectory is a valid online
          // predictor.  Its position is reset to a legal point in the new
          // terminal aperture; its V/A/J remain an initialization only.
          ++output.warm_start_failures;
          local_terminal_seed = previous_tail_seed;
          local_terminal_seed.col(0) = terminal_gate.point;
        } else {
          output.success = false;
          output.failure_code = seed_planner.getLastLbfgsCode();
          output.failure_objective = seed_planner.getLastObjective();
          output.failure_gradient = seed_planner.getLastGradientInfNorm();
          return output;
        }
        terminal_seed = &local_terminal_seed;
      }
      std::vector<double> seed_weights{tail_seed_weight};
      if (robust_tail_seed && terminal_seed) {
        // Same horizon and same objective except for the declared terminal
        // trust-region strength.  No horizon switch and no hidden terminal
        // state constraint is introduced by these retries.
        for (const double weight : {0.0, 0.3, 1.0, 3.0}) {
          if (std::find(seed_weights.begin(), seed_weights.end(), weight) == seed_weights.end()) {
            seed_weights.push_back(weight);
          }
        }
      }
      double final_objective = NAN, final_energy = NAN, final_penalty = NAN, final_time_cost = NAN, final_tail_prior = NAN;
      for (const double seed_weight : seed_weights) {
        drolib::RacePlanner planner(params);
        const auto t0 = std::chrono::steady_clock::now();
        ok = right_pvaj && !final_call ? planner.planTOGTRightPVAJ(local, terminal_gate, terminal_seed, seed_weight)
           : one_sided && !final_call ? planner.planTOGTOneSided(local)
           : planner.planTOGT(local);
        const double local_elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        output.planning += local_elapsed;
        block_planning += local_elapsed;
        ++output.optimizer_attempts;
        if (ok) {
          trajectory = planner.getTrajectory();
          final_objective = planner.getLastObjective();
          final_energy = planner.getLastEnergyCost();
          final_penalty = planner.getLastPenaltyCost();
          final_time_cost = planner.getLastTimeCost();
          final_tail_prior = planner.getLastTailPriorCost();
          break;
        }
        output.failure_code = planner.getLastLbfgsCode();
        output.failure_objective = planner.getLastObjective();
        output.failure_gradient = planner.getLastGradientInfNorm();
      }
      ++output.calls;
      if (ok) {
        output.blocks.push_back(BlockRecord{static_cast<int>(output.blocks.size() + 1), next_gate + 1,
                                             local_gates, rolling ? 1U : local_gates,
                                             block_planning, block_warm_start_planning,
                                             final_objective, final_energy, final_penalty,
                                             final_time_cost, final_tail_prior});
        if (right_pvaj && !final_call) {
          previous_tail_seed = trajectory.polys.getPVAJ(trajectory.getTotalDuration());
          has_previous_tail_seed = true;
        }
        if (attempt) { ++output.rescues; output.rescue_horizon = static_cast<int>(local_gates); }
        break;
      }
    }
    if (!ok) { output.success = false; return output; }
    const size_t executable = rolling ? 1 : local_gates;
    // At the final replanning call, execute its gate segment(s) and the real
    // gate-to-goal segment.  Before that, pass the exact C++ MINCO PVAJ at the
    // gate boundary to the next native problem.
    if (remaining <= executable) {
      output.flight += trajectory.getTotalDuration();
      for (int i = 0; i < trajectory.polys.getPieceNum(); ++i) {
        output.executed_pieces.push_back(ExecutedPiece{trajectory.polys[i].getDuration(), trajectory.polys[i].getCoeffMat().transpose()});
      }
      next_gate += remaining;
    } else {
      const double cut = trajectory.durations.head(static_cast<Eigen::Index>(executable)).sum();
      output.flight += cut;
      for (size_t i = 0; i < executable; ++i) {
        output.executed_pieces.push_back(ExecutedPiece{trajectory.polys[static_cast<int>(i)].getDuration(), trajectory.polys[static_cast<int>(i)].getCoeffMat().transpose()});
      }
      head = state_from_pvaj(trajectory.polys.getPVAJ(cut));
      next_gate += executable;
    }
  }
  output.executed_soft_penalty = evaluate_executed_soft_penalty(params, output.executed_pieces);
  return output;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4 && argc != 6) {
    std::cerr << "usage: togt_native_race <cpc_parameter_dir> <setup_yaml> [--rolling|--rolling-one-sided|--rolling-right-pvaj|--rolling-right-pvaj-warm|--rolling-right-pvaj-lookahead|--rolling-right-pvaj-lookahead-regularized|--rolling-right-pvaj-lookahead-robust|--rolling-rescue|--blocks|--blocks-right-pvaj horizon]\n";
    return 64;
  }
  const auto t0 = std::chrono::steady_clock::now();
  // RaceParams resolves the setup's relative component paths against this
  // directory, so make the root explicit before the released loader sees it.
  const auto parameter_dir = std::filesystem::absolute(argv[1]);
  const auto track_file = std::filesystem::absolute(argv[3]);
  auto params = std::make_shared<drolib::RaceParams>(parameter_dir.string(), argv[2]);
  // Experimental protocol: keep both native L-BFGS stopping tolerances at
  // 1e-5.  All other parameters remain those in cpc_setups.yaml.
  params->lpinit.params.delta = 1.0e-5;
  params->lpinit.params.g_epsilon = 1.0e-5;
  if (const char* raw = std::getenv("TOGT_GRAD_TOL")) params->lpinit.params.g_epsilon = std::stod(raw);
  if (const char* raw = std::getenv("TOGT_MAX_LINESEARCH")) params->lpinit.params.max_linesearch = std::stoi(raw);
  auto track = std::make_shared<drolib::RaceTrack>(track_file);
  auto planner = std::make_shared<drolib::RacePlanner>(*params);
  if (argc == 6) {
    const std::string mode(argv[4]);
    const int horizon = std::stoi(argv[5]);
    if (horizon < 1 || (mode != "--rolling" && mode != "--rolling-one-sided" && mode != "--rolling-right-pvaj" && mode != "--rolling-right-pvaj-warm" && mode != "--rolling-right-pvaj-lookahead" && mode != "--rolling-right-pvaj-lookahead-regularized" && mode != "--rolling-right-pvaj-lookahead-robust" && mode != "--rolling-rescue" && mode != "--blocks" && mode != "--blocks-right-pvaj")) {
      std::cerr << "horizon must be positive and mode must be --rolling, --rolling-one-sided, --rolling-right-pvaj, --rolling-right-pvaj-warm, --rolling-right-pvaj-lookahead, --rolling-right-pvaj-lookahead-regularized, --rolling-right-pvaj-lookahead-robust, --rolling-rescue, --blocks, or --blocks-right-pvaj\n";
      return 64;
    }
    std::vector<drolib::PVAJ> tail_warm_start;
    double reference_planning = 0.0;
    if (mode == "--rolling-right-pvaj-warm") {
      // This reference is used solely as an initial guess.  In each local
      // solve the terminal gate point and PVAJ remain unconstrained variables.
      auto reference_track = std::make_shared<drolib::RaceTrack>(track_file);
      drolib::RacePlanner reference(*params);
      const auto reference_t0 = std::chrono::steady_clock::now();
      if (!reference.planTOGT(reference_track)) {
        std::cerr << "reference TOGT warm-start solve failed\n";
        return 2;
      }
      reference_planning = std::chrono::duration<double>(std::chrono::steady_clock::now() - reference_t0).count();
      const auto reference_trajectory = reference.getTrajectory();
      double gate_time = 0.0;
      for (Eigen::Index i = 0; i < reference_trajectory.durations.size(); ++i) {
        gate_time += reference_trajectory.durations(i);
        tail_warm_start.push_back(reference_trajectory.polys.getPVAJ(gate_time));
      }
    }
    const bool rolling_mode = mode != "--blocks" && mode != "--blocks-right-pvaj";
    const bool regularized_mode = mode == "--rolling-right-pvaj-lookahead-regularized" || mode == "--rolling-right-pvaj-lookahead-robust";
    double tail_seed_weight = regularized_mode ? 0.1 : 0.0;
    if (regularized_mode) {
      if (const char* raw = std::getenv("TOGT_TAIL_SEED_WEIGHT")) tail_seed_weight = std::stod(raw);
    }
    const Result result = native_replan(*params, *track, horizon, rolling_mode,
                                        mode == "--rolling-rescue", mode == "--rolling-one-sided",
                                        mode == "--rolling-right-pvaj" || mode == "--blocks-right-pvaj" || mode == "--rolling-right-pvaj-warm" || mode == "--rolling-right-pvaj-lookahead" || mode == "--rolling-right-pvaj-lookahead-regularized" || mode == "--rolling-right-pvaj-lookahead-robust",
                                        tail_warm_start.empty() ? nullptr : &tail_warm_start,
                                        mode == "--rolling-right-pvaj-lookahead" || regularized_mode,
                                        tail_seed_weight, mode == "--rolling-right-pvaj-lookahead-robust");
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::cout << std::fixed << std::setprecision(9)
              << "native_success=" << (result.success ? 1 : 0) << "\n"
              << "gate_count=" << track->gates.size() << "\n"
              << "replan_calls=" << result.calls << "\n"
              << "optimizer_attempts=" << result.optimizer_attempts << "\n"
              << "warm_start_calls=" << result.warm_start_calls << "\n"
              << "warm_start_failures=" << result.warm_start_failures << "\n"
              << "rescue_count=" << result.rescues << "\n"
              << "last_rescue_horizon=" << result.rescue_horizon << "\n"
              << "failure_lbfgs_code=" << result.failure_code << "\n"
              << "failure_objective=" << result.failure_objective << "\n"
              << "failure_gradient_inf_norm=" << result.failure_gradient << "\n"
              << "reference_warm_start_seconds=" << reference_planning << "\n"
              << "lookahead_warm_start_seconds=" << result.warm_start_planning << "\n"
              << "tail_seed_weight=" << tail_seed_weight << "\n"
              << "planning_seconds=" << result.planning << "\n"
              << "executed_soft_penalty=" << result.executed_soft_penalty << "\n"
              << "wall_seconds=" << seconds << "\n"
              << "flight_time_s=" << result.flight << "\n";
    for (const auto& block : result.blocks) {
      std::cout << "block=" << block.index
                << ",first_gate=" << block.first_gate
                << ",horizon_gates=" << block.horizon
                << ",executed_gates=" << block.executed
                << ",planning_seconds=" << block.planning
                << ",warm_start_seconds=" << block.warm_start_planning
                << ",objective=" << block.objective
                << ",energy_cost=" << block.energy
                << ",penalty_cost=" << block.penalty
                << ",time_cost=" << block.time_cost
                << ",tail_prior_cost=" << block.tail_prior << "\n";
    }
    return result.success ? 0 : 2;
  }
  const bool ok = planner->planTOGT(track);
  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::cout << std::fixed << std::setprecision(9)
            << "native_success=" << (ok ? 1 : 0) << "\n"
            << "gate_count=" << track->gates.size() << "\n"
            << "planning_seconds=" << seconds << "\n";
  if (ok) {
    std::cout << "flight_time_s=" << planner->getTrajectory().getTotalDuration() << "\n"
              << "objective=" << planner->getLastObjective() << "\n"
              << "energy_cost=" << planner->getLastEnergyCost() << "\n"
              << "penalty_cost=" << planner->getLastPenaltyCost() << "\n"
              << "time_cost=" << planner->getLastTimeCost() << "\n"
              << "tail_prior_cost=" << planner->getLastTailPriorCost() << "\n";
  }
  return ok ? 0 : 2;
}
