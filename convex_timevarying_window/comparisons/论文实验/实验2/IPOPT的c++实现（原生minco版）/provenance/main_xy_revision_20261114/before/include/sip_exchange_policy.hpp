#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace sip_exchange {
inline constexpr double default_finite_radius = 0.63094472;
inline constexpr double geometry_tolerance = 1e-5;
inline bool finite(const std::vector<double>& values) {
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}
struct Snapshot {
    std::vector<double> variables, bound_duals, physical;
    std::map<std::string,double> constraint_duals;
    double objective = std::numeric_limits<double>::infinity();
    std::string status;
    int round = -1;
};
inline bool converged(const Snapshot& state) { return state.status == "Solve_Succeeded"; }
inline std::vector<double> constraint_duals(const Snapshot& state, const std::vector<std::string>& keys) {
    std::vector<double> result(keys.size(),0.);
    if (converged(state)) for (size_t k=0;k<keys.size();++k) {
        const auto found=state.constraint_duals.find(keys[k]);
        if (found!=state.constraint_duals.end() && std::isfinite(found->second)) result[k]=found->second;
    }
    return result;
}
inline std::vector<double> bound_duals(const Snapshot& state) {
    return converged(state) && finite(state.bound_duals) ? state.bound_duals : std::vector<double>(state.variables.size(),0.);
}
inline bool dual_warm_start_allowed(const std::string& previous_status,const std::string& source_status) {
    return previous_status=="Solve_Succeeded" && source_status=="Solve_Succeeded";
}
inline std::vector<double> constraint_duals(const Snapshot& state,const std::vector<std::string>& keys,
                                            const std::string& previous_status) {
    return dual_warm_start_allowed(previous_status,state.status) ? constraint_duals(state,keys) : std::vector<double>(keys.size(),0.);
}
inline std::vector<double> bound_duals(const Snapshot& state,const std::string& previous_status) {
    return dual_warm_start_allowed(previous_status,state.status) ? bound_duals(state) : std::vector<double>(state.variables.size(),0.);
}
inline bool accept_verified(const Snapshot& state,double minimum,bool geometry_passed,bool dynamics_passed) {
    return converged(state) && std::isfinite(minimum) && minimum>=-geometry_tolerance &&
        std::isfinite(state.objective) && finite(state.variables) && geometry_passed && dynamics_passed;
}
class Incumbent {
    std::optional<Snapshot> best_;
public:
    bool consider(const Snapshot& candidate,double minimum) {
        if (!converged(candidate) || !std::isfinite(minimum) || minimum < -geometry_tolerance ||
                !std::isfinite(candidate.objective) || !finite(candidate.variables) || !finite(candidate.physical)) return false;
        if (best_ && candidate.objective >= best_->objective) return false;
        best_=candidate;
        return true;
    }
    bool has_value() const { return best_.has_value(); }
    const Snapshot& value() const { return best_.value(); }
    bool should_restore(const Snapshot& candidate) const {
        return best_ && (!converged(candidate) || !std::isfinite(candidate.objective) || candidate.objective > best_->objective);
    }
};
class GeometryRecovery {
    int rollbacks_ = 0;
public:
    bool retry(bool has_incumbent) {
        if (!has_incumbent || rollbacks_ >= 2) return false;
        ++rollbacks_;
        return true;
    }
    void clear() { rollbacks_=0; }
    int count() const { return rollbacks_; }
};
struct WitnessCandidate {
    int gate, piece;
    std::string kind;
    double offset, fraction, severity;
};
inline bool new_witness(const std::vector<double>& values,double offset) {
    return std::isfinite(offset) && std::all_of(values.begin(),values.end(),[&](double old) { return std::abs(old-offset)>1e-5; });
}
class WorstWitnessPerGate {
    std::map<int,WitnessCandidate> worst_;
public:
    void consider(const WitnessCandidate& candidate) {
        const auto found=worst_.find(candidate.gate);
        if (found==worst_.end() || candidate.severity > found->second.severity) worst_.insert_or_assign(candidate.gate,candidate);
    }
    const std::map<int,WitnessCandidate>& values() const { return worst_; }
};
} // namespace sip_exchange
