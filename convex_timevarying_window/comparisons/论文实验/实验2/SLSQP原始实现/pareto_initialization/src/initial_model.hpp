#pragma once
#include <casadi/casadi.hpp>
#include <libcmaes/cmaes.h>
#include <drolib/solver/minco_snap.hpp>
#include "geometry.hpp"
#include <chrono>
#include <filesystem>
#include <numeric>
#include <tuple>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

Json read_json(const fs::path& path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("Cannot read " + path.string());
    Json value;
    stream >> value;
    return value;
}

struct Quality {
    bool finite = false;
    bool feasible = false;
    double maximum = 1e100;
    double mean_square = 1e100;
    double duration = 1e100;
    int violated = 0;
    int physical_violated = 0;
    double physical_maximum = 0.;
    auto key() const {
        return feasible ? std::make_tuple(0, duration, 0., 0.)
                        : std::make_tuple(1, maximum, mean_square, duration);
    }
    Json json() const {
        return Json{{"finite", finite}, {"sampled_feasible", feasible}, {"maximum_violation", maximum},
                    {"mean_square_violation", mean_square}, {"trajectory_time_s", duration},
                    {"violated_rows", violated}, {"physical_violated_rows", physical_violated},
                    {"physical_maximum_violation", physical_maximum}};
    }
};

struct Candidate {
    std::vector<double> log_times;
    std::vector<double> times;
    std::vector<double> arrivals;
    std::vector<double> derivatives;
    std::vector<double> coefficients;
    Quality quality;
    Json json() const {
        return Json{{"log_times", log_times}, {"durations", times}, {"arrivals", arrivals},
                    {"VAJ", derivatives}, {"coefficients_flat", coefficients}, {"quality", quality.json()}};
    }
};

class Model {
    Json metadata_;
    casadi::Function constraints_;
    std::vector<Window> windows_;
    drolib::MincoSnap minco_;
    std::vector<double> lower_, upper_;
    int n_, m_, physical_rows_;
    double tolerance_;
public:
    explicit Model(const fs::path& input)
        : metadata_(read_json(input / "model.json")),
          constraints_(casadi::Function::load((input / "constraints.casadi").string())) {
        n_ = metadata_["n"];
        m_ = metadata_["m"];
        physical_rows_ = metadata_["physical_rows"];
        tolerance_ = metadata_["tolerance"];
        lower_ = metadata_["lower"].get<std::vector<double>>();
        upper_ = metadata_["upper"].get<std::vector<double>>();
        for (const auto& window : metadata_["windows"]) windows_.push_back(load_window(window));
        Eigen::Matrix<double,3,4> head = Eigen::Matrix<double,3,4>::Zero();
        Eigen::Matrix<double,3,4> tail = head;
        for (int axis = 0; axis < 3; ++axis) {
            head(axis,0) = metadata_["start"][axis];
            tail(axis,0) = metadata_["goal"][axis];
        }
        minco_.setConditions(head, tail, n_);
    }
    const Json& metadata() const { return metadata_; }
    Candidate evaluate(const std::vector<double>& log_times) {
        Candidate result;
        result.log_times = log_times;
        result.times.resize(n_);
        result.arrivals.resize(m_);
        Eigen::VectorXd times(n_);
        double bound_violation = 0.;
        double arrival = 0.;
        Eigen::Matrix3Xd points(3,m_);
        for (int piece = 0; piece < n_; ++piece) {
            const double duration = std::exp(log_times[piece]);
            if (!std::isfinite(duration) || duration < 1e-8) return result;
            result.times[piece] = times[piece] = duration;
            bound_violation += std::max(lower_[piece]-duration, 0.) + std::max(duration-upper_[piece], 0.);
            arrival += duration;
            if (piece < m_) {
                result.arrivals[piece] = arrival;
                points.col(piece) = windows_[piece].center + windows_[piece].rotation*windows_[piece].motion(arrival);
            }
        }
        result.quality.duration = arrival;
        if (bound_violation > 1e-10) {
            result.quality.maximum = 1e6 + bound_violation;
            return result;
        }
        minco_.setParameters(points, times);
        const auto& raw = minco_.getCoeffs();
        if (!raw.allFinite()) return result;
        result.coefficients.resize(24*n_);
        for (int piece = 0; piece < n_; ++piece)
            for (int axis = 0; axis < 3; ++axis)
                for (int power = 0; power < 8; ++power)
                    result.coefficients[24*piece+8*axis+power] = raw(8*piece+power,axis)*std::pow(times[piece],power);
        const auto residuals = constraints_(std::vector<casadi::DM>{casadi::DM(result.coefficients),casadi::DM(result.times)})[0].get_elements();
        result.quality.maximum = 0.;
        result.quality.mean_square = 0.;
        for (size_t row = 0; row < residuals.size(); ++row) {
            if (!std::isfinite(residuals[row])) return Candidate{};
            const double violation = std::max(-residuals[row],0.);
            result.quality.maximum = std::max(result.quality.maximum,violation);
            result.quality.mean_square += violation*violation;
            if (violation > tolerance_) {
                ++result.quality.violated;
                if (static_cast<int>(row) < physical_rows_) ++result.quality.physical_violated;
            }
            if (static_cast<int>(row) < physical_rows_)
                result.quality.physical_maximum = std::max(result.quality.physical_maximum,violation);
        }
        result.quality.mean_square /= residuals.size();
        result.quality.finite = true;
        result.quality.feasible = result.quality.maximum <= tolerance_;
        result.derivatives.resize(9*m_);
        for (int order = 1; order <= 3; ++order)
            for (int gate = 0; gate < m_; ++gate)
                for (int axis = 0; axis < 3; ++axis) {
                    double derivative = 0.;
                    for (int power = 7; power >= order; --power) {
                        double factor = 1.;
                        for (int k = 0; k < order; ++k) factor *= power-k;
                        derivative = derivative*times[gate]+factor*raw(8*gate+power,axis);
                    }
                    result.derivatives[3*((order-1)*m_+gate)+axis] = derivative;
                }
        return result;
    }
};

