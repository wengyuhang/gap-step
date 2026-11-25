#pragma once
#include "drolib/planner/traj_solver.hpp"
#include "drolib/shape/ball.h"
#include "drolib/shape/rectangle.h"
#include "mesh.hpp"
#include <filesystem>
#include <fstream>
#include <memory>

namespace togt_source {
namespace fs = std::filesystem;
inline Json read(const fs::path &path) {
    std::ifstream file(path);
    if (!file)
        throw std::runtime_error("Cannot read " + path.string());
    Json value;
    file >> value;
    return value;
}
inline void save(const fs::path &path, const Json &value) {
    fs::create_directories(path.parent_path());
    std::ofstream(path) << value.dump(2) << '\n';
}

// Author maps are defined in the gate-local frame. Only their world pose varies.
class MovingShape final : public drolib::ShapeBase {
    Window window_;
    std::shared_ptr<drolib::ShapeBase> local_;
    bool disk_ = false;
    Eigen::VectorXd originalCoordinates(const Eigen::VectorXd &d) const {
        if (!disk_)
            return d;
        Eigen::Vector3d full = Eigen::Vector3d::Zero();
        full.head<2>() = d;
        return full;
    }

  public:
    MovingShape(const Window &window, bool center, double centerline_inset) : window_(window) {
        name = center ? "MovingFixedCenter" : "MovingAuthorShape";
        if (center) {
            dim = 0;
        } else if (window.waypoint || window.circle) {
            auto ball = std::make_shared<drolib::Ball>();
            ball->position.setZero();
            ball->radius = window.waypoint ? window.waypoint_radius
                                           : window.opening_radius + .085 - centerline_inset;
            if (!(ball->radius > 0.))
                throw std::runtime_error("Empty ball or disk domain");
            ball->margin = 0.;
            ball->initialize();
            local_ = ball;
            disk_ = !window.waypoint;
            dim = disk_ ? 2 : 3;
        } else {
            auto rectangle = std::make_shared<drolib::Rectangle>();
            rectangle->position.setZero();
            rectangle->R_wb.setIdentity();
            Eigen::Vector2d lower = window.physical.front(), upper = lower;
            for (const auto &p : window.physical) {
                lower = lower.cwiseMin(p);
                upper = upper.cwiseMax(p);
            }
            if (window.physical.size() != 4 || (upper + lower).norm() > 1e-10)
                throw std::runtime_error(
                    "Author Rectangle adapter requires centered rectangular gates");
            rectangle->height = upper.x() - lower.x();
            rectangle->width = upper.y() - lower.y();
            rectangle->marginW = rectangle->marginH = 2. * (centerline_inset - .085);
            rectangle->calVertices();
            rectangle->initialize();
            local_ = rectangle;
            dim = rectangle->dimension();
        }
        setTime(0.);
    }
    void setTime(double time) {
        position = window_.center + window_.rotation * window_.motion(time);
    }
    Eigen::Vector3d velocity(double time) const {
        return window_.rotation * window_.motion_velocity(time);
    }
    Eigen::Vector3d toP(const Eigen::VectorXd &d) const override {
        if (!local_)
            return position;
        return position + window_.rotation * local_->toP(originalCoordinates(d));
    }
    Eigen::VectorXd toD(const Eigen::Vector3d &point) const override {
        if (!local_)
            return Eigen::VectorXd(0);
        return local_->toD(window_.rotation.transpose() * (point - position)).head(dim);
    }
    void getGradD(const Eigen::VectorXd &d, const Eigen::Matrix3Xd &gradP, int i, int j,
                  Eigen::Map<Eigen::VectorXd> &gradD) const override {
        if (!local_)
            return;
        Eigen::Matrix3Xd local_gradient(3, 1);
        local_gradient.col(0) = window_.rotation.transpose() * gradP.col(i);
        Eigen::VectorXd full(local_->dimension());
        Eigen::Map<Eigen::VectorXd> mapped(full.data(), full.size());
        local_->getGradD(originalCoordinates(d), local_gradient, 0, 0, mapped);
        gradD.segment(j, dim) = full.head(dim);
    }
};

class Planner {
    Json scene_, vehicle_;
    bool center_;
    std::vector<std::shared_ptr<MovingShape>> shapes_;

  public:
    drolib::TrajSolver solver;
    drolib::TrajParams parameters;
    drolib::LbfgsParams optimizer;
    drolib::QuadManifold quad;
    drolib::PVAJ head = drolib::PVAJ::Zero(), tail = drolib::PVAJ::Zero();
    Planner(const Json &scene, const Json &vehicle, const fs::path &directory, bool center,
            bool enable_motion = true)
        : scene_(scene), vehicle_(vehicle), center_(center) {
        if (!parameters.load(directory / "init/cpc_planning.yaml") ||
            !optimizer.load(directory / "init/cpc_lbfgs.yaml"))
            throw std::runtime_error("Cannot load original TOGT parameters");
        parameters.speedGuess = vehicle.at("initial_speed_guess_mps");
        parameters.minThr = vehicle.at("rotor_min_n");
        parameters.maxThr = vehicle.at("rotor_max_n");
        parameters.thrMean = .5 * (parameters.minThr + parameters.maxThr);
        parameters.thrRadi = .5 * (parameters.maxThr - parameters.minThr);
        parameters.thrRadiSqr = parameters.thrRadi * parameters.thrRadi;
        parameters.collectivtThrMean = 4. * parameters.thrMean;
        parameters.collectivtThrRadi = 4. * parameters.thrRadi;
        parameters.collectivtThrRadiSqr =
            parameters.collectivtThrRadi * parameters.collectivtThrRadi;
        parameters.maxOmgXYSqr = vehicle.at("rate_squared_limits")[0];
        parameters.maxOmgZSqr = vehicle.at("rate_squared_limits")[1];
        parameters.maxOmgXY = std::sqrt(parameters.maxOmgXYSqr);
        parameters.maxOmgZ = std::sqrt(parameters.maxOmgZSqr);
        drolib::QuadParams qp;
        qp.name = vehicle.value("name", std::string("Quadrotor"));
        qp.mass = vehicle.at("mass");
        for (int i = 0; i < 3; ++i)
            qp.inertia[i] = vehicle.at("inertia")[i];
        qp.T_mb.resize(4, 4);
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                qp.T_mb(i, j) = vehicle.at("inverse_mixer")[i][j];
        qp.T_bm = qp.T_mb.inverse();
        quad = drolib::QuadManifold(qp);
        drolib::TrajData data;
        for (const auto &row : scene.at("windows")) {
            auto shape = std::make_shared<MovingShape>(load_window(row), center,
                                                       vehicle.at("finite_safety_radius_m"));
            shapes_.push_back(shape);
            data.waypoints.emplace_back(shape);
        }
        data.allocateSpace();
        for (int i = 0; i < 3; ++i) {
            head(i, 0) = scene.at("start")[i];
            tail(i, 0) = scene.at("goal")[i];
        }
        data.initData(head.col(0), tail.col(0), parameters.speedGuess);
        if (!solver.setInitialGuess(data))
            throw std::runtime_error("Original TrajData initialization failed");
        solver.setConstYawBeforeTilt();
        solver.quad = quad;
        solver.tparams = parameters;
        solver.minco.setConditions(head, tail, data.totalPieces);
#ifndef TOGT_STATIC_REFERENCE
        if (enable_motion)
            solver.updateWaypointMotion = [this](const Eigen::VectorXd &times,
                                                 Eigen::Matrix3Xd &velocities) {
                velocities.resize(3, shapes_.size());
                double arrival = 0.;
                for (size_t i = 0; i < shapes_.size(); ++i) {
                    arrival += times[i];
                    shapes_[i]->setTime(arrival);
                    velocities.col(i) = shapes_[i]->velocity(arrival);
                }
            };
#else
        if (enable_motion)
            throw std::runtime_error("Original static solver cannot enable dynamic motion");
#endif
    }
    double evaluate(const Eigen::VectorXd &x, Eigen::VectorXd &gradient) {
        gradient.resize(x.size());
        return drolib::TrajSolver::costFunction(&solver, x, gradient);
    }
    void setArchivedTimes(const Json &manifest, int index) {
        if (!center_ || manifest.at("n").get<int>() != solver.data.totalPieces)
            throw std::runtime_error("Archived timing requires a matching center problem");
        const auto seed = manifest.at("initials").at(index).at("original");
        double arrival = 0.;
        for (int i = 0; i < solver.data.totalPieces; ++i) {
            solver.data.T[i] = std::exp(seed.at(i).get<double>());
            if (!std::isfinite(solver.data.T[i]) || solver.data.T[i] <= 0.)
                throw std::runtime_error("Invalid archived duration");
            if (i < solver.data.getNumPoints()) {
                arrival += solver.data.T[i];
                shapes_[i]->setTime(arrival);
                solver.data.P.col(i) = shapes_[i]->position;
            }
        }
        solver.data.calcInitialVal();
    }
    void setCenterInitial(const Json &result) {
        if (center_ || !result.value("found", false))
            throw std::runtime_error("Invalid explicit center initialization");
        if (result.at("durations").size() != static_cast<size_t>(solver.data.totalPieces))
            throw std::runtime_error("Center timing size mismatch");
        double arrival = 0.;
        for (int i = 0; i < solver.data.totalPieces; ++i)
            solver.data.T[i] = result.at("durations")[i];
        for (size_t i = 0; i < shapes_.size(); ++i) {
            arrival += solver.data.T[i];
            shapes_[i]->setTime(arrival);
            solver.data.P.col(i) = shapes_[i]->position;
        }
        solver.data.calcInitialVal();
    }
    Json solve() {
        const auto start = std::chrono::steady_clock::now();
        const auto initial_x = solver.data.x;
        Eigen::VectorXd gradient;
        const double initial_objective = evaluate(initial_x, gradient);
        const double initial_time = solver.data.T.sum();
        const bool success = solver.solve(head, tail, quad, parameters, optimizer);
        const double final_objective = evaluate(solver.data.x, gradient);
        Json coefficients = Json::array(), points = Json::array(), times = Json::array();
        double arrival = 0.;
        for (int piece = 0; piece < solver.data.totalPieces; ++piece) {
            Curve curve;
            for (int power = 0; power < 8; ++power)
                curve.row(power) = solver.minco.getCoeffs().row(8 * piece + power) *
                                   std::pow(solver.data.T[piece], power);
            coefficients.push_back(curve_json(curve));
        }
        for (int i = 0; i < solver.data.getNumPoints(); ++i) {
            const auto p = solver.data.P.col(i);
            points.push_back({p.x(), p.y(), p.z()});
            arrival += solver.data.T[i];
            times.push_back(arrival);
        }
        Json result{
            {"found", success},
            {"method", center_ ? "dynamic_togt_center" : "dynamic_togt"},
            {"implementation",
             "author TrajData and TrajSolver source with optional moving-waypoint chain rule"},
            {"initialization_origin",
             "author TrajData::initData distance/speedGuess; independent single start"},
            {"status", static_cast<int>(solver.status)},
            {"objective", final_objective},
            {"initial_objective", initial_objective},
            {"initial_trajectory_time_s", initial_time},
            {"nominal_time_s", solver.data.T.sum()},
            {"durations", std::vector<double>(solver.data.T.data(),
                                              solver.data.T.data() + solver.data.T.size())},
            {"coefficients", coefficients},
            {"crossing_points", points},
            {"crossing_times_s", times},
            {"variables", std::vector<double>(solver.data.x.data(),
                                              solver.data.x.data() + solver.data.x.size())},
            {"initial_variables",
             std::vector<double>(initial_x.data(), initial_x.data() + initial_x.size())},
            {"optimizer_variables", solver.data.x.size()},
            {"point_variables", solver.data.spatialVarDim},
            {"gradient_inf_norm", gradient.cwiseAbs().maxCoeff()},
            {"solve_s",
             std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count()},
            {"vehicle", vehicle_},
            {"global_planning", true},
            {"replanning", false},
            {"config",
             {{"lbfgs_past", optimizer.params.past},
              {"lbfgs_delta", optimizer.params.delta},
              {"lbfgs_gradient_tolerance", optimizer.params.g_epsilon},
              {"lbfgs_max_iterations", optimizer.params.max_iterations},
              {"time_weight", parameters.weightTime},
              {"energy_weight", parameters.weightEnergy},
              {"thrust_weight", parameters.weightThr},
              {"bodyrate_weight", parameters.weightOmg},
              {"quadrature_intervals", parameters.numConstPena}}}};
#ifndef TOGT_STATIC_REFERENCE
        result["optimizer_return_code"] = solver.optimizerReturnCode;
        result["optimizer_message"] = drolib::lbfgs_strerror(solver.optimizerReturnCode);
        result["iterations"] = solver.optimizerIterations;
        result["evaluations"] = solver.functionEvaluations;
#endif
        return result;
    }
};
} // namespace togt_source
