#include "drolib/solver/minco_snap.hpp"
#include "trajectory.hpp"
#include <casadi/casadi.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
using casadi::DM;
using casadi::Function;
using casadi::SX;
namespace fs = std::filesystem;
Json read(const fs::path &p) {
    std::ifstream f(p);
    if (!f)
        throw std::runtime_error("Missing " + p.string());
    Json j;
    f >> j;
    return j;
}
void save(const fs::path &p, const Json &j) {
    fs::create_directories(p.parent_path());
    std::ofstream(p) << j.dump(2) << '\n';
}
DM dense(const Eigen::MatrixXd &m) {
    DM a = DM::zeros(m.rows(), m.cols());
    for (int i = 0; i < m.rows(); ++i)
        for (int j = 0; j < m.cols(); ++j)
            a(i, j) = m(i, j);
    return a;
}
Eigen::VectorXd basis(double u, int d) {
    Eigen::VectorXd b = Eigen::VectorXd::Zero(8);
    for (int k = d; k < 8; ++k)
        b[k] = std::tgamma(k + 1.) / std::tgamma(k - d + 1.) * std::pow(u, k - d);
    return b;
}
DM inverse() {
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(32, 32);
    int row = 0;
    for (int d = 0; d < 4; ++d)
        a.block(row++, 0, 1, 8) = basis(0., d).transpose();
    for (int i = 0; i < 3; ++i) {
        a.block(row++, 8 * i, 1, 8) = basis(1., 0).transpose();
        a.block(row++, 8 * (i + 1), 1, 8) = basis(0., 0).transpose();
        for (int d = 1; d < 7; ++d) {
            a.block(row, 8 * i, 1, 8) = basis(1., d).transpose();
            a.block(row++, 8 * (i + 1), 1, 8) = -basis(0., d).transpose();
        }
    }
    for (int d = 0; d < 4; ++d)
        a.block(row++, 24, 1, 8) = basis(1., d).transpose();
    if (row != 32)
        throw std::runtime_error("MINCO row count");
    return dense(a.inverse());
}
SX coefficients(const SX &head, const SX &tail, const SX &middle, const SX &duration,
                const DM &inv) {
    const SX h = duration / 4.;
    std::vector<SX> rhs;
    for (int d = 0; d < 4; ++d)
        rhs.push_back(head(casadi::Slice(), d) * SX::pow(h, d));
    for (int i = 0; i < 3; ++i) {
        rhs.push_back(middle(casadi::Slice(), i));
        rhs.push_back(middle(casadi::Slice(), i));
        for (int d = 1; d < 7; ++d)
            rhs.push_back(SX::zeros(3, 1));
    }
    for (int d = 0; d < 4; ++d)
        rhs.push_back(tail(casadi::Slice(), d) * SX::pow(h, d));
    return SX::mtimes(inv, SX::horzcat(rhs).T());
}
SX position(const Window &w, const SX &t) {
    const SX phase = 2. * pi * w.frequency * t + w.phase;
    SX motion;
    if (w.swing) {
        const SX angle = std::asin(w.amplitude / w.length) * SX::sin(phase);
        motion =
            SX::vertcat({w.length * (1. - SX::cos(angle)), -w.length * SX::sin(angle), SX(0.)});
    } else
        motion = SX::vertcat({SX(0.), -w.amplitude * SX::sin(phase), SX(0.)});
    if (w.motion_angle != 0.) {
        const double c = std::cos(w.motion_angle), s = std::sin(w.motion_angle);
        motion = SX::vertcat({c * motion(0) - s * motion(1),
                                s * motion(0) + c * motion(1), motion(2)});
    }
    return SX(DM(std::vector<double>{w.center.x(), w.center.y(), w.center.z()})) +
           SX::mtimes(dense(w.rotation), motion);
}
void graph(const Window &w, const Json &vehicle, const fs::path &folder, int gate,
           const Json &check, const Json &config) {
    const int dimension = 29;
    const SX x = SX::sym("local_variables", dimension), p = SX::sym("local_parameters", 25);
    SX head = SX::reshape(p(casadi::Slice(1, 13)), 3, 4),
       final = SX::reshape(p(casadi::Slice(13, 25)), 3, 4);
    const SX times = SX::exp(x(casadi::Slice(0, 2)));
    const auto inv = inverse();
    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(8, 8);
    for (int i = 4; i < 8; ++i)
        for (int j = 4; j < 8; ++j)
            q(i, j) = std::tgamma(i + 1.) / std::tgamma(i - 3.) * std::tgamma(j + 1.) /
                      std::tgamma(j - 3.) / (i + j - 7.);
    const DM energy = dense(q);
    SX stamp = p(0), effort = 0., thrust_penalty = 0., rate_penalty = 0.;
    std::vector<SX> blocks, tr, br;
    for (int leg = 0; leg < 2; ++leg) {
        const SX duration = times(leg), h = duration / 4.;
        const SX tail = leg == 0 ? SX::horzcat({position(w, stamp + duration),
                                                SX::reshape(x(casadi::Slice(2, 11)) * 6., 3, 3)})
                                 : final;
        const SX middle = SX::reshape(x(casadi::Slice(11 + 9 * leg, 20 + 9 * leg)) * 10., 3, 3);
        const SX c = coefficients(head, tail, middle, duration, inv);
        blocks.push_back(c);
        for (int i = 0; i < 4; ++i) {
            const SX piece = c(casadi::Slice(8 * i, 8 * (i + 1)), casadi::Slice());
            effort += SX::trace(SX::mtimes({piece.T(), energy, piece})) / SX::pow(h, 7);
        }
        for (int node = 0; node <= 32; ++node) {
            const double fraction = double(node) * 4 / 32;
            const int piece = std::min(int(fraction), 3);
            const double u = fraction - piece;
            const SX co = c(casadi::Slice(8 * piece, 8 * (piece + 1)), casadi::Slice());
            std::vector<SX> state;
            for (int d = 0; d < 4; ++d)
                state.push_back(SX::mtimes(co.T(), dense(basis(u, d))) / SX::pow(h, d));
            const SX force = state[2] + SX(DM(std::vector<double>{0., 0., vehicle["gravity"]}));
            const SX norm = SX::sqrt(SX::sumsqr(force)), axis = force / norm;
            const SX dz = (state[3] - axis * SX::dot(axis, state[3])) / norm;
            const SX ratio = dz(2) / (1. + axis(2));
            const SX omega = SX::vertcat({-dz(1) + axis(1) * ratio, dz(0) - axis(0) * ratio,
                                          (axis(1) * dz(0) - axis(0) * dz(1)) / (1. + axis(2))});
            const SX thrust = vehicle["mass"].get<double>() * norm;
            const double low = 4 * vehicle["rotor_min_n"].get<double>(),
                         high = 4 * vehicle["rotor_max_n"].get<double>();
            const SX gt = SX::pow(thrust - .5 * (low + high), 2) - std::pow(.5 * (high - low), 2);
            const SX gb = SX::sumsqr(omega) - std::pow(config["body_rate_max"].get<double>(), 2);
            tr.push_back(gt);
            br.push_back(gb);
            const SX weight = duration / 32. * ((node == 0 || node == 32) ? .5 : 1.);
            thrust_penalty += weight * SX::pow(SX::fmax(gt, 0.), 3);
            rate_penalty += weight * SX::pow(SX::fmax(gb, 0.), 3);
        }
        head = tail;
        stamp += duration;
    }
    const SX terms = SX::vertcat({effort, 1e4 * SX::sum1(times), 100. * thrust_penalty,
                                  100. * rate_penalty, SX(0.), SX(0.)});
    const SX objective = SX::sum1(terms) / 1e4;
    const std::string label = std::to_string(gate);
    Function fg("fg_" + label, {x, p}, {objective, SX::gradient(objective, x)}),
        details("details_" + label, {x, p},
                {SX::vertcat(blocks), times, terms,
                 SX::vertcat(
                     {SX::mmax(SX::vertcat(tr)), SX::mmax(SX::vertcat(br)), SX(-1.), SX(-1.)})});
    fg.save((folder / ("fg_" + label + ".casadi")).string());
    details.save((folder / ("details_" + label + ".casadi")).string());
    const auto seed = check["seed"].get<std::vector<double>>(),
               parameters = check["parameters"].get<std::vector<double>>();
    const auto values = fg(std::vector<DM>{DM(seed), DM(parameters)});
    const auto analytic = values[1].get_elements();
    double error = 0.;
    Json finite_difference_steps = Json::array();
    for (int k = 0; k < dimension; ++k) {
        double best = INFINITY;
        Json steps = Json::array();
        for (double relative_step : {1e-3, 1e-4, 1e-5, 1e-6}) {
            auto plus = seed, minus = seed;
            const double step = relative_step * (1. + std::abs(seed[k]));
            plus[k] += step;
            minus[k] -= step;
            const double numeric =
                (static_cast<double>(fg(std::vector<DM>{DM(plus), DM(parameters)})[0]) -
                 static_cast<double>(fg(std::vector<DM>{DM(minus), DM(parameters)})[0])) /
                (2. * step);
            const double difference =
                std::abs(numeric - analytic[k]) / (1. + std::abs(numeric) + std::abs(analytic[k]));
            steps.push_back(
                Json{{"step", step}, {"numeric", numeric}, {"relative_error", difference}});
            best = std::min(best, difference);
        }
        finite_difference_steps.push_back(Json{{"coordinate", k},
                                               {"analytic", analytic[k]},
                                               {"best_relative_error", best},
                                               {"steps", steps}});
        error = std::max(error, best);
    }
    if (error > 2e-4)
        throw std::runtime_error("Wang objective derivative failed gate " + label + " error " +
                                 std::to_string(error));
    save(folder / ("gradient_" + label + ".json"),
         Json{{"passed", true},
              {"relative_error", error},
              {"objective", static_cast<double>(values[0])},
              {"gradient", analytic},
              {"finite_difference_steps", finite_difference_steps}});
}
int main(int argc, char **argv) {
    try {
        if (argc != 3)
            throw std::runtime_error("Usage: wang_inputs COMMON_INPUTS OUTPUT");
        const fs::path in = argv[1], out = fs::absolute(argv[2]);
        if (fs::exists(out))
            throw std::runtime_error("Use a new graph directory");
        const auto vehicle = read(in / "vehicle.json"), protocol = read(in / "PROTOCOL.json");
        for (const auto &level_json : protocol["levels"]) {
            const std::string level = level_json;
            const auto scene = read(in / "assets" / level / "scene.json");
            const int gate_count=scene.at("windows").size(), segment_count=gate_count+1;
            std::vector<Window> windows;
            for (const auto &w : scene["windows"])
                windows.push_back(load_window(w));
            Vector start(scene["start"][0], scene["start"][1], scene["start"][2]),
                goal(scene["goal"][0], scene["goal"][1], scene["goal"][2]);
            std::vector<Vector> anchors{start};
            for (const auto &w : windows)
                anchors.push_back(w.center);
            anchors.push_back(goal);
            double length = 0., maximum = 0.;
            for (size_t i = 1; i < anchors.size(); ++i) {
                double d = (anchors[i] - anchors[i - 1]).norm();
                length += d;
                maximum = std::max(maximum, d);
            }
            const double step = std::max({1., length / 6.5 / segment_count, std::sqrt(3 * maximum / 8.)});
            Eigen::VectorXd times = Eigen::VectorXd::Constant(segment_count, step);
            Eigen::Matrix3Xd points(3, gate_count);
            for (int i = 0; i < gate_count; ++i)
                points.col(i) =
                    windows[i].center + windows[i].rotation * windows[i].motion((i + 1) * step);
            drolib::PVAJ head = drolib::PVAJ::Zero(), tail = head;
            head.col(0) = start;
            tail.col(0) = goal;
            drolib::MincoSnap minco;
            minco.setConditions(head, tail, segment_count);
            minco.setParameters(points, times);
            Trajectory route;
            route.durations = std::vector<double>(segment_count, step);
            for (int i = 0; i < segment_count; ++i) {
                Curve c;
                for (int power = 0; power < 8; ++power)
                    c.row(power) = minco.getCoeffs().row(8 * i + power) * std::pow(step, power);
                route.coefficients.push_back(c);
            }
            Json co = Json::array();
            for (const auto &c : route.coefficients)
                co.push_back(curve_json(c));
            Json nodes = Json::array(), tails = Json::array(), tail_times = Json::array(),
                 choices = Json::array(), normals = Json::array(), depths = Json::array();
            for (int i = 0; i <= segment_count; ++i)
                nodes.push_back(i * step);
            const double depth = vehicle["planning_proxy_radius_m"].get<double>() + .085 + 1e-6;
            for (int i = 0; i < gate_count; ++i) {
                Vector normal =
                    -scene["windows"][i]["sign"].get<double>() * windows[i].rotation.col(2);
                if (windows[i].waypoint)
                    normal = (windows[i].center - windows[i - 1].center).normalized();
                normals.push_back({normal.x(), normal.y(), normal.z()});
                depths.push_back(depth);
                Eigen::Matrix<double, 3, 4> state = Eigen::Matrix<double, 3, 4>::Zero();
                if (i+1 == gate_count) {
                    state.col(0) = goal;
                    tail_times.push_back(segment_count * step);
                    choices.push_back("actual_goal_zero_vaj");
                } else {
                    Vector position = .5 * (points.col(i) + points.col(i + 1));
                    position +=
                        normal *
                        std::max(0., 2 * depth - (position - windows[i].center).dot(normal));
                    state.col(0) = position;
                    state.col(1) = 6.5 * (points.col(i + 1) - position).normalized();
                    tail_times.push_back((i + 1.5) * step);
                    choices.push_back("forward_geometric_cruise_state");
                }
                tails.push_back(state_json(state));
            }
            Json reference{{"durations", route.durations}, {"coefficients", co},
                           {"node_times", nodes},          {"tail_states", tails},
                           {"tail_times", tail_times},     {"tail_choices", choices},
                           {"normals", normals},           {"body_exit_depths", depths}};
            Json config{
                {"pieces", 4},
                {"order", 4},
                {"intervals", 32},
                {"rho", 1e4},
                {"lambda_thrust", 100.},
                {"lambda_body_rate", 100.},
                {"lambda_gap", 1e4},
                {"lambda_obstacle", 1e4},
                {"body_rate_max", std::sqrt(vehicle["rate_squared_limits"][0].get<double>())},
                {"thrust_min", 4 * vehicle["rotor_min_n"].get<double>()},
                {"thrust_max", 4 * vehicle["rotor_max_n"].get<double>()},
                {"position_scale", 10.},
                {"derivative_scale", 6.},
                {"solve_budget_ms", 2000.},
                {"max_iterations", 5000},
                {"gradient_tolerance", 1e-5},
                {"relative_tolerance", 1e-6},
                {"radius", vehicle["planning_proxy_radius_m"]},
                {"vehicle", vehicle},
                {"flatness_model", "QuadA zero aerodynamic drag; full three-axis Hopf body rates"}};
            const auto folder = out / (level + "_wang2023");
            fs::create_directories(folder);
            Json checks = Json::array();
            for (int gate = 0; gate < gate_count; ++gate) {
                const double begin = gate * step, middle = (gate + 1) * step,
                             end = tail_times[gate];
                std::vector<double> x(29, 0.), p{0.};
                x[0] = std::log(middle - begin);
                x[1] = std::log(end - middle);
                const auto hs = gate == 0 ? head : ([&]() {
                    Eigen::Matrix<double, 3, 4> v;
                    for (int i = 0; i < 3; ++i)
                        for (int d = 0; d < 4; ++d)
                            v(i, d) = tails[gate - 1][i][d];
                    return v;
                })();
                Eigen::Matrix<double, 3, 4> ts;
                for (int i = 0; i < 3; ++i)
                    for (int d = 0; d < 4; ++d)
                        ts(i, d) = tails[gate][i][d];
                for (const auto &state : {hs, ts})
                    for (int d = 0; d < 4; ++d)
                        for (int axis = 0; axis < 3; ++axis)
                            p.push_back(state(axis, d));
                for (int d = 1; d < 4; ++d) {
                    const Vector v = route.sample(middle, d) / 6.;
                    for (int axis = 0; axis < 3; ++axis)
                        x[2 + 3 * (d - 1) + axis] = v[axis];
                }
                for (int leg = 0; leg < 2; ++leg) {
                    const double a = leg == 0 ? begin : middle, b = leg == 0 ? middle : end;
                    const Vector target =
                        windows[gate].center +
                        windows[gate].rotation * windows[gate].motion(middle - begin);
                    const Vector first = leg == 0 ? Vector(hs.col(0)) : target;
                    const Vector last = leg == 0 ? target : Vector(ts.col(0));
                    for (int k = 1; k < 4; ++k) {
                        const double u = k / 4.;
                        const double blend = 35 * std::pow(u, 4) - 84 * std::pow(u, 5) +
                                             70 * std::pow(u, 6) - 20 * std::pow(u, 7);
                        const Vector v = (route.sample(a + u * (b - a)) +
                                          (1 - blend) * (first - route.sample(a)) +
                                          blend * (last - route.sample(b))) /
                                         10.;
                        for (int axis = 0; axis < 3; ++axis)
                            x[11 + 9 * leg + 3 * (k - 1) + axis] = v[axis];
                    }
                }
                Json check{{"seed", x}, {"parameters", p}};
                graph(windows[gate], vehicle, folder, gate, check, config);
                const auto values = read(folder / ("gradient_" + std::to_string(gate) + ".json"));
                check["objective"] = values["objective"];
                check["gradient"] = values["gradient"];
                checks.push_back(check);
            }
            save(folder / "manifest.json",
                 Json{{"level", level},
                      {"dimension", 29},
                      {"state_end", 11},
                      {"config", config},
                      {"reference", reference},
                      {"windows", scene["windows"]},
                      {"start", scene["start"]},
                      {"settings", {{"temporal_scales", {1., .8, 1.2, 1.5}}, {"align_reference_pvaj", true}}},
                      {"checks", checks},
                      {"scenario_variant", scene["scenario_variant"]}});
            std::cout << "WANG_INPUT " << level << std::endl;
        }
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
