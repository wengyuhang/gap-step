#include "drolib/solver/minco_snap.hpp"
#include "trajectory.hpp"
#include <casadi/casadi.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
using casadi::DM;
using casadi::Function;
namespace fs = std::filesystem;
Json read(const fs::path &p) {
    Json j;
    std::ifstream f(p);
    if (!f)
        throw std::runtime_error("Missing " + p.string());
    f >> j;
    return j;
}
int main(int argc, char **argv) {
    try {
        if (argc != 4)
            throw std::runtime_error("Usage: wang_verify GRAPH_ROOT COMMON_INPUTS OUTPUT");
        const fs::path root = argv[1], inputs = argv[2];
        const auto protocol = read(inputs / "PROTOCOL.json");
        Json rows = Json::array();
        int graph_count=0;
        for (const auto &lj : protocol["levels"]) {
            const std::string level = lj;
            const auto path = root / (level + "_wang2023");
            const auto m = read(path / "manifest.json");
            double coefficient_error = 0., gradient_error = 0., center_error = 0.;
            for (int gate = 0; gate < static_cast<int>(m.at("windows").size()); ++gate) {
                auto details = Function::load(
                    (path / ("details_" + std::to_string(gate) + ".casadi")).string());
                auto fg =
                    Function::load((path / ("fg_" + std::to_string(gate) + ".casadi")).string());
                const auto seed = m["checks"][gate]["seed"].get<std::vector<double>>(),
                           p = m["checks"][gate]["parameters"].get<std::vector<double>>();
                const auto values = details(std::vector<DM>{DM(seed), DM(p)});
                const auto c = values[0].get_elements(), times = values[1].get_elements();
                drolib::PVAJ head, tail;
                for (int d = 0; d < 4; ++d)
                    for (int axis = 0; axis < 3; ++axis) {
                        head(axis, d) = p[1 + 3 * d + axis];
                        tail(axis, d) = p[13 + 3 * d + axis];
                    }
                const auto w = load_window(m["windows"][gate]);
                const Vector point = w.center + w.rotation * w.motion(p[0] + times[0]);
                drolib::PVAJ crossing;
                crossing.col(0) = point;
                for (int d = 1; d < 4; ++d)
                    for (int axis = 0; axis < 3; ++axis)
                        crossing(axis, d) = seed[2 + 3 * (d - 1) + axis] * 6.;
                for (int leg = 0; leg < 2; ++leg) {
                    drolib::MincoSnap original;
                    original.setConditions(leg == 0 ? head : crossing, leg == 0 ? crossing : tail,
                                           4);
                    Eigen::Matrix3Xd middle(3, 3);
                    for (int k = 0; k < 3; ++k)
                        for (int axis = 0; axis < 3; ++axis)
                            middle(axis, k) = 10. * seed[11 + 9 * leg + 3 * k + axis];
                    Eigen::VectorXd spans = Eigen::VectorXd::Constant(4, times[leg] / 4.);
                    original.setParameters(middle, spans);
                    for (int k = 0; k < 4; ++k)
                        for (int power = 0; power < 8; ++power)
                            for (int axis = 0; axis < 3; ++axis) {
                                const double expected = original.getCoeffs()(8 * k + power, axis) *
                                                        std::pow(spans[k], power),
                                             actual = c[axis * 64 + 8 * (leg * 4 + k) + power];
                                coefficient_error =
                                    std::max(coefficient_error, std::abs(actual - expected) /
                                                                    (1. + std::abs(expected)));
                            }
                }
                const auto check = read(path / ("gradient_" + std::to_string(gate) + ".json"));
                gradient_error = std::max(gradient_error, check["relative_error"].get<double>());
                Trajectory curve;
                for (int leg = 0; leg < 2; ++leg)
                    for (int k = 0; k < 4; ++k) {
                        curve.durations.push_back(times[leg] / 4.);
                        Curve co;
                        for (int power = 0; power < 8; ++power)
                            for (int axis = 0; axis < 3; ++axis)
                                co(power, axis) = c[axis * 64 + 8 * (leg * 4 + k) + power];
                        curve.coefficients.push_back(co);
                    }
                center_error = std::max(center_error, (curve.sample(times[0]) - point).norm());
            }
            graph_count += m.at("windows").size();
            const bool passed =
                coefficient_error < 1e-7 && gradient_error < 2e-4 && center_error < 1e-8;
            if (!passed)
                throw std::runtime_error("Wang independent MINCO/gradient verification failed " +
                                         level + " coefficient " +
                                         std::to_string(coefficient_error));
            rows.push_back(
                Json{{"level", level},
                     {"passed", passed},
                     {"coefficient_relative_error_against_author_minco", coefficient_error},
                     {"gradient_relative_error", gradient_error},
                     {"dynamic_center_error_m", center_error}});
        }
        std::ofstream(argv[3]) << Json{{"passed", true}, {"graphs", graph_count}, {"rows", rows}}.dump(2)
                               << '\n';
        std::cout << "Verified " << graph_count
                  << " Wang graphs against independent author MINCO and dynamic center equalities\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
