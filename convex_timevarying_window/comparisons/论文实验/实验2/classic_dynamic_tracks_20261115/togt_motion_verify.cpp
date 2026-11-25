#include "adapter.hpp"
#include <iostream>
using togt_source::Planner;
int main(int argc, char **argv) {
    try {
        if (argc != 5)
            throw std::runtime_error("Usage: verify SCENE VEHICLE PARAMETERS OUTPUT");
        const auto moving = togt_source::read(argv[1]), vehicle = togt_source::read(argv[2]);
        Json stationary = moving;
        for (auto &w : stationary["windows"])
            w["amplitude"] = 0.;
        Json checks = Json::array(), differences = Json::array();
        bool passed = true;
        for (bool center : {false, true}) {
            Planner dynamic(moving, vehicle, argv[3], center);
            Planner zero_motion(stationary, vehicle, argv[3], center);
            Planner static_path(stationary, vehicle, argv[3], center, false);
            double value_error = 0., gradient_error = 0., finite_error = 0., point_error = 0.,
                   point_gradient = 0.;
            const Eigen::VectorXd initial_durations = dynamic.solver.data.T;
            for (int trial = 0; trial < 4; ++trial) {
                Eigen::VectorXd x = dynamic.solver.data.x;
                const int n = dynamic.solver.data.temporalVarDim;
                Eigen::VectorXd durations = initial_durations;
                for (int k = 0; k < n; ++k)
                    durations[k] *= (.13 + .015 * trial) * (1. + .07 * std::sin(k + .3));
                Eigen::Map<Eigen::VectorXd> times(x.data(), n);
                drolib::TrajSolver::backwardT(durations, times);
                for (int k = n; k < x.size(); ++k)
                    x[k] += .04 * std::sin(k + .4 * trial);
                Eigen::VectorXd a, b, g;
                const double av = zero_motion.evaluate(x, a), bv = static_path.evaluate(x, b);
                value_error = std::max(value_error, std::abs(av - bv) / (1. + std::abs(bv)));
                gradient_error = std::max(gradient_error, (a - b).cwiseAbs().maxCoeff());
                dynamic.evaluate(x, g);
                if (!center)
                    point_gradient =
                        std::max(point_gradient, g.tail(x.size() - n).cwiseAbs().maxCoeff());
                for (int k = 0; k < x.size(); ++k) {
                    double best = INFINITY;
                    Json steps = Json::array();
                    for(double scale : {1e-4,1e-5,1e-6,1e-7}) {
                        const double h=scale*(1.+std::abs(x[k]));
                        auto high=x,low=x;high[k]+=h;low[k]-=h;
                        Eigen::VectorXd scratch;
                        const double upper=dynamic.evaluate(high,scratch),lower=dynamic.evaluate(low,scratch);
                        const double numerical=(upper-lower)/(2*h);
                        const double relative=std::abs(numerical-g[k])/(1.+std::abs(numerical)+std::abs(g[k]));
                        steps.push_back(Json{{"h",h},{"upper_value",upper},{"lower_value",lower},{"numerical",numerical},{"relative_error",relative}});
                        best=std::min(best,relative);
                    }
                    finite_error=std::max(finite_error,best);
                    differences.push_back(Json{{"center",center},{"trial",trial},{"coordinate",k},{"analytic",g[k]},
                        {"best_relative_error",best},{"steps",steps}});
                }
                dynamic.evaluate(x, g);
                double arrival = 0.;
                for (int k = 0; k < dynamic.solver.data.getNumPoints(); ++k) {
                    arrival += dynamic.solver.data.T[k];
                    const auto w = load_window(moving["windows"][k]);
                    const Vector point = dynamic.solver.data.P.col(k);
                    if (center)
                        point_error =
                            std::max(point_error,
                                     (point - w.center - w.rotation * w.motion(arrival)).norm());
                    else if (!w.waypoint)
                        point_error =
                            std::max(point_error,
                                     std::abs((w.rotation.transpose() * (point - w.center)).z()));
                }
            }
            const bool ok = value_error < 1e-12 && gradient_error < 1e-10 && finite_error < 2e-4 &&
                            point_error < 1e-8 && (center || point_gradient > 1e-4);
            passed = passed && ok;
            checks.push_back(Json{{"center", center},
                                  {"passed", ok},
                                  {"stationary_value_error", value_error},
                                  {"stationary_gradient_error", gradient_error},
                                  {"dynamic_gradient_relative_error", finite_error},
                                  {"pose_error_m", point_error},
                                  {"active_point_gradient", point_gradient}});
        }
        togt_source::save(argv[4], Json{{"passed", passed}, {"checks", checks}, {"finite_difference_step_sweep", differences}});
        std::cout << checks.dump(2) << '\n';
        return passed ? 0 : 1;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
