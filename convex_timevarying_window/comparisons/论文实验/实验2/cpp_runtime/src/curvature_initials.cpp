#include "trajectory.hpp"
#include "dynamics_exchange.hpp"
#include "drolib/solver/minco_snap.hpp"
#include "minco_pv.hpp"
#include <chrono>
#include <filesystem>
#include <map>

namespace fs = std::filesystem;
Json read(const fs::path& file) {
    std::ifstream stream(file);
    if (!stream) throw std::runtime_error("Missing " + file.string());
    Json value; stream >> value; return value;
}
void save(const fs::path& file, const Json& value) {
    fs::create_directories(file.parent_path());
    std::ofstream(file) << value.dump(2) << '\n';
}

int main(int argc, char** argv) {
    try {
        const auto began = std::chrono::steady_clock::now();
        if (argc != 4 && argc != 5 && argc != 6) throw std::runtime_error("INPUT_ROOT LEVEL NEW_OUTPUT [--robust-ipopt-profile | --finite-feasible-profile | --finite-feasible-refined-profile | --conservative-profile | --phase-spread-profile | --togt-raw-speed SPEED]");
        const bool robust_ipopt=argc==5 && std::string(argv[4])=="--robust-ipopt-profile";
        const bool refine_feasible=robust_ipopt || (argc==5 && std::string(argv[4])=="--finite-feasible-refined-profile");
        const bool finite_feasible=refine_feasible || (argc==5 && std::string(argv[4])=="--finite-feasible-profile");
        const bool conservative=argc==5 && std::string(argv[4])=="--conservative-profile";
        const bool phase_spread=argc==5 && std::string(argv[4])=="--phase-spread-profile";
        const bool stationary_phase=argc==5 && std::string(argv[4])=="--stationary-phase-profile";
        if(argc==5 && !conservative && !phase_spread && !stationary_phase && !finite_feasible)throw std::runtime_error("Unknown initialization mode");
        const bool togt_raw=argc==6;
        if(togt_raw && std::string(argv[4])!="--togt-raw-speed")throw std::runtime_error("Unknown initialization mode");
        const double raw_speed=togt_raw?std::stod(argv[5]):1.;
        if(!std::isfinite(raw_speed) || raw_speed<=0.)throw std::runtime_error("Invalid raw initial speed");
        const fs::path root = argv[1], output = argv[3];
        const std::string level = argv[2];
        if (fs::exists(output)) throw std::runtime_error("Output already exists");
        const auto scene = read(root / "assets" / level / "scene.json");
        const auto vehicle = read(root / "vehicle.json");
        const dynamics_exchange::Model dynamic_model(read(root / "inputs" / (level+"_center") / "exchange_dynamics.json"));
        if(finite_feasible) {
            const auto hover=dynamic_model.residuals<double>({0.,0.,0.},{0.,0.,0.},{0.,0.,0.});
            for(double r:hover)if(!std::isfinite(r) || r>=0.)throw std::runtime_error("Finite-feasible initialization requires strictly feasible hover dynamics");
        }
        const int m = scene["windows"].size(), n = m + 1;
        drolib::PVAJ head = drolib::PVAJ::Zero(), tail = head;
        for (int axis = 0; axis < 3; ++axis) {
            head(axis, 0) = scene["start"][axis];
            tail(axis, 0) = scene["goal"][axis];
        }
        std::vector<Window> windows;
        std::vector<Vector> path{head.col(0)};
        for (const auto& row : scene["windows"]) {
            windows.push_back(load_window(row));
            path.push_back(windows.back().center+(togt_raw?windows.back().rotation*windows.back().motion(0.):Vector::Zero().eval()));
        }
        path.push_back(tail.col(0));
        std::vector<double> distance;
        for (int k = 0; k < n; ++k) distance.push_back((path[k + 1] - path[k]).norm());
        const double collective = 4. * vehicle["rotor_max_n"].get<double>() / vehicle["mass"].get<double>();
        const double horizontal = std::sqrt(collective * collective - std::pow(vehicle["gravity"].get<double>(), 2));
        if (!std::isfinite(horizontal) || horizontal <= 0.) throw std::runtime_error("Invalid vehicle thrust");
        std::map<std::string, Json> manifests;
        for (std::string method : {"ours", "center"}) {
            auto value = read(root / "inputs" / (level + "_" + method) / "manifest.json");
            if (value["n"] != n || value["m"] != m || value["state_scale"].size() != 9 * m)
                throw std::runtime_error("Expected matching full PVAJ input");
            if(value.value("point_coordinates",std::string())!="scaled_xy" ||
                value["free_points"].get<bool>()!=(method=="ours"))
                throw std::runtime_error("Expected explicit xy free-point and variable-free center inputs");
            value["initials"] = Json::array();
            value["native_soft_formulation"] = "residual_epigraph";
            value["native_finite_solver"] = "ipopt";
            value["solver_options"]["ipopt.hessian_approximation"] = "exact";
            value["solver_options"]["ipopt.nlp_scaling_method"] = "gradient-based";
            value["slack_scale"] = Json::array();
            if(robust_ipopt) {
                value["lift_minco_coefficients"]=true;
                value["bernstein_coefficient_coordinates"]=true;
                value["hermite_dynamics_evaluation"]=true;
                value["warm_start_push"]=1e-9;
                value["maximum_exchange_rounds"]=64;
                value["solver_options"]["ipopt.mu_strategy"]="adaptive";
                value["solver_options"]["ipopt.theta_max_fact"]=1.;
                value["solver_options"]["ipopt.watchdog_shortened_iter_trigger"]=0;
            }

            value["initialization"] = {{"rule", "nominal curvature speed profile; forward/backward acceleration pass; center points; MINCO derivatives"},
                {"speed_caps_mps", {12., 24.}}, {"acceleration_fractions", {.2, .35, .5, .65}}, {"optimizer_calls", 0}};
            if(conservative){value["initialization"]["speed_caps_mps"]={8.,12.};value["initialization"]["acceleration_fractions"]={.04,.08,.12,.16};}
            if(finite_feasible)value["initialization"]["finite_feasibility"]={{"bracket_refinement_steps",refine_feasible?8:0},{"time_backoff_factor",1.25},{"maximum_backoffs",32},{"nodes_per_piece",17},{"maximum_normalized_residual",0.},{"chart_domain_minimum",.001},{"regenerate_moving_centers_and_minco",true},{"initialization_only",true}};
            if(phase_spread)value["initialization"]={{"rule","raw base curvature profile; per-gate Halton arrival offsets; center points; unoptimized MINCO derivatives"},{"halton_index_stride",104729},{"maximum_adjacent_duration_fraction",.2},{"maximum_motion_period_fraction",.25},{"optimizer_calls",0}};
            if(stationary_phase)value["initialization"]={{"rule","original eight nominal curvature profiles; nearest stationary-motion gate phase with positive initial duration; center points; unoptimized MINCO derivatives"},{"minimum_initial_duration_fraction",.5},{"optimized_arrivals_unrestricted",true},{"optimizer_calls",0}};
            if(togt_raw)value["initialization"]={{"rule","TOGT unoptimized t=0 center distances / speed; evaluate actual centers at arrivals; MINCO derivatives"},{"speed_mps",raw_speed},{"optimizer_calls",0}};
            manifests[method] = std::move(value);
        }
        Json audit = Json::array();
        for (int seed = 0; seed < (togt_raw?1:8); ++seed) {
            const int profile_seed=phase_spread?0:seed;
            const double speed = conservative ? (profile_seed%2?12.:8.) : (profile_seed % 2 ? 24. : 12.);
            const double acceleration = horizontal * (conservative?std::array<double,4>{.04,.08,.12,.16}:std::array<double, 4>{.2, .35, .5, .65})[profile_seed / 2];
            std::vector<double> v(n + 1, speed);
            v.front() = v.back() = 0.;
            for (int gate = 1; gate < n; ++gate) {
                const Vector incoming = path[gate] - path[gate - 1], outgoing = path[gate + 1] - path[gate];
                const double cosine = std::clamp(incoming.normalized().dot(outgoing.normalized()), -1., 1.);
                const double curvature = 2. * std::sqrt(.5 * (1. - cosine)) /
                    std::max(1e-6, std::min(incoming.norm(), outgoing.norm()));
                v[gate] = std::min(speed, std::sqrt(acceleration / std::max(curvature, 1e-6)));
            }
            for (int k = 0; k < n; ++k) v[k + 1] = std::min(v[k + 1], std::sqrt(v[k] * v[k] + 2. * acceleration * distance[k]));
            for (int k = n - 1; k >= 0; --k) v[k] = std::min(v[k], std::sqrt(v[k + 1] * v[k + 1] + 2. * acceleration * distance[k]));
            Eigen::VectorXd times(n);
            Eigen::Matrix3Xd points(3, m), states(3, 3 * m);
            for (int k = 0; k < n; ++k) {
                times[k] = togt_raw?distance[k]/raw_speed:2. * distance[k] / std::max(1e-6, v[k] + v[k + 1]);
            }
            std::vector<double> arrival_offsets(m,0.);
            double max_phase_speed_factor=0.;
            if(stationary_phase) {
                const Eigen::VectorXd nominal=times;double nominal_arrival=0.,previous=0.;
                for(int gate=0;gate<m;++gate) {
                    nominal_arrival+=nominal[gate];
                    double target=std::max(nominal_arrival,previous+.5*nominal[gate]);
                    const auto& w=windows[gate];
                    if(w.frequency>0. && w.amplitude!=0.) {
                        const double omega=2.*pi*w.frequency;
                        const double nearest=std::round((omega*nominal_arrival+w.phase-.5*pi)/pi);
                        const double earliest=std::ceil((omega*(previous+.5*nominal[gate])+w.phase-.5*pi)/pi);
                        target=(.5*pi+std::max(nearest,earliest)*pi-w.phase)/omega;
                        max_phase_speed_factor=std::max(max_phase_speed_factor,std::abs(std::cos(omega*target+w.phase)));
                    }
                    times[gate]=target-previous;arrival_offsets[gate]=target-nominal_arrival;previous=target;
                    if(!(times[gate]>0.))throw std::runtime_error("Stationary-phase initializer produced nonpositive duration");
                }
            }
            if(phase_spread && seed>0){
                int prime=1;
                for(int gate=0;gate<m;++gate){
                    bool composite;
                    do {++prime;composite=false;for(int d=2;d*d<=prime;++d)if(prime%d==0){composite=true;break;}}while(composite);
                    int integer=seed*104729;double radical=0.,power=1.;while(integer){power/=prime;radical+=(integer%prime)*power;integer/=prime;}
                    const double width=std::min(.2*std::min(times[gate],times[gate+1]),.25/std::max(1e-9,windows[gate].frequency));
                    arrival_offsets[gate]=width*(2.*radical-1.);
                }
                for(int k=0;k<n;++k)times[k]+=(k<m?arrival_offsets[k]:0.)-(k?arrival_offsets[k-1]:0.);
            }
            drolib::MincoSnap initial;
            initial.setConditions(head, tail, n);
            Eigen::Matrix<double,Eigen::Dynamic,3> coefficients;
            auto refit=[&]() {
                double arrival=0.;
                for(int k=0;k<n;++k) {
                    arrival+=times[k];
                    if(k<m)points.col(k)=windows[k].center+windows[k].rotation*windows[k].motion(arrival);
                }
                initial.setParameters(points,times);coefficients=initial.getCoeffs();
            };
            refit();
            Json feasibility_trials=Json::array();
            auto test_feasibility=[&](int attempt) {
                double maximum=-INFINITY,minimum_chart=INFINITY;
                bool valid=true;
                for(int piece=0;piece<n;++piece)for(int node=0;node<=16;++node) {
                    std::array<std::array<double,3>,3> derivatives{};
                    const double t=times[piece]*double(node)/16.;
                    for(int order=2;order<=4;++order)for(int axis=0;axis<3;++axis) {
                        double value=0.;
                        for(int power=7;power>=order;--power) {
                            double factor=1.;for(int k=0;k<order;++k)factor*=power-k;
                            value=value*t+factor*coefficients(8*piece+power,axis);
                        }
                        derivatives[order-2][axis]=value;
                    }
                    Vector force(derivatives[0][0],derivatives[0][1],derivatives[0][2]+dynamic_model.gravity);
                    const double chart=1.+force.z()/force.norm();
                    minimum_chart=std::min(minimum_chart,chart);
                    // Reject the legacy numeric evaluator's alternate-chart
                    // fallback rather than treating it as physical feasibility.
                    if(!std::isfinite(chart) || chart<=.001){valid=false;continue;}
                    const auto residual=dynamic_model.residuals<double>(derivatives[0],derivatives[1],derivatives[2]);
                    for(double r:residual){if(!std::isfinite(r))valid=false;else maximum=std::max(maximum,r);}
                }
                const bool accepted=valid && maximum<=0.;
                feasibility_trials.push_back({{"attempt",attempt},{"duration_s",times.sum()},{"maximum_normalized_residual",maximum},{"minimum_chart_denominator",minimum_chart},{"passed",accepted}});
                return accepted;
            };
            int backoffs=0;
            if(finite_feasible)for(int attempt=0;;++attempt) {
                if(test_feasibility(attempt)){backoffs=attempt;break;}
                if(attempt==32)throw std::runtime_error("Unable to construct finite-feasible raw initial within fixed backoff budget");
                times*=1.25;refit();
            }
            if(refine_feasible && backoffs>0) {
                // Retain a verified feasible endpoint. Moving-window feasibility need
                // not be monotone: this finds a local bracket, not a global minimum.
                const Eigen::VectorXd feasible_times=times;
                double lower=.8,upper=1.;
                for(int trial=0;trial<8;++trial) {
                    const double midpoint=.5*(lower+upper);
                    times=midpoint*feasible_times;refit();
                    if(test_feasibility(33+trial))upper=midpoint;else lower=midpoint;
                }
                times=upper*feasible_times;refit();
                if(!test_feasibility(41))throw std::runtime_error("Refined initial lost sampled feasibility");
            }
            for (int order = 1; order <= 3; ++order) for (int gate = 0; gate < m; ++gate) for (int axis = 0; axis < 3; ++axis) {
                double value = 0.;
                for (int power = 7; power >= order; --power) {
                    double factor = 1.; for (int d = 0; d < order; ++d) factor *= power - d;
                    value = value * times[gate] + factor * coefficients(8 * gate + power, axis);
                }
                states(axis, (order - 1) * m + gate) = value;
            }
            togt_pv::MincoPV check(head, tail, n, 3);
            if (!check.fit(points, states, times)) throw std::runtime_error("Initial MINCO reconstruction failed");
            double error = 0., forward = INFINITY;
            for (int k = 0; k < 8 * n; ++k) for (int axis = 0; axis < 3; ++axis)
                error = std::max(error, std::abs(check.coefficients()(k, axis) - coefficients(k, axis)) / (1. + std::abs(coefficients(k, axis))));
            for (int gate = 0; gate < m; ++gate) {
                const Vector normal = -scene["windows"][gate]["sign"].get<double>() * windows[gate].rotation.col(2);
                forward = std::min(forward, normal.dot(states.col(gate)));
            }
            if (error > 1e-6) throw std::runtime_error("Initial physical trajectory mismatch");
            for (auto& entry : manifests) {
                auto& value = entry.second;
                const int nv = value["nvar"], vs = value["velocity_start"];
                std::vector<double> x(nv, 0.), parameters(1 + 2 * n, 0.);
                double arrival = 0.;
                for (int k = 0; k < n; ++k) {
                    x[k] = std::log(times[k]); arrival += times[k];
                    parameters[1 + k] = arrival; parameters[1 + n + k] = 1.;
                }
                parameters[0] = arrival;
                std::copy(states.data(), states.data() + states.size(), x.begin() + vs);
                value["initials"].push_back({{"index", seed}, {"original", x}, {"finite", std::vector<double>(nv, 0.)}, {"parameters", parameters}});
            }
            audit.push_back({{"seed", seed}, {"speed_cap_mps", speed}, {"profile_acceleration_mps2", acceleration},
                {"duration_s", times.sum()}, {"minimum_forward_speed_mps", forward}, {"coefficient_relative_error", error}});
            if(finite_feasible)audit.back()["finite_feasibility_trials"]=feasibility_trials;
            if(phase_spread)audit.back()["arrival_offsets_s"]=arrival_offsets;
            if(stationary_phase){audit.back()["arrival_offsets_s"]=arrival_offsets;audit.back()["maximum_abs_cos_phase"]=max_phase_speed_factor;}
            if(togt_raw) {
                audit.back().erase("speed_cap_mps");
                audit.back().erase("profile_acceleration_mps2");
                audit.back()["raw_distance_speed_mps"]=raw_speed;
            }
        }
        for (const auto& entry : manifests) {
            const auto destination = output / entry.first;
            save(destination / "manifest.json", entry.second);
            fs::copy_file(root / "inputs" / (level + "_" + entry.first) / "exchange_dynamics.json", destination / "exchange_dynamics.json");
        }
        save(output / "INITIALIZATION_AUDIT.json", {{"optimizer_calls", 0}, {"paired_raw_curves_identical", true},
            {"seeds", audit}, {"preparation_wall_s", std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count()}});
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
