#include "drolib/solver/minco_snap.hpp"
#include "restored_kernels.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
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
int main(int argc, char **argv) {
    try {
        if (argc != 4)
            throw std::runtime_error("Usage: inputs TEN_LEVEL_INPUTS POINT7_INPUTS NEW_OUTPUT");
        const fs::path old = argv[1], point7 = argv[2], out = fs::absolute(argv[3]);
        if (fs::exists(out))
            throw std::runtime_error("Use a new output");
        const double sphere = .30, tube = .085, gap = 1e-4;
        const auto physical_scene = read(point7 / "assets/static/scene.json");
        const double physical_radius = physical_scene.at("collision_radius_m");
        if (!(physical_radius > 0. && physical_radius <= sphere))
            throw std::runtime_error("Invalid physical collision radius");
        auto vehicle = read(point7 / "vehicle.json");
        vehicle["physical_collision_radius_m"] = physical_radius;
        vehicle["physical_collision_geometry"] = "sphere";
        vehicle["planning_safety_sphere_radius_m"] = sphere;
        vehicle["planning_proxy_radius_m"] = sphere;
        vehicle["nominal_point_inset_m"] = sphere;
        vehicle["extra_point_clearance_m"] = sphere - physical_radius;
        vehicle["finite_safety_radius_m"] = sphere + tube + gap;
        vehicle["verification_safety_radius_m"] = sphere + tube;
        vehicle["collision_proxy_note"] =
            "planning safety sphere 0.30m; collision uses the configured simulation body sphere, recorded independently";
        save(out / "vehicle.json", vehicle);
        const auto seventh = read(point7 / "assets/static/scene.json")["windows"][6];
        const std::vector<std::string> levels{"static", "mixed_base", "a25_f3", "a25_f6", "a50_f1",
                                              "a50_f3", "a50_f6",     "a75_f1", "a75_f3", "a75_f6"};
        Json preflight = Json::array();
        for (const auto &level : levels) {
            auto scene = read(old / "assets" / level / "scene.json");
            scene["windows"][6] = seventh;
            scene["vehicle"] = vehicle;
            scene["collision_radius_m"] = physical_radius;
            scene["physical_collision_radius_m"] = physical_radius;
            scene["planning_safety_sphere_radius_m"] = sphere;
            scene["scenario_variant"] = "six_dynamic_gates_original_ball7_common_safety_sphere030";
            scene["geometry_sources"]["nominal_inset_per_side_m"] = sphere;
            save(out / "assets" / level / "scene.json", scene);
            for (const std::string method : {"ours", "center"}) {
                const auto source = old / "inputs" / (level + "_" + method);
                auto m = read(source / "manifest.json");
                native_minco::remove_domain_bounds(m);
                const int n = m["n"], count = m["m"];
                const bool free = m["free_points"];
                const int old_vs = m["velocity_start"];
                m["windows"] = scene["windows"];
                for (auto &w : m["windows"])
                    w.erase("mesh");
                m["vehicle"] = vehicle;
                m["finite_radius"] = sphere + tube + gap;
                m["verification_radius"] = sphere + tube;
                m["config"]["radius"] = sphere;
                m["config"]["witness_slack"] = gap;
                m["scenario_variant"] = scene["scenario_variant"];
                if (free) {
                    m["velocity_start"] = old_vs + 1;
                    m["nvar"] = m["nvar"].get<int>() + 1;
                    m["slices"][6]["end"] = old_vs + 1;
                    m["slices"][6]["navigation_waypoint"] = true;
                    const int prefix = count + m["point_domain_rows"].get<int>() - 3;
                    for (const char *key : {"lower_g", "upper_g"}) {
                        auto &g = m[key];
                        g.erase(g.begin() + prefix, g.begin() + prefix + 3);
                    }
                    m["point_domain_rows"] = m["point_domain_rows"].get<int>() - 3;
                }
                m["point_scale_m"] = Json::array();
                m["point_lower_m"] = Json::array();
                m["point_upper_m"] = Json::array();
                for (const auto &w : m["windows"]) {
                    const auto domain = planar_points::domain(load_window(w), sphere + gap);
                    m["point_scale_m"].push_back(std::vector<double>(
                        domain.scale.data(), domain.scale.data() + domain.scale.size()));
                    m["point_lower_m"].push_back(std::vector<double>(
                        domain.lower.data(), domain.lower.data() + domain.lower.size()));
                    m["point_upper_m"].push_back(std::vector<double>(
                        domain.upper.data(), domain.upper.data() + domain.upper.size()));
                }
                m["point_erosion_radius_m"] = sphere + gap;
                m["finite_safety_sphere_radius_m"] = sphere + gap;
                m["verification_safety_sphere_radius_m"] = sphere;
                m["body_sphere_radius_m"] = physical_radius;
                m["physical_collision_radius_m"] = physical_radius;
                m["planning_safety_sphere_radius_m"] = sphere;
                drolib::PVAJ head = drolib::PVAJ::Zero(), tail = head;
                for (int axis = 0; axis < 3; ++axis) {
                    head(axis, 0) = scene["start"][axis];
                    tail(axis, 0) = scene["goal"][axis];
                }
                drolib::MincoSnap minco;
                minco.setConditions(head, tail, n);
                for (auto &seed : m["initials"]) {
                    auto x = seed["original"].get<std::vector<double>>();
                    if (free)
                        x.insert(x.begin() + old_vs, 0.);
                    Eigen::VectorXd times(n);
                    Eigen::Matrix3Xd points(3, count);
                    double at = 0.;
                    for (int i = 0; i < n; ++i) {
                        times[i] = std::exp(x[i]);
                        at += times[i];
                        if (i < count) {
                            auto w = load_window(m["windows"][i]);
                            points.col(i) = w.center + w.rotation * w.motion(at);
                        }
                    }
                    minco.setParameters(points, times);
                    const auto &c = minco.getCoeffs();
                    const int vs = m["velocity_start"];
                    for (int d = 1; d <= 3; ++d)
                        for (int i = 0; i < count; ++i)
                            for (int axis = 0; axis < 3; ++axis) {
                                double v = 0.;
                                for (int power = 7; power >= d; --power) {
                                    double factor = 1.;
                                    for (int j = 0; j < d; ++j)
                                        factor *= power - j;
                                    v = v * times[i] + factor * c(8 * i + power, axis);
                                }
                                x[vs + 3 * ((d - 1) * count + i) + axis] = v;
                            }
                    seed["original"] = x;
                    seed["finite"] = std::vector<double>(x.size() + m["slack_scale"].size(), 0.);
                }
                planar_points::validate(m);
                auto dyn = read(source / "exchange_dynamics.json");
                m["dynamic_residual_scale"] = dyn["residual_scale"];
                std::vector<Window> windows;
                for (const auto &w : m["windows"])
                    windows.push_back(load_window(w));
                auto node = restored_kernels::dynamic_node(dyn);
                native_minco::Graph graph(m, scene, windows, node, 1000.);
                if (graph.expressions[6].numel() != m["lower_g"].size())
                    throw std::runtime_error("Finite constraint dimensions mismatch");
                const auto dest = out / "inputs" / (level + "_" + method);
                save(dest / "manifest.json", m);
                save(dest / "exchange_dynamics.json", dyn);
                node.save((dest / "exchange_dynamic_node.casadi").string());
                preflight.push_back(Json{{"level", level},
                                         {"method", method},
                                         {"starts", m["initials"].size()},
                                         {"nvar", m["nvar"]},
                                         {"finite_rows", m["lower_g"].size()},
                                         {"sphere_radius_m", sphere}});
            }
        }
        save(out / "PREFLIGHT.json", Json{{"passed", true}, {"checks", preflight}});
        save(out / "PROTOCOL.json", Json{{"levels", levels},
                                         {"safety_sphere_radius_m", sphere},
                                         {"physical_collision_radius_m", physical_radius},
                                         {"physical_frame_tube_radius_m", tube},
                                         {"numerical_finite_margin_m", gap},
                                         {"our_starts_per_method", 8},
                                         {"our_independent_starts", true},
                                         {"togt_starts_per_method", 1},
                                         {"waypoint7_radius_m", .3},
                                         {"dynamics_model", "QuadA"},
                                         {"planning_only", true}});
        std::cout
            << "Generated twenty independent native PVAJ inputs; 0.30m planning safety sphere and independent physical collision radius\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
