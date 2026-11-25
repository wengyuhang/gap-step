#include "dynamic_togt.hpp"
#include "mesh.hpp"
#include "restored_kernels.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
namespace fs = std::filesystem;
using casadi::DM;
using casadi::Function;
using casadi::MX;
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
void require(bool v, const char *message) {
    if (!v)
        throw std::runtime_error(message);
}
std::vector<double> initial(const Json &m) {
    auto x = m["initials"][0]["original"].get<std::vector<double>>();
    const int n = m["n"], vs = m["velocity_start"], nv = m["nvar"];
    for (int i = 0; i < n; ++i) {
        double t = std::exp(x[i]);
        x[i] = t > 1. ? std::sqrt(2. * t - 1.) - 1. : 1. - std::sqrt(2. / t - 1.);
    }
    if (m["free_points"].get<bool>())
        for (int gate = 0; gate < m["m"].get<int>(); ++gate)
            for (int axis = 0; axis < m["point_scale_m"][gate].size(); ++axis)
                x[m["slices"][gate]["begin"].get<int>() + axis] /=
                    m["point_scale_m"][gate][axis].get<double>();
    for (int i = vs; i < nv; ++i)
        x[i] /= m["state_scale"][i - vs].get<double>();
    x.resize(nv + m["slack_scale"].size(), 0.);
    return x;
}
void prepare(const fs::path &old, const fs::path &author, const fs::path &out,
             const fs::path &old_results, const fs::path &check_dir) {
    require(!fs::exists(out), "Use a new input directory");
    auto scene = read(old / "assets/static/scene.json");
    auto previous_scene = scene;
    const fs::path source = author / "resources/racetrack/race_uzh_7g_multiprisma.yaml";
    drolib::Yaml yaml(source);
    require(yaml["Gate7"]["type"].as<std::string>() == "SingleBall",
            "Author seventh object must be SingleBall");
    Vector position;
    yaml["Gate7"]["position"] >> position;
    const double radius = yaml["Gate7"]["radius"].as<double>();
    require(std::abs(radius - .3) < 1e-12, "Unexpected author waypoint radius");
    auto &point = scene["windows"][6];
    point["navigation_waypoint"] = true;
    point["waypoint_radius_m"] = radius;
    point["source_shape"] = "SingleBall";
    point["center"] = {position.x(), position.y(), position.z()};
    point["rotation"] = {{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}};
    point["rpy_radians"] = {0., 0., 0.};
    point["vertices"] = Json::array();
    point["circle"] = false;
    point["swing"] = false;
    point["amplitude"] = 0.;
    point["frequency"] = 0.;
    point.erase("mesh");
    scene["scenario_variant"] = "togt_quada_6g_original_singleball7_static_v1";
    scene["geometry_sources"]["waypoint7_source"] = fs::absolute(source).string();
    scene["geometry_sources"]["waypoint7_semantics"] =
        "3D navigation neighborhood radius 0.3m, no frame/plane/direction/repeated-crossing "
        "restriction";
    for (int i = 0; i < 6; ++i)
        require(scene["windows"][i] == previous_scene["windows"][i], "An earlier gate changed");
    save(out / "assets/static/scene.json", scene);
    save(out / "vehicle.json", read(old / "vehicle.json"));
    for (const std::string method : {"ours", "center"}) {
        const fs::path before = old / "inputs" / ("static_" + method),
                       dest = out / "inputs" / ("static_" + method);
        auto m = read(before / "manifest.json");
        auto old_m = m;
        m["windows"] = scene["windows"];
        m["scenario_variant"] = scene["scenario_variant"];
        m["point_scale_m"][6] = {radius, radius, radius};
        m["point_lower_m"][6] = {-radius, -radius, -radius};
        m["point_upper_m"][6] = {radius, radius, radius};
        m["point_coordinate_description"] =
            "scaled 2D physical gate coordinates and scaled 3D navigation ball coordinates";
        if (m["free_points"].get<bool>()) {
            const int old_vs = m["velocity_start"], old_nv = m["nvar"],
                      erase_at =
                          m["workspace_rows"].get<int>() + m["m"].get<int>() +
                          (m.value("restore_historical_domain", false) ? m["n"].get<int>() : 0) +
                          m["point_domain_rows"].get<int>() - 3;
            m["velocity_start"] = old_vs + 1;
            m["nvar"] = old_nv + 1;
            m["slices"][6]["end"] = old_vs + 1;
            m["slices"][6]["navigation_waypoint"] = true;
            m["point_domain_rows"] = m["point_domain_rows"].get<int>() - 3;
            for (const char *key : {"lower_g", "upper_g"}) {
                auto &g = m[key];
                g.erase(g.begin() + erase_at, g.begin() + erase_at + 3);
            }
            for (auto &seed : m["initials"]) {
                auto x = seed["original"].get<std::vector<double>>();
                x.insert(x.begin() + old_vs, 0.);
                seed["original"] = x;
                seed["finite"] = std::vector<double>(x.size() + m["slack_scale"].size(), 0.);
            }
        }
        planar_points::validate(m);
        for (int seed = 0; seed < 8; ++seed) {
            const auto &a = m["initials"][seed]["original"],
                       &b = old_m["initials"][seed]["original"];
            for (int k = 0; k < m["n"].get<int>(); ++k)
                require(a[k] == b[k], "Time initialization changed");
            for (int k = 0; k < 63; ++k)
                require(a[m["velocity_start"].get<int>() + k] ==
                            b[old_m["velocity_start"].get<int>() + k],
                        "VAJ initialization changed");
            require(m["initials"][seed]["parameters"] == old_m["initials"][seed]["parameters"],
                    "Normalization changed");
        }
        save(dest / "manifest.json", m);
        for (const char *file : {"exchange_dynamics.json", "exchange_dynamic_node.casadi"})
            fs::copy_file(before / file, dest / file);
    }
    Json same_curves = Json::array();
    for (const std::string method : {"ours", "center", "dynamic_togt", "dynamic_togt_center"}) {
        const fs::path file =
            (method == "ours" || method == "center")
                ? old_results / "static/plans" / ("static_" + method) / "result.json"
                : old_results / "static/baselines" / method / "result.json";
        auto result = read(file);
        if (!result.value("found", false))
            continue;
        const auto assessment = mesh_evaluate(Trajectory::load(result), scene);
        save(check_dir / "same_curve_recheck" / method / "assessment.json", assessment);
        same_curves.push_back(
            Json{{"method", method},
                 {"trajectory_unchanged", true},
                 {"source", fs::absolute(file).string()},
                 {"old_verdict", read(file.parent_path() / "assessment.json")["verdict"]},
                 {"point7_verdict", assessment["verdict"]},
                 {"time_s", result["nominal_time_s"]}});
    }
    save(check_dir / "SAME_CURVE_RECHECK.json", same_curves);
    save(out / "PROTOCOL.json", Json{{"changed_object", "Gate7 only"},
                                     {"source", fs::absolute(source).string()},
                                     {"waypoint_radius_m", radius},
                                     {"physical_gate_count", 6},
                                     {"navigation_waypoint_count", 1},
                                     {"static_only", true},
                                     {"our_independent_starts_per_method", 8},
                                     {"initial_times_and_actual_VAJ_unchanged", true},
                                     {"dynamic_togt_original_center_then_free", true},
                                     {"gazebo", false}});
    std::cout << "PREPARED six unchanged frames + original 3D SingleBall waypoint; eight "
                 "initializations preserved\n";
}
void validate(const fs::path &inputs, const fs::path &author, const fs::path &out) {
    auto scene = read(inputs / "assets/static/scene.json"),
         f = read(inputs / "inputs/static_ours/manifest.json"),
         c = read(inputs / "inputs/static_center/manifest.json"),
         dynamic = read(inputs / "inputs/static_ours/exchange_dynamics.json");
    native_minco::remove_domain_bounds(f);
    native_minco::remove_domain_bounds(c);
    require(f["nvar"] == 86 && c["nvar"] == 71, "Unexpected PVAJ dimensions");
    require(f["point_domain_rows"] == 16 && c["point_domain_rows"] == 0,
            "Wrong physical gate/waypoint domains");
    require(!scene["windows"][6].contains("mesh"),
            "Navigation waypoint must have no collision mesh");
    std::vector<Window> windows;
    for (const auto &row : scene["windows"])
        windows.push_back(load_window(row));
    const auto dyn = restored_kernels::dynamic_node(dynamic);
    f["dynamic_residual_scale"] = dynamic["residual_scale"];
    c["dynamic_residual_scale"] = dynamic["residual_scale"];
    native_minco::Graph gf(f, scene, windows, dyn, 1000.), gc(c, scene, windows, dyn, 1000.);
    require(gf.expressions[6].numel() == f["lower_g"].size(), "Finite bound count mismatch");
    require(gc.expressions[6].numel() == c["lower_g"].size(), "Center finite bound count mismatch");
    auto xf = initial(f), xc = initial(c);
    const auto params = f["initials"][0]["parameters"].get<std::vector<double>>();
    auto a = gf.trajectory(std::vector<DM>{DM(xf), DM(params)}),
         b = gc.trajectory(std::vector<DM>{DM(xc), DM(params)});
    double equal = 0.;
    auto ca = a[2].get_elements(), cb = b[2].get_elements();
    for (size_t i = 0; i < ca.size(); ++i)
        equal = std::max(equal, std::abs(ca[i] - cb[i]));
    require(equal < 1e-9, "Center/free initialization must describe the same curve");
    const MX z = gf.expressions[0], p = gf.expressions[1];
    const auto output = gf.trajectory(std::vector<MX>{z, p});
    Function jac("waypoint_coefficient_jacobian", {z, p}, {MX::jacobian(output[2], z)});
    const auto J = jac(std::vector<DM>{DM(xf), DM(params)})[0];
    const int begin = f["slices"][6]["begin"];
    double grad_error = 0.;
    for (int k = begin; k < begin + 3; ++k) {
        auto plus = xf, minus = xf;
        const double h = 1e-5;
        plus[k] += h;
        minus[k] -= h;
        auto up = gf.trajectory(std::vector<DM>{DM(plus), DM(params)})[2].get_elements(),
             dn = gf.trajectory(std::vector<DM>{DM(minus), DM(params)})[2].get_elements();
        for (size_t i = 0; i < up.size(); ++i) {
            const double exact = static_cast<double>(J(i, k));
            grad_error = std::max(grad_error, std::abs(exact - (up[i] - dn[i]) / (2 * h)) /
                                                  (1. + std::abs(exact)));
        }
    }
    require(grad_error < 1e-6, "Waypoint coefficient derivative failed");
    Function domains("waypoint_domain", {z, p}, {gf.expressions[6]});
    auto outside = xf;
    for (int k = begin; k < begin + 3; ++k)
        outside[k] = 1.;
    const auto d = domains(std::vector<DM>{DM(outside), DM(params)})[0].get_elements();
    require(std::abs(d[22] + 2.) < 1e-10, "3D ball must reject bounding-box corner");
    require(std::abs(d[6] - 1.) < 1e-12,
            "No directional velocity condition is allowed at navigation waypoint");
    dynamic_togt::Planner planner(scene, false, author / "parameters/cpc",
                                  read(inputs / "vehicle.json"));
    const auto seed = planner.initial();
    const auto eval = planner.evaluate(seed);
    double togt_error = 0.;
    for (int k = 0; k < seed.size(); ++k) {
        auto plus = seed, minus = seed;
        double h = 1e-6 * (1. + std::abs(seed[k]));
        plus[k] += h;
        minus[k] -= h;
        double numeric =
            (planner.evaluate(plus).objective - planner.evaluate(minus).objective) / (2 * h);
        togt_error = std::max(togt_error, std::abs(numeric - eval.gradient[k]) /
                                              (1. + std::abs(eval.gradient[k])));
    }
    require(togt_error < 2e-5, "TOGT 3D waypoint gradient failed");
    save(out, Json{{"passed", true},
                   {"our_free_variables", 86},
                   {"our_center_variables", 71},
                   {"togt_free_variables", planner.dimension()},
                   {"center_free_initial_coefficient_error", equal},
                   {"waypoint_coefficient_gradient_error", grad_error},
                   {"togt_gradient_error", togt_error},
                   {"first_six_frames_unchanged", true},
                   {"waypoint_mesh_absent", true},
                   {"no_waypoint_direction_constraint", true}});
    std::cout << "PREFLIGHT PASSED native gradient " << grad_error << " TOGT gradient "
              << togt_error << '\n';
}
void report(const fs::path &results, const fs::path &inputs, const fs::path &output) {
    require(read(results / "status.json")["state"] == "complete", "Planning is incomplete");
    auto summary = read(results / "SUMMARY.json");
    const auto scene = read(inputs / "assets/static/scene.json");
    require(summary.size() == 4, "Expected four methods");
    Json candidates = Json::array();
    for (auto &row : summary) {
        const fs::path source = row["source"].get<std::string>();
        const auto result = read(source);
        require(result.value("found", false), "Selected method has no trajectory");
        const auto assessment = mesh_evaluate(Trajectory::load(result), scene);
        require(assessment["passed"].get<bool>(), "Independent selected trajectory check failed");
        require(assessment["checks"].size() == 48,
                "Six physical gates must check all eight pieces");
        double total = 0.;
        for (double time : result["durations"])
            total += time;
        require(std::abs(total - result["nominal_time_s"].get<double>()) < 1e-8,
                "Full trajectory duration mismatch");
        row["trajectory_time_s"] = total;
        row["outcome"] = "通过";
        row["waypoint_distance_m"] = assessment["opening_events"].back()["distance_to_center_m"];
        double ground_clearance = INFINITY;
        for (const auto &ground : assessment["ground"])
            ground_clearance =
                std::min(ground_clearance, ground["minimum_clearance_m"].get<double>());
        row["minimum_ground_clearance_m"] = ground_clearance;
        row["ground_contact"] = ground_clearance < -1e-8;
        save(results / "final_checks" / row["method"].get<std::string>() / "assessment.json",
             assessment);
        if (row["method"] != "ours" && row["method"] != "center")
            continue;
        require(result["candidate_records"].size() == 8, "Expected eight independent candidates");
        for (const auto &candidate_row : result["candidate_records"]) {
            const fs::path candidate_file = candidate_row["source"].get<std::string>();
            const auto candidate = read(candidate_file);
            require(candidate["found"].get<bool>(),
                    "This batch should contain sixteen valid candidates");
            const auto check = mesh_evaluate(Trajectory::load(candidate), scene);
            require(check["passed"].get<bool>(), "Raw independent candidate verification failed");
            std::map<int, Json> calls;
            int previous_revision = -1;
            for (const auto &call : candidate["optimizer_calls"]) {
                const int revision = call["constraint_revision"];
                require(revision > previous_revision, "Unchanged finite problem was solved twice");
                previous_revision = revision;
                calls[call["round"].get<int>()] = call;
                if (call["warm_start_used"].get<bool>())
                    require(call["previous_solve_status"] == "Solve_Succeeded" &&
                                call["dual_source_status"] == "Solve_Succeeded",
                            "Invalid multiplier warm start");
            }
            for (const auto &record : candidate["records"]) {
                const auto &call = calls.at(record["round"].get<int>());
                require(call["status"] == "Solve_Succeeded" ||
                            call["status"] == "Solved_To_Acceptable_Level",
                        "Verification before finite convergence");
                require(call["constraint_violation_inf"].get<double>() <= 1e-5 &&
                            call["variable_bound_violation_inf"].get<double>() <= 1e-5,
                        "Verification before finite feasibility");
                std::set<int> gates;
                for (const auto &witness : record["geometry_added_witnesses"]) {
                    const int gate = witness["gate"];
                    require(gate < 6 && gates.insert(gate).second,
                            "Navigation waypoint received a frame witness or duplicate witness");
                }
            }
            candidates.push_back(Json{{"job", candidate_row["job"]},
                                      {"passed", true},
                                      {"finite_calls", candidate["optimizer_calls"].size()},
                                      {"source", candidate_file.string()}});
        }
    }
    require(candidates.size() == 16, "Expected sixteen audited independent candidates");
    save(results / "FINAL_SUMMARY.json", summary);
    save(results / "CANDIDATE_AUDIT.json", Json{{"passed", true}, {"candidates", candidates}});
    const std::map<std::string, std::string> names{{"ours", "本文自由点"},
                                                   {"center", "本文中心"},
                                                   {"dynamic_togt", "动态TOGT自由点"},
                                                   {"dynamic_togt_center", "动态TOGT中心"}};
    std::ofstream f(output);
    f << "# 第7处恢复原SingleBall航点的静态验证\n\n"
      << "2026-11-15，C++"
         "规划。六个实体门依次为方、圆、方、圆、方、圆，第7处按作者公开混合七门文件恢复为半径0."
         "3米的三维航点邻域。自由点方案可在球内选点，中心方案固定到球心。航点不设门平面方向和重复穿"
         "越限制。完整时间包含起点、六门、第7航点及到作者终点的末段。\n\n"
      << "本文两种方案各8个独立初值，16路并行。八组逐段时间和实际VAJ初值逐值沿用上一批，点初值位于"
         "中心。前六门、QuadA、门余量、目标权重和优化器采用上一批设置。动态TOGT采用原中心后自由的单"
         "初值流程。\n\n"
      << "|方法|窗框及任务检查|完整轨迹时间/s|计算墙钟/s|通过初值|第7航点距球心/m|\n"
      << "|---|---|---:|---:|---:|---:|\n";
    for (const std::string method : {"ours", "center", "dynamic_togt", "dynamic_togt_center"})
        for (const auto &row : summary) {
            if (row["method"] != method)
                continue;
            f << '|' << names.at(method) << "|通过|" << std::fixed << std::setprecision(3)
              << row["trajectory_time_s"].get<double>() << '|'
              << row["compute_wall_s"].get<double>() << '|'
              << row["qualified_starts"].get<std::string>() << '|' << std::setprecision(6)
              << row["waypoint_distance_m"].get<double>() << "|\n";
        }
    f << "\n计算墙钟包括本方法求解及独立检查，本文采用八初值并行结束的实际等待时间。TOGT自由点包含"
         "中心初始化成本。共享输入生成未单独计时。\n\n"
      << "## 原曲线隔离复查\n\n"
      << "旧静态动态TOGT自由点10.010665秒、中心10."
         "010847秒的曲线完全保留，只把第7处按原SingleBall解释，两条曲线均由窗框碰撞变为六门及航点任"
         "务检查通过。该结果定位了旧静态碰撞的来源。重新求解四方法的结果列于上表。\n\n"
      << "旧本文自由点是在第7方形开口内选点，该点超出半径0."
         "3米的原航点邻域，因此旧曲线的任务检查失败。上表本文自由点是满足新航点域后重新求解的结果。"
         "\n\n"
      << "## 地面与证据记录\n\n"
      << "沿用用户既有统计口径，地面相交单列。本文中心的最小名义离地净空为";
    for (const auto &row : summary)
        if (row["method"] == "center")
            f << row["minimum_ground_clearance_m"].get<double>();
    f << "米，有地面相交。另三条选中轨迹的地面检查通过。表内通过指六门窗框、穿越顺序、航点邻域和起"
         "终PVAJ检查。动力学沿用软惩罚，检查为浮点数值证据。本轮只评估规划。\n\n"
      << "最终数据为FINAL_SUMMARY.json。16个初值的独立复查与外层准入审计见CANDIDATE_AUDIT."
         "json。原曲线比较见SAME_CURVE_RECHECK.json，实际运行源码与二进制哈希见FROZEN_PROTOCOL."
         "json。\n";
    std::cout << "REPORT PASSED four methods and sixteen independent candidates\n";
}

int main(int argc, char **argv) {
    try {
        if (argc == 7 && std::string(argv[1]) == "prepare")
            prepare(argv[2], argv[3], argv[4], argv[5], argv[6]);
        else if (argc == 5 && std::string(argv[1]) == "validate")
            validate(argv[2], argv[3], argv[4]);
        else if (argc == 5 && std::string(argv[1]) == "report")
            report(argv[2], argv[3], argv[4]);
        else
            throw std::runtime_error(
                "Usage: point7 prepare OLD_INPUTS AUTHOR NEW_INPUTS OLD_RESULTS CHECK_DIR | "
                "validate INPUTS AUTHOR OUT_JSON | report RESULTS INPUTS OUT_MD");
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
