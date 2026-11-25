#include "mesh.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>

namespace fs = std::filesystem;

Json read(const fs::path &path) {
    std::ifstream stream(path);
    if (!stream)
        throw std::runtime_error("Missing " + path.string());
    Json value;
    stream >> value;
    return value;
}

void save(const fs::path &path, const Json &value) {
    fs::create_directories(path.parent_path());
    std::ofstream stream(path);
    if (!stream)
        throw std::runtime_error("Cannot write " + path.string());
    stream << value.dump(2) << '\n';
}

int main(int argc, char **argv) {
    try {
        if (argc != 5)
            throw std::runtime_error(
                "Usage: recheck_body ORIGINAL_RESULTS INPUTS COLLISION_MODEL NEW_OUTPUT");
        const fs::path original = fs::absolute(argv[1]), inputs = fs::absolute(argv[2]),
                       model_file = fs::absolute(argv[3]), out = fs::absolute(argv[4]);
        if (fs::exists(out))
            throw std::runtime_error("Use a new output directory");
        const auto model = read(model_file);
        const double body_radius = model.at("physical_collision_radius_m");
        const double safety_radius = model.at("planning_safety_sphere_radius_m");
        if (!(body_radius > 0. && body_radius <= safety_radius))
            throw std::runtime_error("Invalid body and safety radii");
        Json rows = read(original / "SUMMARY.json"), comparisons = Json::array(),
             collisions = Json::array(), counts = Json::object(), ground_counts = Json::object();
        if (rows.size() != 50)
            throw std::runtime_error("Expected fifty source rows");
        const std::vector<std::string> levels{
            "static", "mixed_base", "a25_f3", "a25_f6", "a50_f1",
            "a50_f3", "a50_f6", "a75_f1", "a75_f3", "a75_f6"};
        const std::vector<std::string> methods{
            "ours", "center", "dynamic_togt", "dynamic_togt_center", "wang2023"};
        std::map<std::string, Json> scenes;
        for (const auto &level : levels) {
            auto scene = read(inputs / "assets" / level / "scene.json");
            scene["collision_radius_m"] = body_radius;
            scene["physical_collision_radius_m"] = body_radius;
            scene["planning_safety_sphere_radius_m"] = safety_radius;
            scene["vehicle"]["physical_collision_radius_m"] = body_radius;
            save(out / "assets" / level / "scene.json", scene);
            scenes[level] = std::move(scene);
        }
        for (auto &row : rows) {
            const std::string level = row.at("level"), method = row.at("method");
            const fs::path raw_path = row.at("source").get<std::string>();
            const auto raw = read(raw_path);
            const auto previous = read(raw_path.parent_path() / "assessment.json");
            const auto curve = Trajectory::load(raw);
            if (std::abs(curve.end() - row.at("trajectory_time_s").get<double>()) > 1e-8)
                throw std::runtime_error("Source duration mismatch");
            auto assessment = mesh_evaluate(curve, scenes.at(level));
            assessment["physical_collision_radius_m"] = body_radius;
            assessment["planning_safety_sphere_radius_m"] = safety_radius;
            assessment["source_trajectory"] = raw_path.string();
            const fs::path assessment_path = out / level / method / "assessment.json";
            save(assessment_path, assessment);
            std::string outcome = "证据不足";
            if (assessment.at("collision_detected").get<bool>())
                outcome = "碰撞";
            else if (!assessment.at("ordered_exactly_once").get<bool>() ||
                     !assessment.at("endpoints_passed").get<bool>())
                outcome = "未完成";
            else if (assessment.at("passed").get<bool>())
                outcome = "通过";
            const auto prior_outcome = row.at("outcome");
            row["outcome"] = outcome;
            row["collision_detected"] = assessment.at("collision_detected");
            row["physical_collision_radius_m"] = body_radius;
            row["planning_safety_sphere_radius_m"] = safety_radius;
            row["assessment_source"] = assessment_path.string();
            row["previous_safety_ball_outcome"] = prior_outcome;
            row["safety_sphere_overlap_detected"] = previous.at("collision_detected");
            row["physical_recheck_s"] = assessment.at("check_s");
            // Preserve the measured cost of producing the original frozen trajectory.
            row["original_solve_and_safety_check_wall_s"] = row.at("compute_wall_s");
            row["compute_with_physical_recheck_s"] =
                row.at("compute_wall_s").get<double>() + assessment.at("check_s").get<double>();
            double ground = INFINITY;
            for (const auto &piece : assessment.at("ground"))
                ground = std::min(ground, piece.at("minimum_clearance_m").get<double>());
            row["minimum_ground_clearance_m"] = ground;
            row["ground_contact"] = ground < -1e-8;
            if (!counts.contains(method)) {
                counts[method] = Json::object();
                ground_counts[method] = 0;
            }
            counts[method][outcome] = counts[method].value(outcome, 0) + 1;
            ground_counts[method] = ground_counts[method].get<int>() + (ground < -1e-8);
            comparisons.push_back(Json{{"level", level}, {"method", method},
                                       {"before", prior_outcome}, {"after", outcome},
                                       {"source", raw_path.string()},
                                       {"full_trajectory_time_s", curve.end()}});
            for (const auto &check : assessment.at("checks"))
                if (check.at("outcome") == "collision")
                    collisions.push_back(Json{{"level", level}, {"method", method},
                                              {"gate", check.at("gate")},
                                              {"piece", check.at("piece")},
                                              {"witness", check.at("witness")}});
        }
        save(out / "SUMMARY.json", rows);
        save(out / "MODEL.json", model);
        save(out / "COMPARISON.json", comparisons);
        save(out / "COLLISION_WITNESSES.json", collisions);
        save(out / "STATISTICS.json", Json{{"outcomes", counts}, {"ground_contacts", ground_counts}});
        save(out / "PROTOCOL.json",
             Json{{"planning_safety_sphere_radius_m", safety_radius},
                  {"physical_collision_radius_m", body_radius},
                  {"source_results", original.string()},
                  {"source_inputs", inputs.string()},
                  {"collision_model_source", model_file.string()},
                  {"new_optimizer_calls", 0}, {"rechecked_trajectories", rows.size()},
                  {"method_of_recheck", "C++ mesh_evaluate, same gate meshes and motion"}});
        auto group = [&](const std::string &level) {
            std::map<std::string, Json> result;
            for (const auto &row : rows)
                if (row.at("level") == level)
                    result[row.at("method").get<std::string>()] = row;
            if (result.size() != 5)
                throw std::runtime_error("Missing or duplicate method");
            return result;
        };
        std::ofstream report(out / "PLANNING.md");
        report << "# 五方法十档，规划安全球与机体碰撞体分别评估\n\n"
               << "2026-11-15。规划安全球半径" << safety_radius
               << "米；物理碰撞按现有QuadA仿真碰撞体，球半径" << body_radius
               << "米。原作者未公开含桨完整外廓，本次采用已有仿真代理尺寸。"
                  "前六实体门、原第7航点、窗口运动、全部轨迹及求解参数沿用原记录。\n\n"
                  "## 完整轨迹时间与机体碰撞\n\n"
                  "单位秒，包含第7航点到终点的末段。碰撞单元格保留完整轨迹时间。"
                  "窗框、任务与端点检查通过后参与最短时间加粗。地面相交另记。\n\n"
                  "|幅度比|频率倍率|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|Wang独立复现|\n"
                  "|---|---|---:|---:|---:|---:|---:|\n";
        for (const auto &level : levels) {
            const auto g = group(level);
            double best = INFINITY;
            for (const auto &[method, row] : g)
                if (row.at("outcome") == "通过")
                    best = std::min(best, row.at("trajectory_time_s").get<double>());
            const std::string amplitude = level == "static" ? "0" :
                level == "mixed_base" || level.substr(0, 3) == "a25" ? "0.25" :
                level.substr(0, 3) == "a50" ? "0.50" : "0.75";
            const std::string frequency = level == "static" || level == "mixed_base" ? "1" : level.substr(5);
            report << '|' << amplitude << '|' << frequency << '|';
            for (const auto &method : methods) {
                const auto &row = g.at(method);
                const double time = row.at("trajectory_time_s");
                const bool bold = row.at("outcome") == "通过" && std::abs(time - best) < 1e-9;
                report << (bold ? "**" : "") << std::fixed << std::setprecision(3) << time
                       << (bold ? "**" : "");
                if (row.at("outcome") != "通过")
                    report << "（" << row.at("outcome").get<std::string>() << "）";
                report << '|';
            }
            report << '\n';
        }
        report << "\n## 原始计算墙钟\n\n"
                  "单位秒，沿用实际产生各条轨迹时的求解及安全球复查计时，"
                  "本文为八初值并行等待时间。公共输入和CasADi图预生成未计入。"
                  "新增物理碰撞复查耗时另存SUMMARY.json的physical_recheck_s。\n\n"
                  "|条件|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|Wang独立复现|\n"
                  "|---|---:|---:|---:|---:|---:|\n";
        for (const auto &level : levels) {
            const auto g = group(level);
            report << '|' << level << '|';
            for (const auto &method : methods)
                report << std::fixed << std::setprecision(3)
                       << g.at(method).at("compute_wall_s").get<double>() << '|';
            report << '\n';
        }
        report << "\n## 统计与来源\n\n";
        const std::map<std::string, std::string> names{
            {"ours", "本文自由点"}, {"center", "本文中心"},
            {"dynamic_togt", "动态TOGT自由点"}, {"dynamic_togt_center", "动态TOGT中心"},
            {"wang2023", "Wang独立复现"}};
        for (const auto &method : methods)
            report << "- " << names.at(method) << "通过" << counts.at(method).value("通过", 0)
                   << "/10，窗框碰撞" << counts.at(method).value("碰撞", 0)
                   << "/10，地面相交另记" << ground_counts.at(method).get<int>() << "/10。\n";
        report << "\nMODEL.json记录仿真碰撞体来源，COMPARISON.json逐项保存前后判定。"
                  "SUMMARY.json保留原始轨迹来源、计算时间、物理碰撞与安全球相交两个字段。"
                  "Wang的角速范数15rad/s与本文/TOGT的XY15、Z2.99形式差异沿用原协议。"
                  "本次为已有名义曲线的C++几何重查。\n";
        if (!report)
            throw std::runtime_error("Cannot write report");
        std::cout << counts.dump(2) << '\n' << "GROUND " << ground_counts.dump() << '\n';
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
