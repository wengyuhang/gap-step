#include "mesh.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
namespace fs = std::filesystem;
Json read(const fs::path &p) {
    std::ifstream f(p);
    if (!f)
        throw std::runtime_error("Missing " + p.string());
    Json j;
    f >> j;
    return j;
}
void save(const fs::path &p, const Json &j) { std::ofstream(p) << j.dump(2) << '\n'; }
int main(int argc, char **argv) {
    try {
        if (argc != 3)
            throw std::runtime_error("Usage: report RESULT_DIRECTORY INPUT_DIRECTORY");
        const fs::path out = fs::absolute(argv[1]), inputs = fs::absolute(argv[2]);
        if (read(out / "OURS_STATUS.json")["state"] != "complete" ||
            read(out / "BASELINES_STATUS.json")["state"] != "complete")
            throw std::runtime_error("Finish both coordinators before final report");
        const std::vector<std::string> levels{"static", "mixed_base", "a25_f3", "a25_f6", "a50_f1",
                                              "a50_f3", "a50_f6",     "a75_f1", "a75_f3", "a75_f6"},
            methods{"ours", "center", "dynamic_togt", "dynamic_togt_center", "wang2023"};
        const std::map<std::string, std::string> names{{"ours", "本文自由点"},
                                                       {"center", "本文中心"},
                                                       {"dynamic_togt", "动态TOGT自由点"},
                                                       {"dynamic_togt_center", "动态TOGT中心"},
                                                       {"wang2023", "Wang独立复现"}};
        Json rows = read(out / "BASELINE_SUMMARY.json"), audit = Json::array(),
             collisions = Json::array();
        for (const auto &level : levels)
            for (const std::string method : {"ours", "center"}) {
                const auto file = out / level / "plans" / (level + "_" + method) / "result.json";
                const auto raw = read(file);
                int qualified = 0;
                for (const auto &candidate : raw["candidate_records"]) {
                    qualified += candidate["passed"].get<bool>();
                    const auto c = read(candidate["source"].get<std::string>());
                    std::map<int, Json> calls;
                    int revision = -1;
                    for (const auto &call : c["optimizer_calls"]) {
                        if (call["constraint_revision"].get<int>() <= revision)
                            throw std::runtime_error("Unchanged finite problem retried");
                        revision = call["constraint_revision"];
                        calls[call["round"].get<int>()] = call;
                        if (call.value("warm_start_used", false) &&
                            (call["previous_solve_status"] != "Solve_Succeeded" ||
                             call["dual_source_status"] != "Solve_Succeeded"))
                            throw std::runtime_error("Invalid multiplier warm start");
                    }
                    for (const auto &record : c["records"]) {
                        const auto &call = calls.at(record["round"].get<int>());
                        if ((call["status"] != "Solve_Succeeded" &&
                             call["status"] != "Solved_To_Acceptable_Level") ||
                            call["constraint_violation_inf"].get<double>() > 1e-5 ||
                            call["variable_bound_violation_inf"].get<double>() > 1e-5)
                            throw std::runtime_error("Verification before finite readiness");
                        std::set<int> gates;
                        for (const auto &w : record["geometry_added_witnesses"])
                            if (!gates.insert(w["gate"].get<int>()).second)
                                throw std::runtime_error("Multiple witnesses per gate");
                    }
                    audit.push_back(Json{{"job", candidate["job"]},
                                         {"found", c["found"]},
                                         {"termination_reason", c["termination_reason"]},
                                         {"finite_calls", c["optimizer_calls"].size()}});
                }
                const bool found = raw["found"];
                std::string outcome = "未找到合格规划";
                if (found) {
                    const auto a = read(file.parent_path() / "assessment.json");
                    outcome = a["collision_detected"].get<bool>() ? "碰撞"
                              : !a["ordered_exactly_once"].get<bool>() ||
                                      !a["endpoints_passed"].get<bool>()
                                  ? "未完成"
                              : a["passed"].get<bool>() ? "通过"
                                                        : "证据不足";
                }
                rows.push_back(
                    Json{{"level", level},
                         {"method", method},
                         {"outcome", outcome},
                         {"trajectory_time_s", raw.value("nominal_time_s", Json(nullptr))},
                         {"partial_time_s", nullptr},
                         {"compute_wall_s", raw["planning_wall_s"]},
                         {"qualified_starts", std::to_string(qualified) + "/8"},
                         {"source", file.string()}});
            }
        if (rows.size() != 50 || audit.size() != 160)
            throw std::runtime_error("Expected fifty method rows and 160 independent candidates");
        for (auto &row : rows) {
            const auto raw = read(row["source"].get<std::string>());
            if (raw.contains("durations") && !raw["durations"].empty()) {
                double total = 0.;
                for (double t : raw["durations"])
                    total += t;
                if (std::abs(total - raw["nominal_time_s"].get<double>()) > 1e-8)
                    throw std::runtime_error("Full time mismatch");
            }
            const fs::path folder = fs::path(row["source"].get<std::string>()).parent_path();
            const fs::path file = folder / "assessment.json";
            if (fs::exists(file)) {
                const auto a = read(file);
                double ground = INFINITY;
                for (const auto &g : a["ground"])
                    ground = std::min(ground, g["minimum_clearance_m"].get<double>());
                row["minimum_ground_clearance_m"] = ground;
                row["ground_contact"] = ground < -1e-8;
                for (const auto &c : a["checks"])
                    if (c["outcome"] == "collision")
                        collisions.push_back(Json{{"level", row["level"]},
                                                  {"method", row["method"]},
                                                  {"gate", c["gate"]},
                                                  {"piece", c["piece"]},
                                                  {"witness", c["witness"]}});
            } else {
                row["minimum_ground_clearance_m"] = nullptr;
                row["ground_contact"] = nullptr;
            }
        }
        save(out / "SUMMARY.json", rows);
        save(out / "CANDIDATE_AUDIT.json", Json{{"passed", true}, {"candidates", audit}});
        save(out / "COLLISION_WITNESSES.json", collisions);
        std::ofstream f(out / "PLANNING.md");
        f << "# "
             "五方法十档规划实验，统一0.30米安全球\n\n2026-11-15。六个2."
             "4米实体门（3方3圆）加作者原第7个半径0.3米SingleBall航点，QuadA。六门混合平移/"
             "摆动，全初相0。规划安全球半径0.30米，机体碰撞尺寸读取输入scene.json的collision_radius_m，实体框管半径0."
             "085米只计一次，有限问题另加0."
             "1毫米数值间隙。各方法从同一起点到作者终点，时间包含第7航点后的末段。\n\n本文两方法各8"
             "个独立初值、16路并行，无中心释放。动态TOGT两方法各使用作者独立单初值。Wang为现有独立"
             "复现的N2/"
             "M4局部规划、4个时间初值，整机离面后接续参考，最终航点后执行至固定终点。全部优化为C++"
             "。\n\n## "
             "轨迹时间与结果\n\n单位秒。碰撞单元格保留该完整轨迹时间。未找到完整规划时，前缀时间单"
             "列于原始数据。最短时间仅在通过窗框、任务及端点检查的完整曲线之间加粗。\n\n|幅度比|"
             "频率倍率|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|Wang独立复现|\n|---|---|---:"
             "|---:|---:|---:|---:|\n";
        auto group = [&](const std::string &level) {
            std::map<std::string, Json> g;
            for (const auto &row : rows)
                if (row["level"] == level)
                    g[row["method"].get<std::string>()] = row;
            return g;
        };
        for (const auto &level : levels) {
            const auto g = group(level);
            double best = INFINITY;
            for (const auto &[m, row] : g)
                if (row["outcome"] == "通过")
                    best = std::min(best, row["trajectory_time_s"].get<double>());
            f << '|'
              << (level == "static"                                      ? "0"
                  : level == "mixed_base" || level.substr(0, 3) == "a25" ? "0.25"
                  : level.substr(0, 3) == "a50"                          ? "0.50"
                                                                         : "0.75")
              << '|' << (level == "static" || level == "mixed_base" ? "1" : level.substr(5)) << '|';
            for (const auto &m : methods) {
                const auto &row = g.at(m);
                if (row["trajectory_time_s"].is_number()) {
                    const double t = row["trajectory_time_s"];
                    const bool bold = row["outcome"] == "通过" && std::abs(t - best) < 1e-9;
                    f << (bold ? "**" : "") << std::fixed << std::setprecision(3) << t
                      << (bold ? "**" : "");
                    if (row["outcome"] != "通过")
                        f << "（" << row["outcome"].get<std::string>() << "）";
                } else {
                    f << row["outcome"].get<std::string>();
                    if (row["partial_time_s"].is_number())
                        f << "，前缀" << row["partial_time_s"].get<double>() << "s";
                }
                f << '|';
            }
            f << '\n';
        }
        f << "\n## "
             "求解及独立复查墙钟\n\n单位秒。本文统计8初值并行全部结束的实际等待时间，基线统计进程加"
             "载、求解及复查。公共输入和CasADi图预生成没有计入该列。失败求解的耗时照常列出。\n\n|"
             "条件|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|Wang独立复现|\n|---|---:|---:|--"
             "-:|---:|---:|\n";
        for (const auto &level : levels) {
            const auto g = group(level);
            f << '|' << level << '|';
            for (const auto &m : methods)
                f << std::fixed << std::setprecision(3) << g.at(m)["compute_wall_s"].get<double>()
                  << '|';
            f << '\n';
        }
        f << "\n## "
             "模型与复查口径\n\n规划安全球为0.30米，物理碰撞采用输入场景中的独立机体碰撞尺寸，圆窗512段网格复查。通过为名义曲线的窗框、门序、航点与起终PVAJ检查。"
             "地面相交依既有口径单列在SUMMARY的ground_"
             "contact中。动力学仍沿用各方法软约束。Wang平台映射改为QuadA无阻力平坦性、总推力0."
             "4～27.516N、完整三轴角速度范数上限15rad/s。该范数约束与本文/TOGT的XY范数15、Z2.99rad/"
             "s形式不同。Wang固定局部末端与原第7航点处理属于已记录的场景适配。\n\n完整来源见SUMMARY"
             ".json，碰撞见证见COLLISION_WITNESSES."
             "json，160个本文候选的补点准入与乘子审计见CANDIDATE_AUDIT."
             "json。这里只评估规划，实验未执行Gazebo飞行。\n";
        std::cout << "FINAL fifty methods and 160 independent candidates audited\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
