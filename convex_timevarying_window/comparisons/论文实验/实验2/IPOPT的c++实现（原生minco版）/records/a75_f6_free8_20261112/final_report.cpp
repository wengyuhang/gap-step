#include "json.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>

using Json = nlohmann::json;
namespace fs = std::filesystem;

Json read_json(const fs::path& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Missing " + path.string());
    Json result;
    input >> result;
    return result;
}

int main(int argc, char** argv) {
    try {
        if (argc != 2) throw std::runtime_error("Usage: final_report BATCH_DIRECTORY");
        const fs::path batch = fs::absolute(argv[1]);
        const fs::path run = batch / "run";
        const auto status = read_json(run / "status.json");
        if (status["state"] != "planning_complete" || status["completed"] != 8)
            throw std::runtime_error("Wait for all eight independent candidates");
        const auto candidates = read_json(run / "candidate_summary.json");
        const auto selected = read_json(run / "plans/a75_f6_ours/result.json");
        std::set<int> seeds;
        Json rows = Json::array();
        int qualified = 0;
        for (int seed = 0; seed < 8; ++seed) {
            const auto folder = run / "candidates" / ("a75_f6_ours_" + std::to_string(seed));
            const auto raw = read_json(folder / "result.json");
            if (raw["minco_derivative_order"] != 3 || raw["baseline_offset_used"].get<bool>() ||
                !raw["restored_historical_soft_workflow"].get<bool>() ||
                raw["dynamics_weight"] != 1000 || raw["time_weight"] != 1)
                throw std::runtime_error("Wrong planning mode");
            Json candidate;
            for (const auto& item : candidates) if (item["seed"] == seed) candidate = item;
            if (candidate.is_null() || candidate["method"] != "ours" || candidate["exit_code"] != 0)
                throw std::runtime_error("Wrong or interrupted candidate");
            seeds.insert(seed);
            const bool found = raw["found"];
            bool passed = false, collision = false, ground = false;
            if (found) {
                const auto geometry = read_json(folder / "assessment.json");
                passed = geometry["passed"];
                collision = geometry["collision_detected"];
                for (const auto& item : geometry["ground"]) ground = ground || item["outcome"] == "collision";
                if (!raw["optimizer_calls"].back()["success"].get<bool>() ||
                    !raw["records"].back()["global_passed"].get<bool>())
                    throw std::runtime_error("Unqualified solver candidate accepted");
            }
            const std::string outcome = !found ? "未找到" : collision ? "碰撞" : passed ? "无碰撞轨迹" : "未完成";
            Json finite = Json::array(), continued_limits = Json::array();
            for (size_t index = 0; index < raw["optimizer_calls"].size(); ++index) {
                const auto& call = raw["optimizer_calls"][index];
                finite.push_back(Json{{"round",index+1},{"status",call["status"]},
                    {"iterations",call["iterations"]},{"solver_s",call["solver_s"]},
                    {"minimum_constraint",call["minimum_constraint"]},{"objective",call["native_objective"]}});
                if (call["status"] == "Maximum_Iterations_Exceeded" && index+1 < raw["optimizer_calls"].size())
                    continued_limits.push_back(index+1);
            }
            qualified += passed;
            rows.push_back(Json{{"seed",seed},{"outcome",outcome},{"found",found},{"passed",passed},
                {"time_s",found ? raw["nominal_time_s"] : Json(nullptr)},
                {"objective",found ? raw["objective"] : Json(nullptr)},
                {"process_wall_s",candidate["observed_process_wall_s"]},
                {"finite_calls",finite},{"continued_iteration_limit_rounds",continued_limits},
                {"termination_reason",raw["termination_reason"]},
                {"ground_collision",found ? Json(ground) : Json(nullptr)},
                {"source",(folder/"result.json").string()}});
        }
        if (seeds.size() != 8 || candidates.size() != 8) throw std::runtime_error("Wrong initial count");
        const double wall = read_json(run / "PAIRED_WALL.json")["free_wall_s"];
        const Json summary{{"level","a75_f6"},{"qualified",qualified},{"starts",8},{"parallel_wall_s",wall},
            {"best_time_s",selected["found"].get<bool>() ? selected["nominal_time_s"] : Json(nullptr)},
            {"best_candidate",selected["selected_candidate_job"]},{"rows",rows},{"gazebo_executed",false}};
        std::ofstream(batch/"SUMMARY.json") << summary.dump(2) << '\n';
        std::ofstream report(batch/"REPORT.md");
        report << "# 0.75／6自由点八独立初值重跑\n\n"
            << "当前原生PVAJ MincoPV3与TOGT正段时间参数。八个冻结独立初值，八物理核单线程并行，"
            << "时间和动力学权重1/1000。补点使用前轮原始变量和乘子。\n\n"
            << "保留取消迭代触顶即停后的流程，以及原节点几何最小值-1e-5门槛。每轮有限问题最多5000次，最多24轮交换。\n\n"
            << "| 初值 | 名义窗口结果 | 完整轨迹／s | 候选计算墙钟／s | 有限问题迭代次数 |\n"
            << "|---:|---|---:|---:|---|\n";
        for (const auto& row : rows) {
            report << "| " << row["seed"].get<int>() << " | " << row["outcome"].get<std::string>() << " | ";
            if (row["time_s"].is_null()) report << "—";
            else report << std::fixed << std::setprecision(3) << row["time_s"].get<double>();
            report << " | " << std::fixed << std::setprecision(2) << row["process_wall_s"].get<double>() << " | ";
            for (size_t k = 0; k < row["finite_calls"].size(); ++k) {
                if (k) report << "、";
                report << row["finite_calls"][k]["iterations"].get<int>();
            }
            report << " |\n";
        }
        report << "\n合格候选" << qualified << "/8，总并行墙钟" << std::fixed << std::setprecision(2) << wall << "秒。";
        if (selected["found"].get<bool>()) report << "最佳候选" << selected["selected_candidate_job"].get<std::string>()
            << "，完整轨迹时间" << std::setprecision(6) << selected["nominal_time_s"].get<double>() << "秒。";
        report << "\n\n完整时间包含七窗后返航。计算墙钟覆盖本轮八候选与独立几何检查，历史初值生成成本单独归档。"
            << "最终接纳要求有限问题成功和原浮点窗口复查，另经独立STL窗框、顺序及端点复查。地面记录单列在SUMMARY.json。"
            << "动力学使用软惩罚，本轮Gazebo执行尚未验证。\n\n"
            << "逐轮状态、触顶后继续轮次与退出原因保存在SUMMARY.json，各候选原始记录在run/candidates。"
            << "核心源码Git提交为f3ca973，本轮输入、二进制与源文件SHA见FROZEN_SHA256.txt。\n";
        std::ofstream(batch/"VALIDATION.json") << Json{{"passed",true},{"independent_free_starts",8},
            {"qualified",qualified},{"correct_runtime_mode",true},{"all_processes_exited_zero",true}}.dump(2) << '\n';
        std::cout << summary.dump(2) << '\n';
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
