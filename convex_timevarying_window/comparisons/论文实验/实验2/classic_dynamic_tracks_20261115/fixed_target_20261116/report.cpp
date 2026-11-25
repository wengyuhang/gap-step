#include "json.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
namespace fs = std::filesystem;
using Json = nlohmann::json;
Json load(const fs::path &path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("Missing " + path.string());
    Json value;
    stream >> value;
    return value;
}
std::string number(double value, int digits = 3) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(digits) << value;
    return stream.str();
}
int main(int argc, char **argv) {
    try {
        if (argc != 2) throw std::runtime_error("Usage: REPORT RESULT_DIRECTORY");
        const fs::path directory = argv[1];
        const auto summary = load(directory / "SUMMARY.json");
        const auto status = load(directory / "STATUS.json");
        std::ofstream report(directory / "REPORT.md");
        report << "# 固定赛道统一启动实验\n\n";
        report << "六条件采用同一冻结协议。原目标运动在初始化、有限优化、补点和独立复查中保持同一函数。所有输出是名义规划结果。\n\n";
        report << "|赛道|自由点全程s|中心全程s|自由点节时|自由点成功初值|中心成功初值|自由点计算s|中心计算s|\n|---|---:|---:|---:|---:|---:|---:|---:|\n";
        Json statistics = Json::array();
        for (const auto &condition : summary.at("conditions")) {
            const auto &free = condition.at("methods").at("ours");
            const auto &center = condition.at("methods").at("center");
            const bool freeFound = free.at("found"), centerFound = center.at("found");
            double advantage = 0.;
            std::string freeTime = "未找到", centerTime = "未找到", advantageText = "—";
            if (freeFound) freeTime = number(free.at("nominal_time_s"));
            if (centerFound) centerTime = number(center.at("nominal_time_s"));
            if (freeFound && centerFound) {
                const double tf = free.at("nominal_time_s"), tc = center.at("nominal_time_s");
                advantage = 100. * (tc - tf) / tc;
                advantageText = number(advantage, 2) + "%";
                if (tf < tc) freeTime = "**" + freeTime + "**";
                if (tc < tf) centerTime = "**" + centerTime + "**";
            }
            report << condition.at("level").get<std::string>() << '|' << freeTime << '|' << centerTime << '|' << advantageText << '|'
                << free.at("successful_candidates") << "/8|" << center.at("successful_candidates") << "/8|"
                << number(free.at("elapsed_to_final_selection_s")) << '|' << number(center.at("elapsed_to_final_selection_s")) << "|\n";
            Json counts = Json::object();
            for (const auto &method : {"ours", "center"}) {
                std::map<std::string, int> classes, solverStatuses;
                for (const auto &candidate : condition.at("methods").at(method).at("candidates")) {
                    ++classes[candidate.at("classification").get<std::string>()];
                    for (const auto &call : candidate.value("optimizer_calls", Json::array()))
                        ++solverStatuses[call.at("status").get<std::string>()];
                }
                const int missing = 8 - condition.at("methods").at(method).at("candidates").size();
                if (missing) classes["initialization_or_preparation_failed"] = missing;
                counts[method] = {{"candidate_classifications", classes}, {"finite_solver_statuses", solverStatuses}};
            }
            statistics.push_back({{"level", condition.at("level")}, {"scene_bytes_unchanged", condition.at("scene_bytes_unchanged")},
                {"counts", counts}, {"free_time_reduction_percent", freeFound && centerFound ? Json(advantage) : Json(nullptr)}});
        }
        report << "\n全程时间覆盖原起点、按序穿完所有窗口及原指定终点。三个布局是Split-S 7窗、AlphaPilot 11窗、AirSim 21窗。\n\n";
        report << "计算时间含八路当前目标MINCO时间初始化的公共墙钟、两方法全部PVAJ准备和各自八候选与独立复查的实际并行等待。全部候选结束后按原总目标在合格解中选择。候选CPU时间未累加。\n\n";
        report << "一次完整候选须有限问题正常结束且当前有限约束合格，原SIP复查通过，独立实体窗框碰撞/顺序/终点检查通过。未找到记录不记为无碰撞通过。优化器状态和失败原因在逐候选明细。\n\n";
        report << "原生PVAJ MINCO、TOGT正时间映射、原动力学软惩罚1/1000、33节点/段积分、规划安全球0.30m、有限球与内缩0.3001m、原0.1m/s法向速度条件沿用本批协议。几何仍是浮点连续域数值证据，动力学仍是软约束。\n\n";
        report << "本批初始化采用八个原逐段时间模式，各自在当前目标运动下先做MINCO中心时间优化，再恢复PVAJ。自由/中心对应种子物理轨迹相同。每次正式求解只读原场景，不载入静态、低幅度或其他赛道的已求结果。八候选均保留。\n\n";
        report << "固定的窗口运动参数见上级motion_audit/MOTION_TABLE.md和MOTION_AUDIT.json。每个移动窗的完整参数组合各不相同，静止门按原场景列出。\n\n";
        report << "## 逐候选结果\n\n|条件|候选|分类|全程s|有限求解状态及迭代次数|\n|---|---|---|---:|---|\n";
        for (const auto &condition : summary.at("conditions"))
            for (const auto &method : {"ours", "center"})
                for (const auto &candidate : condition.at("methods").at(method).at("candidates")) {
                    report << condition.at("level").get<std::string>() << '|' << candidate.at("name").get<std::string>() << '|'
                        << candidate.at("classification").get<std::string>() << '|';
                    if (candidate.contains("nominal_time_s")) report << number(candidate.at("nominal_time_s"));
                    else report << "—";
                    report << '|';
                    for (const auto &call : candidate.value("optimizer_calls", Json::array()))
                        report << call.at("status").get<std::string>() << '(' << call.at("iterations") << ") ";
                    report << "|\n";
                }
        report << "\n## 初始化失败\n\n|条件|时间种子|作者初始化状态|说明|\n|---|---:|---|---|\n";
        for (const auto &condition : summary.at("conditions")) {
            const std::string level = condition.at("level");
            for (int index = 0; index < 8; ++index) {
                const auto seed = load(directory / level / "seeds" / std::to_string(index) / "result.json");
                if (!seed.value("found", false))
                    report << level << '|' << index << '|' << seed.at("status") << "|双方对应种子在初始化阶段停止，纳入8个尝试的分母|\n";
            }
        }
        report << "\n批次状态为" << status.at("state").get<std::string>() << "，已完成" << summary.at("conditions").size() << "个条件。\n";
        std::ofstream(directory / "STATISTICS.json") << statistics.dump(2) << '\n';
    } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
    return 0;
}
