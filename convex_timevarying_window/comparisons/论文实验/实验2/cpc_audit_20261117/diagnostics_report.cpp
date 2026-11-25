#include "json.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
using J = nlohmann::json;
namespace fs = std::filesystem;
J read(fs::path p) {
  std::ifstream f(p);
  J j;
  f >> j;
  return j;
}
int main(int argc, char **argv) {
  try {
    fs::path root = argv[1], inputs = argv[2];
    std::ofstream out(root / "RESULTS.md");
    out << "# "
           "动态CPC逐项记录\n\n仅规划，原始单次求解；时间包含到任务终点。未完成"
           "不计为有效竞速成绩。轨迹候选满足有限约束与优化器收敛分别列出。\n\n|"
           "条件|判定|候选时间(s)|计算墙钟(s，不含复查)|迭代数|求解器状态|"
           "最大有限约束违反|加密积分位置漂移(m)|\n|---|---|---:|---:|---:|---|"
           "---:|---:|\n";
    J rows = J::array();
    auto protocol = read(inputs / "PROTOCOL.json");
    for (auto lv : protocol["levels"]) {
      std::string name = lv;
      fs::path d = root / "results" / name;
      out << "|" << name << "|";
      if (!fs::exists(d / "result.json") ||
          !fs::exists(d / "assessment.json")) {
        out << "待完成|—|—|—|—|—|—|\n";
        continue;
      }
      auto r = read(d / "result.json"), a = read(d / "assessment.json");
      out << a["verdict"].get<std::string>() << "|" << std::fixed
          << std::setprecision(3) << r["nominal_time_s"].get<double>() << "|"
          << r["total_compute_s"].get<double>() << "|" << r["iterations"] << "|"
          << r["status"].get<std::string>() << "|" << std::scientific
          << r["constraint_violation"].get<double>() << "|";
      if (a.contains("finer_rollout_max_position_drift_m"))
        out << a["finer_rollout_max_position_drift_m"].get<double>();
      else
        out << "—";
      out << "|\n";
      rows.push_back(
          {{"level", name},
           {"verdict", a["verdict"]},
           {"finite_feasible", r["found"]},
           {"optimizer_converged", r["optimizer_converged"]},
           {"status", r["status"]},
           {"iterations", r["iterations"]},
           {"trajectory_s", r["nominal_time_s"]},
           {"compute_s", r["total_compute_s"]},
           {"check_s", a["check_s"]},
           {"result", fs::absolute(d / "result.json").string()},
           {"assessment", fs::absolute(d / "assessment.json").string()}});
    }
    out << "\n原始assessment."
           "json记录每窗真实网格距离、有限开口过平面事件、导航点最近距离、终点"
           "误差及RK4缺陷；≤1 "
           "ms采样无碰撞不是连续安全证书。0.01互补放宽可让进度清零但仍未进入0."
           "3 m导航邻域，导航点任务据实际距离判定。正在求解的候选不填终值。\n";
    std::ofstream(root / "RESULTS.json") << rows.dump(2) << '\n';
    return 0;
  } catch (std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
