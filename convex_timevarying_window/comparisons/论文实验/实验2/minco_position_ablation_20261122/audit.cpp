#include "drolib/solver/minco_snap.hpp"
#include "json.hpp"
#include "minco_pv.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
using Json = nlohmann::json;

Json load(const fs::path& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot read " + path.string());
    return Json::parse(f);
}

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

Eigen::Vector3d point(const Json& p) {
    return {p[0].get<double>(), p[1].get<double>(), p[2].get<double>()};
}

double minco_parity(const drolib::PVAJ& head, const drolib::PVAJ& tail,
                   const Eigen::Matrix3Xd& positions, const Eigen::VectorXd& times,
                   const Json* poly = nullptr) {
    drolib::MincoSnap reference;
    reference.setConditions(head, tail, times.size());
    reference.setParameters(positions, times);
    togt_pv::MincoPV fitted(head, tail, times.size(), 0);
    require(fitted.fit(positions, Eigen::Matrix3Xd(3, 0), times), "MINCO fit failed");
    double error = 0.;
    for (int i = 0; i < times.size(); ++i) {
        for (int k = 0; k < 8; ++k) {
            const Eigen::Vector3d expected =
                reference.getCoeffs().row(8*i+k).transpose() * std::pow(times[i], k);
            const Eigen::Vector3d actual = fitted.normalizedCoefficients().row(8*i+k);
            error = std::max(error, (actual-expected).cwiseAbs().maxCoeff()/(1.+expected.cwiseAbs().maxCoeff()));
            if (poly) {
                const Eigen::Vector3d stored = point((*poly)[i][k]);
                error = std::max(error, (stored-expected).cwiseAbs().maxCoeff()/(1.+expected.cwiseAbs().maxCoeff()));
            }
        }
    }
    return error;
}

std::string seconds(double value) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(3) << value;
    return s.str();
}

std::string time_cell(const Json& method) {
    if (method.at("found").get<bool>()) return seconds(method.at("nominal_time_s"));
    for (const auto& c : method.at("failed_candidates"))
        if (c.at("classification") == "collision") return "碰撞";
    for (const auto& c : method.at("failed_candidates"))
        if (c.at("classification") == "task_incomplete") return "未完成";
    for (const auto& c : method.at("failed_candidates"))
        if (c.at("classification") == "execution_error") return "执行异常";
    return "未找到";
}

int main(int argc, char** argv) try {
    if (argc == 1) {
        drolib::PVAJ head = drolib::PVAJ::Zero(), tail = drolib::PVAJ::Zero();
        head.col(0) << -2., 5., 3.;
        tail.col(0) << 12., -3., 1.;
        Eigen::Matrix3Xd positions(3,4);
        positions << 1.,3.,7.,8., 2.,-1.,4.,-1., 2.,8.,7.,3.;
        Eigen::VectorXd times(5);
        times << 0.2, 1.3, 0.65, 0.41, 0.8;
        const double error = minco_parity(head, tail, positions, times);
        require(error < 1e-9, "Position-only model does not reproduce TOGT MINCO");
        std::cout << "MINCO parity passed, relative coefficient error " << error << '\n';
        return 0;
    }
    require(argc == 4, "Usage: audit WORKDIR BASELINE_DIR OUTPUT_DIR");
    const fs::path work = argv[1], baseline = argv[2], output = argv[3];
    const Json levels = load(baseline/"source_inputs/PROTOCOL.json").at("levels");
    Json rows = Json::array();
    int free_found=0, center_found=0, free_faster=0;
    double maximum_parity_error=0.;
    std::ostringstream planning, compute;
    planning << "# 仅位置 MINCO 消融\n\n只统计通过原有限约束、连续几何复查与独立网格、门序、终点验收的轨迹。"
             << "时间包含末节点飞往原终点。下表无单位标注的时间均为秒。\n\n"
             << "| 条件 | 原 PVAJ 自由点 | 仅位置自由点 | 原 PVAJ 中心 | 仅位置中心 | 仅位置自由/中心合格初值 |\n"
             << "|---|---:|---:|---:|---:|---|\n";
    compute << "# 并行规划计算时间\n\n单位秒。每组计入生成初值和该方法 8 候选全部结束的墙钟时间，包含复查。\n\n"
            << "| 条件 | 原 PVAJ 自由点 | 仅位置自由点 | 原 PVAJ 中心 | 仅位置中心 |\n"
            << "|---|---:|---:|---:|---:|\n";
    for (const std::string level : levels) {
        if (!fs::exists(output/"results"/(level+".json"))) continue;
        const auto summary = load(output/"results"/(level+".json")).at("methods");
        const auto original = load(baseline/"results"/level/"SUMMARY.json").at("methods");
        const auto scene = load(baseline/"source_inputs/assets"/level/"scene.json");
        drolib::PVAJ head = drolib::PVAJ::Zero(), tail = drolib::PVAJ::Zero();
        head.col(0) = point(scene.at("start"));
        tail.col(0) = point(scene.at("goal"));
        Json candidates = Json::array();
        for (const std::string method : {"ours","center"}) {
            int accepted=0;
            double best=INFINITY;
            std::string selected;
            const auto old_init = load(baseline/"results/inputs"/level/method/"manifest.json");
            const auto new_init = load(work/"inputs"/level/method/"manifest.json");
            require(old_init.at("initials") == new_init.at("initials"), "Raw initial trajectories changed: "+level+"/"+method);
            for (int seed=0; seed<8; ++seed) {
                const auto name = method+"_"+std::to_string(seed);
                const auto folder = work/level/name;
                auto c = load(folder/"CANDIDATE.json");
                Json item{{"name",name},{"accepted",c.at("accepted")},{"classification",c.at("classification")},
                    {"finite_solves",c.value("optimizer_calls",Json::array()).size()},
                    {"termination_reason",c.value("termination_reason",Json(nullptr))}};
                if (fs::exists(folder/"result.json")) {
                    const auto r = load(folder/"result.json");
                    const auto independent_dim = new_init.at("velocity_start").get<int>();
                    require(r.at("minco_derivative_order")==0 && r.at("trajectory_variable_count")==independent_dim &&
                            r.at("initial_variables").size()==static_cast<size_t>(independent_dim), "Internal derivative variables retained");
                    item["initial_variables"] = r.at("initial_variables");
                    if (c.at("accepted").get<bool>()) {
                        ++accepted;
                        const double objective=r.at("objective");
                        if (objective<best) {best=objective;selected=name;}
                        for (const auto& call:r.at("optimizer_calls"))
                            require(call.at("status")=="Solve_Succeeded", "Accepted non-converged finite solve");
                        require(c.at("assessment").at("passed").get<bool>(), "Mesh acceptance mismatch");
                        const auto t=r.at("durations").get<std::vector<double>>();
                        const Eigen::Map<const Eigen::VectorXd> times(t.data(),t.size());
                        Eigen::Matrix3Xd positions(3,t.size()-1);
                        for (int k=0;k<positions.cols();++k)
                            positions.col(k)=point(r.at("crossing_points")[k]);
                        const double parity=minco_parity(head,tail,positions,times,&r.at("coefficients"));
                        require(parity<1e-7, "Accepted trajectory differs from position-only MINCO: "+level+"/"+name);
                        maximum_parity_error=std::max(maximum_parity_error,parity);
                        item["reference_coefficient_error"] = parity;
                        item["nominal_time_s"] = r.at("nominal_time_s");
                        item["objective"] = objective;
                    }
                }
                candidates.push_back(item);
            }
            const auto& s=summary[method];
            require(accepted==s.at("successful_candidates").get<int>(), "Candidate count mismatch");
            if (accepted) require(s.at("selected")==selected && s.at("objective")==best, "Selected candidate differs from minimum objective");
        }
        const auto& a=summary["ours"];
        const auto& b=summary["center"];
        const bool pass_a=a.at("found"), pass_b=b.at("found");
        free_found+=pass_a;
        center_found+=pass_b;
        free_faster+=pass_a && pass_b && a.at("nominal_time_s").get<double>()<b.at("nominal_time_s").get<double>();
        planning << '|' << level << '|' << time_cell(original["ours"]) << '|' << time_cell(a)
                 << '|' << time_cell(original["center"]) << '|' << time_cell(b) << '|'
                 << a.at("successful_candidates") << "/8，" << b.at("successful_candidates") << "/8|\n";
        compute << '|' << level;
        for (const std::string method:{"ours","center"})
            compute << '|' << seconds(original[method].at("total_planning_wall_s")) << '|'
                    << seconds(summary[method].at("total_planning_wall_s"));
        compute << "|\n";
        rows.push_back({{"level",level},{"methods",summary},{"candidates",candidates}});
    }
    std::ofstream(output/"PLANNING.md") << planning.str();
    std::ofstream(output/"COMPUTE_TIME.md") << compute.str();
    std::ofstream(output/"AUDIT.json") << Json{{"completed",rows.size()},{"expected",levels.size()},
        {"free_found",free_found},{"center_found",center_found},{"free_faster_when_both_found",free_faster},
        {"maximum_reference_coefficient_error",maximum_parity_error},{"rows",rows}}.dump(2) << '\n';
    std::cout << rows.size() << '/' << levels.size() << " verified; free " << free_found
              << ", center " << center_found << ", coefficient error " << maximum_parity_error << '\n';
    return 0;
} catch(const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
