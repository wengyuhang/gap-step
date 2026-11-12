#include "geometry.hpp"
#include <filesystem>
#include <iomanip>
#include <map>
#include <numeric>
#include <sstream>

namespace fs=std::filesystem;
Json read_file(const fs::path& path) {
    Json value;std::ifstream file(path);
    if(!file)throw std::runtime_error("Missing file "+path.string());
    file>>value;return value;
}
std::string number(const Json& value,int precision=3) {
    if(value.is_null())return "未找到";
    std::ostringstream stream;stream<<std::fixed<<std::setprecision(precision)<<value.get<double>();return stream.str();
}
int main(int argc,char** argv) {
    try {
        if(argc!=3)throw std::runtime_error("Usage: restored_audit BATCH_DIRECTORY ASSET_DIRECTORY");
        const fs::path batch=argv[1],assets=argv[2],matrix=batch/"matrix";
        const auto state=read_file(matrix/"status.json");
        if(state["state"]!="complete")throw std::runtime_error("Wait for all ten conditions");
        const auto protocol=read_file(matrix/"PROTOCOL.json");
        const auto levels=protocol["levels"].get<std::vector<std::string>>();
        const int derivative_order=protocol["derivative_order"],state_count=21*derivative_order;
        std::map<std::pair<std::string,std::string>,Json> historical;
        for(const auto& row:read_file(batch/"HISTORICAL_PLANNING.json"))
            historical[{row["level"],row["variant"]}]=row["time_s"];
        const auto initialization_file=batch/"inputs/INITIALIZATION.json";
        const double initialization_s=fs::exists(initialization_file)?read_file(initialization_file).value("preparation_s",0.):0.;
        Json rows=Json::array(),audit=Json::array(),finite=Json::array(),recoveries=Json::array();
        int qualified=0,candidates_count=0,failed_finite=0;
        for(const auto& level:levels) {
            const fs::path group=matrix/"groups"/level;
            const auto candidates=read_file(group/"candidate_summary.json");
            const auto wall=read_file(group/"PAIRED_WALL.json");
            const auto scene=read_file(assets/level/"scene.json");
            const auto free_input=read_file(batch/"inputs"/(level+"_ours")/"manifest.json");
            const auto center_input=read_file(batch/"inputs"/(level+"_center")/"manifest.json");
            bool paired=true;
            for(int seed=0;seed<8;++seed) {
                const auto a=free_input["initials"][seed]["original"].get<std::vector<double>>();
                const auto b=center_input["initials"][seed]["original"].get<std::vector<double>>();
                for(int k=0;k<8;++k)paired=paired && a[k]==b[k];
                for(int k=0;k<state_count;++k)paired=paired && a[33+k]==b[8+k];
            }
            if(!paired)throw std::runtime_error("Unpaired initialization at "+level);
            int free_count=0,center_count=0,release_count=0;
            for(const auto& candidate:candidates) {
                const auto method=candidate["method"].get<std::string>();
                free_count+=method=="ours";center_count+=method=="center";release_count+=method=="release";
                const auto raw=read_file(candidate["source"].get<std::string>());
                if(raw["minco_derivative_order"]!=derivative_order ||
                    raw["baseline_offset_used"].get<bool>() || raw["legacy_trajectory_conversion_used"].get<bool>() ||
                    !raw["restored_historical_soft_workflow"].get<bool>() || !raw["workspace_constraints"].get<bool>() ||
                    raw["dynamics_weight"]!=1000. || raw["time_weight"]!=1.)
                    throw std::runtime_error("Wrong runtime mode at "+candidate["job"].get<std::string>());
                if(protocol.value("stop_at_iteration_limit",false)) {
                    const auto& calls=raw["optimizer_calls"];
                    for(size_t index=0;index<calls.size();++index)if(calls[index]["status"]=="Maximum_Iterations_Exceeded") {
                        if(index+1!=calls.size() || raw["found"].get<bool>())throw std::runtime_error("Continued after finite iteration limit");
                        for(const auto& check:raw["records"])if(check["round"]==calls[index]["round"])
                            throw std::runtime_error("Verified or exchanged after iteration limit");
                    }
                }
                for(const auto& call:raw["optimizer_calls"]) {
                    failed_finite+=!call["success"].get<bool>();
                    finite.push_back(Json{{"level",level},{"job",candidate["job"]},{"round",call["round"]},
                        {"status",call["status"]},{"success",call["success"]},{"iterations",call["iterations"]},
                        {"solver_s",call["solver_s"]},{"primal",call.value("final_inf_pr",Json(nullptr))},
                        {"dual",call.value("final_inf_du",Json(nullptr))}});
                }
                if(candidate["passed"].get<bool>()) {
                    Json limits=Json::array();const auto& calls=raw["optimizer_calls"];
                    for(size_t index=0;index+1<calls.size();++index)if(calls[index]["status"]=="Maximum_Iterations_Exceeded")
                        limits.push_back(Json{{"round",index+1},{"iterations",calls[index]["iterations"]},
                            {"added_constraint_rows",calls[index+1]["constraint_rows"].get<int>()-calls[index]["constraint_rows"].get<int>()},
                            {"next_status",calls[index+1]["status"]}});
                    if(!limits.empty())recoveries.push_back(Json{{"level",level},{"job",candidate["job"]},
                        {"method",candidate["method"]},{"seed",candidate["seed"]},{"time_s",raw["nominal_time_s"]},
                        {"limit_rounds",limits},{"final_status",calls.back()["status"]}});
                }
                if(raw["found"].get<bool>() && (!raw["optimizer_calls"].back()["success"].get<bool>() ||
                    !raw["records"].back()["global_passed"].get<bool>()))
                    throw std::runtime_error("Unqualified candidate accepted");
            }
            if(free_count!=8 || center_count!=8 || release_count>1)throw std::runtime_error("Wrong candidate count at "+level);
            candidates_count+=candidates.size();
            for(const auto& method:{"ours","center"}) {
                const fs::path folder=group/"plans"/(level+"_"+method);
                const auto raw=read_file(folder/"result.json");
                const bool found=raw["found"];
                Json time=nullptr;bool ground=false;double ground_min=INFINITY,center_error=0.;
                if(found) {
                    const auto geometry=read_file(folder/"assessment.json");
                    if(!geometry["passed"].get<bool>() || geometry["collision_detected"].get<bool>() ||
                        !geometry["ordered_exactly_once"].get<bool>() || !geometry["endpoints_passed"].get<bool>())
                        throw std::runtime_error("Selected trajectory geometry failed");
                    const auto durations=raw["durations"].get<std::vector<double>>();
                    time=raw["nominal_time_s"];
                    if(durations.size()!=8 || std::abs(std::accumulate(durations.begin(),durations.end(),0.)-time.get<double>())>1e-9)
                        throw std::runtime_error("Incomplete return time");
                    for(const auto& item:geometry["ground"]) {
                        ground=ground || item["outcome"]=="collision";
                        if(item.contains("minimum_clearance_m"))ground_min=std::min(ground_min,item["minimum_clearance_m"].get<double>());
                    }
                    if(std::string(method)=="center")for(int gate=0;gate<7;++gate) {
                        const auto w=load_window(scene["windows"][gate]);const double t=raw["crossing_times_s"][gate];
                        Vector p;for(int axis=0;axis<3;++axis)p[axis]=raw["crossing_points"][gate][axis];
                        center_error=std::max(center_error,(p-w.center-w.rotation*w.motion(t)).norm());
                    }
                    if(center_error>1e-7)throw std::runtime_error("Strict-center equality failed");
                    ++qualified;
                }
                const double compute=wall[std::string(method)=="center"?"center_wall_s":"free_wall_s"];
                if(compute<0.)throw std::runtime_error("Invalid process wall time");
                rows.push_back(Json{{"level",level},{"method",method},{"found",found},{"time_s",time},
                    {"historical_time_s",historical.at({level,method})},{"planning_wall_s",compute},{"shared_initialization_s",initialization_s},{"compute_wall_s",compute+initialization_s},
                    {"ground_collision",found?Json(ground):Json(nullptr)},
                    {"minimum_ground_clearance_m",std::isfinite(ground_min)?Json(ground_min):Json(nullptr)},
                    {"center_max_error_m",std::string(method)=="center" && found?Json(center_error):Json(nullptr)},
                    {"source",(folder/"result.json").string()}});
            }
            audit.push_back(Json{{"level",level},{"paired_initial_T_V",paired},{"free_starts",free_count},
                {"center_starts",center_count},{"release_starts",release_count},{"passed",true}});
        }
        std::ofstream(batch/"FINAL_SUMMARY.json")<<rows.dump(2)<<'\n';
        std::ofstream(batch/"MAX_ITER_RECOVERIES.json")<<recoveries.dump(2)<<'\n';
        std::ofstream(batch/"FINITE_DETAILS.json")<<finite.dump(2)<<'\n';
        std::ofstream(batch/"VALIDATION.json")<<Json{{"passed",true},{"runs",audit},{"selected_qualified",qualified},
            {"selected_total",2*levels.size()},{"stop_at_iteration_limit_verified",protocol.value("stop_at_iteration_limit",false)},{"candidates",candidates_count},{"unconverged_finite_calls",failed_finite}}.dump(2)<<'\n';
        std::ofstream report(batch/"FINAL_REPORT.md");
        report<<"# 历史完整软约束流程的C++ MINCO重跑\n\n"
            <<"完整七窗及返航规划时间，单位秒。截图原始结果来自higher_swing_10x5_full_v1_20261109，"
            <<"本轮采用原生MincoPV库的指定节点导数形式和TOGT正时间映射。\n\n"
            <<"| 条件 | 原截图自由点 | 本轮自由点 | 原截图中心 | 本轮中心 |\n"
            <<"|---|---:|---:|---:|---:|\n";
        for(size_t k=0;k<rows.size();k+=2) {
            const auto& a=rows[k];const auto& b=rows[k+1];
            const bool both=a["found"].get<bool>() && b["found"].get<bool>();
            std::string free=number(a["time_s"]),center=number(b["time_s"]);
            if(both) {if(a["time_s"].get<double>()<b["time_s"].get<double>())free="**"+free+"**";else center="**"+center+"**";}
            report<<"| "<<a["level"].get<std::string>()<<" | "<<number(a["historical_time_s"])<<" | "<<free
                <<" | "<<number(b["historical_time_s"])<<" | "<<center<<" |\n";
        }
        report<<"\n## 计算墙钟与地面记录\n\n| 条件 | 自由点计算／s | 中心计算／s | 自由点地面 | 中心地面 |\n|---|---:|---:|---|---|\n";
        for(size_t k=0;k<rows.size();k+=2) {
            const auto& a=rows[k];const auto& b=rows[k+1];
            auto ground=[](const Json& row){return row["ground_collision"].is_null()?"—":row["ground_collision"].get<bool>()?"相交":"通过";};
            report<<"| "<<a["level"].get<std::string>()<<" | "<<number(a["compute_wall_s"],2)<<" | "<<number(b["compute_wall_s"],2)
                <<" | "<<ground(a)<<" | "<<ground(b)<<" |\n";
        }
        report<<"\n本批公共初值重建耗时"<<std::fixed<<std::setprecision(6)<<initialization_s<<"秒，已加入各方法计算墙钟。并行候选耗时按得到全部候选及最终结果的实际延迟统计。\n";
        report<<"\n## 并行设置\n\n";
        if(protocol.value("total_workers",0)==16 && protocol.value("workers_per_condition",0)==8)
            report<<"两个条件同时启动，每个条件8个单线程求解进程，共16路，使用互不重叠的CPU池。各条件完成8自由点和8中心初值后，再从本条件最佳合格中心开放选点优化一次。\n\n";
        else report<<protocol.value("scheduling_note",std::string("16 workers per condition"))<<"\n\n";
        if(fs::exists(batch/"RECOVERY.json"))report<<"首组一个未完成候选在调度接管的执行会话清理中被中断，保存旧进度后只重试该候选。15个已完成候选保留。自由方法墙钟包含原运行、中断间隔、重试和最终中心释放，中心原墙钟保留。详见RECOVERY.json和interrupted_attempts。\n\n";
        if(protocol.value("stop_at_iteration_limit",false))
            report<<"本轮任一有限问题达到5000次上限即终止当前初值，该轮后续复查与补点跳过。逐候选已检查此规则。\n";
        else report<<"达到5000步后继续补点并最终接纳的候选见MAX_ITER_RECOVERIES.json，其中保留Solve_Succeeded与Solved_To_Acceptable_Level的原始区别。\n";
        for(const auto& row:rows)if(row["method"]=="ours" && !row["found"].get<bool>()) {
            const std::string level=row["level"];
            const auto candidates=read_file(matrix/"groups"/level/"candidate_summary.json");
            int independent_limits=0;
            for(const auto& candidate:candidates) {
                const auto raw=read_file(candidate["source"].get<std::string>());
                if(candidate["method"]=="ours" && !raw["found"].get<bool>() && raw["optimizer_calls"].back()["status"]=="Maximum_Iterations_Exceeded")++independent_limits;
            }
            report<<"\n"<<level<<"自由点未找到合格规划。其中"<<independent_limits<<"个独立自由点初值达到有限问题上限后终止。";
            for(const auto& candidate:candidates)if(candidate["method"]=="release") {
                const auto raw=read_file(candidate["source"].get<std::string>());
                report<<"最佳中心开放选点共求解"<<raw["optimizer_calls"].size()<<"轮有限问题，最后返回"<<raw["optimizer_calls"].back()["status"].get<std::string>()<<"。各轮迭代次数为";
                for(size_t index=0;index<raw["optimizer_calls"].size();++index) {
                    if(index)report<<"、";report<<raw["optimizer_calls"][index]["iterations"].get<int>();
                }
                report<<"。";
            }
            report<<"\n";
        }
        report<<"\n## 实现与核对\n\n"
            <<"本文自由点和中心各8个独立初值，位置从移动中心出发，初始V、A、J由C++位置式MINCO生成。"
            <<"时间采用原八个冻结Sobol样本。独立求解后，从本条件最佳合格中心开放25点参数再优化一次。"
            <<"窗口映射、工作空间和段时长边界、安全补点与圆窗等价热启动恢复历史设置。"
            <<"动力学使用95%x500软界、33节点/段、权重1000，时间权重1。有限问题最多5000步、安全交换最多24轮。\n\n"
            <<"初始化、核构建与导数、优化、复查、并行调度、择优及报告均由C++执行。"
            <<"有限问题返回状态、各轮复查及初始化终止原因均保留。最终接纳要求成功或可接受精度状态和完整复查通过。"
            <<"原始各轮状态和时间保存在FINITE_DETAILS.json。\n\n"
            <<"本轮合格选择"<<qualified<<"/"<<2*levels.size()<<"，总候选"<<candidates_count<<"。"
            <<"选中轨迹核对完整返航、严格中心等式、原浮点Bernstein窗口复查、独立STL框碰撞、顺序及端点。"
            <<"地面记录单列，动力学为软惩罚，Gazebo执行尚未验证。"
            <<"轨迹自由度见PROTOCOL.json，表中结果来自本轮重新求解。\n";
        std::cout<<"VALIDATED "<<qualified<<"/"<<2*levels.size()<<" "<<candidates_count<<" candidates\n";
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
