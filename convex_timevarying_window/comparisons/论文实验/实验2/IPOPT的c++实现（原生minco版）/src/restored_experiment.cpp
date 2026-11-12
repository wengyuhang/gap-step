#include "json.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <sys/wait.h>
#include <unistd.h>

namespace fs=std::filesystem;
using Json=nlohmann::json;
using Clock=std::chrono::steady_clock;

Json read_json(const fs::path& path) {
    Json value;std::ifstream input(path);
    if(!input)throw std::runtime_error("Cannot open "+path.string());
    input>>value;return value;
}
void save_json(const fs::path& path,const Json& value) {
    const fs::path temporary=path.string()+".writing";
    std::ofstream(temporary)<<value.dump(2)<<'\n';fs::rename(temporary,path);
}
void report(const fs::path& output,const Json& rows) {
    std::ofstream markdown(output/"REPORT.md");
    markdown<<"# 历史软约束流程的C++ MINCO重跑\n\n"
        <<"完整七窗与返航规划时间。八个独立自由点、八个独立中心初值，再从最佳合格中心开放选点求解一次。"
        <<"使用C++原生MINCO、TOGT正时间映射、原内缩映射、1000动力学软权重、原工作空间及段时长约束。\n\n"
        <<"| 条件 | 方法 | 窗口轨迹结果 | 完整轨迹／s | 计算墙钟／s | 地面记录 |\n"
        <<"|---|---|---|---:|---:|---|\n";
    for(const auto& row:rows) {
        markdown<<"| "<<row["level"].get<std::string>()<<" | "
            <<(row["method"]=="ours"?"自由点":"中心")<<" | "<<row["outcome"].get<std::string>()<<" | ";
        if(row["time_s"].is_null())markdown<<"—";
        else markdown<<std::fixed<<std::setprecision(3)<<row["time_s"].get<double>();
        markdown<<" | "<<std::fixed<<std::setprecision(2)<<row["compute_wall_s"].get<double>()<<" | "
            <<row["ground"].get<std::string>()<<" |\n";
    }
    markdown<<"\n原始截图与本轮各自独立保存，表中展示新求解的实际结果。"
        <<"初始时间为历史Sobol样本数据，中心点与速度均由C++重新构造。"
        <<"成功结果通过原浮点Bernstein局部复查及独立STL网格、穿窗顺序和端点复查。"
        <<"地面接触单列。Gazebo闭环执行尚未验证。\n";
}
int main(int argc,char** argv) {
    try {
        if(argc<6)throw std::runtime_error("Usage: restored_experiment BIN INPUTS ASSETS OUTPUT WORKERS [DERIVATIVE_ORDER [LEVEL...]]");
        const fs::path binary=fs::absolute(argv[1]),inputs=fs::absolute(argv[2]),assets=fs::absolute(argv[3]),output=fs::absolute(argv[4]);
        if(fs::exists(output))throw std::runtime_error("Use a new result directory");
        fs::create_directories(output/"groups");
        std::vector<std::string> levels{"static","mixed_base","a25_f3","a25_f6","a50_f1","a50_f3","a50_f6","a75_f1","a75_f3","a75_f6"};
        const int derivative_order=argc>=7?std::stoi(argv[6]):1;
        if(derivative_order!=1 && derivative_order!=3)throw std::runtime_error("Expected derivative order1 or3");
        const std::string mode=derivative_order==3?"--restored-pvaj":"--restored-pv";
        if(argc>7) {
            const auto supported=levels;levels.clear();
            for(int k=7;k<argc;++k) {
                if(std::find(supported.begin(),supported.end(),argv[k])==supported.end())throw std::runtime_error("Unsupported condition");
                levels.push_back(argv[k]);
            }
        }
        const auto began=Clock::now();Json rows=Json::array();
        save_json(output/"PROTOCOL.json",Json{{"trajectory",derivative_order==3?"MincoPV derivative_order3 prescribed PVAJ":"MincoPV derivative_order1 prescribed PV"},{"derivative_order",derivative_order},{"levels",levels},{"time_mapping","TOGT positive independent durations"},
            {"methods",{"ours","center"}},{"independent_starts_per_method",8},{"best_center_release",true},
            {"finite_max_iterations",5000},{"exchange_rounds",24},{"dynamics_weight",1000},{"time_weight",1},
            {"workspace_constraints",true},{"duration_range_s",{.3,15.}},{"workers",std::stoi(argv[5])},
            {"timing_archive","original eight independent Sobol duration samples"},{"all_planning_code","C++17"},
            {"geometry_evidence","floating local enclosures and independent mesh checks"},{"gazebo_executed",false}});
        for(const auto& level:levels) {
            const fs::path group=output/"groups"/level;
            save_json(output/"status.json",Json{{"state","running"},{"level",level},{"finished_methods",rows.size()},{"pid",getpid()}});
            const pid_t child=fork();
            if(child<0)throw std::runtime_error("fork failed");
            if(child==0) {
                execl((binary/"matrix").c_str(),"matrix",binary.c_str(),inputs.c_str(),assets.c_str(),group.c_str(),argv[5],level.c_str(),mode.c_str(),nullptr);
                _exit(127);
            }
            int status=0;if(waitpid(child,&status,0)<0 || !WIFEXITED(status) || WEXITSTATUS(status)!=0)
                throw std::runtime_error("Condition failed, preserved "+group.string());
            for(const auto& method:{"ours","center"}) {
                const fs::path folder=group/"plans"/(level+"_"+method);
                const auto raw=read_json(folder/"result.json");
                bool found=raw["found"],passed=false,collision=false,ground=false;
                if(found) {
                    const auto assessment=read_json(folder/"assessment.json");passed=assessment["passed"];collision=assessment["collision_detected"];
                    for(const auto& item:assessment["ground"])ground=ground || item["outcome"]=="collision";
                }
                rows.push_back(Json{{"level",level},{"method",method},{"outcome",!found?"未找到合格规划":collision?"碰撞":passed?"窗口复查通过":"未完成"},
                    {"time_s",found?raw["nominal_time_s"]:Json(nullptr)},{"compute_wall_s",raw["planning_wall_s"]},
                    {"ground",!found?"—":ground?"相交":"通过"},{"source",(folder/"result.json").string()}});
            }
            save_json(output/"SUMMARY.json",rows);report(output,rows);
            std::cout<<"CONDITION_COMPLETE "<<level<<std::endl;
        }
        save_json(output/"status.json",Json{{"state","complete"},{"methods",rows.size()},{"elapsed_s",std::chrono::duration<double>(Clock::now()-began).count()}});
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
