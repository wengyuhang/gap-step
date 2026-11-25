#include "mesh.hpp"
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
Json read_json(const fs::path& path){Json value;std::ifstream(path)>>value;return value;}
int main(int argc,char** argv) {
    try {
        if(argc!=6)throw std::runtime_error("Usage: restored_release_trial BIN INPUTS SCENE CENTER NEW_OUTPUT");
        const fs::path binary=fs::absolute(argv[1]),input=fs::absolute(argv[2]),scene_file=fs::absolute(argv[3]),
            center_file=fs::absolute(argv[4]),out=fs::absolute(argv[5]);
        if(fs::exists(out))throw std::runtime_error("Use a new output directory");
        fs::create_directories(out/"free");
        const auto center=read_json(center_file);
        if(!center.value("found",false) || center["minco_derivative_order"]!=3)throw std::runtime_error("Expected solved PVAJ center trajectory");
        fs::copy_file(center_file,out/"center_initial.json");fs::copy_file(scene_file,out/"scene.json");
        std::ofstream(out/"PROTOCOL.json")<<Json{{"level","a75_f6"},{"center_time_s",center["nominal_time_s"]},
            {"center_source",center_file.string()},{"center_recomputed",false},{"free_solves",1},{"independent_cold_starts",0},
            {"trajectory","native MincoPV derivative_order3, independent actual PVAJ"},{"time_mapping","TOGT positive durations"},
            {"time_weight",1},{"dynamics_weight",1000},{"stop_at_iteration_limit",false},{"finite_max_iterations",5000},
            {"exchange_rounds",24},{"finite_geometry_minimum_for_verification",-1e-5},
            {"warm_start","preceding raw primal and dual, historical equivalent circle preimage"},
            {"scope","C++ planning and independent numerical geometry assessment"}}.dump(2)<<'\n';
        const auto began=Clock::now();const pid_t child=fork();if(child<0)throw std::runtime_error("fork failed");
        if(child==0) {
            const int log=open((out/"solver.log").c_str(),O_CREAT|O_WRONLY|O_TRUNC,0644);dup2(log,STDOUT_FILENO);dup2(log,STDERR_FILENO);close(log);
            cpu_set_t cpus;CPU_ZERO(&cpus);CPU_SET(0,&cpus);sched_setaffinity(0,sizeof(cpus),&cpus);
            const std::string release="release="+(out/"center_initial.json").string();
            execl((binary/"sip_planner").c_str(),"sip_planner",input.c_str(),(out/"free").c_str(),release.c_str(),
                "--restored-pvaj",(out/"scene.json").c_str(),"1000",nullptr);_exit(127);
        }
        int status=0;if(waitpid(child,&status,0)<0 || !WIFEXITED(status) || WEXITSTATUS(status)!=0)throw std::runtime_error("Free planner process failed");
        const double wall=std::chrono::duration<double>(Clock::now()-began).count();
        const auto raw=read_json(out/"free/result.json");Json assessment=nullptr;
        bool passed=false;
        if(raw["found"].get<bool>()) {
            assessment=mesh_evaluate(Trajectory::load(raw),read_json(out/"scene.json"));passed=assessment["passed"];
            std::ofstream(out/"free/assessment.json")<<assessment.dump(2)<<'\n';
        }
        const auto embedding=read_json(out/"free/center_embedding.json");
        const Json summary{{"found",raw["found"]},{"geometry_passed",passed},{"center_time_s",center["nominal_time_s"]},
            {"free_time_s",passed?raw["nominal_time_s"]:Json(nullptr)},{"new_free_process_wall_s",wall},
            {"termination_reason",raw["termination_reason"]},{"center_embedding",embedding},{"finite_calls",raw["optimizer_calls"].size()},
            {"last_status",raw["optimizer_calls"].back()["status"]},{"raw_result",(out/"free/result.json").string()}};
        std::ofstream(out/"SUMMARY.json")<<summary.dump(2)<<'\n';
        std::ofstream report(out/"REPORT.md");
        report<<"# 0.75／6 固定19.742秒中心解开放自由选点\n\n"
            <<"原生PVAJ MINCO与TOGT正时间，时间/软动力学权重1/1000。直接复用指定中心解，仅运行一次自由选点。\n\n"
            <<"中心完整七窗及返航时间为"<<std::fixed<<std::setprecision(6)<<center["nominal_time_s"].get<double>()<<"秒。\n\n"
            <<"本次结果为"<<(passed?"窗口复查通过":raw["found"].get<bool>()?"独立复查未通过":"未找到合格规划")<<"，自由点优化墙钟"<<wall<<"秒。";
        if(passed)report<<"完整轨迹时间"<<raw["nominal_time_s"].get<double>()<<"秒。";
        report<<"终止原因"<<raw["termination_reason"].get<std::string>()<<"。\n\n"
            <<"取消达到有限迭代上限即结束初值的规则。节点几何最小值低于-1e-5时按原流程退出。单次有限问题最多5000次，最多24轮安全交换，补点使用上一轮变量和乘子。\n\n"
            <<"| 轮次 | 返回状态 | 迭代次数 | 求解／s | 节点几何最小值 |\n|---:|---|---:|---:|---:|\n";
        for(const auto& call:raw["optimizer_calls"])report<<"| "<<call["round"].get<int>()+1<<" | "<<call["status"].get<std::string>()
            <<" | "<<call["iterations"].get<int>()<<" | "<<call["solver_s"].get<double>()<<" | "<<call["minimum_constraint"].get<double>()<<" |\n";
        report<<"\n初始轨迹等价性见free/center_embedding.json。窗口数值复查、独立STL与任务检查分别保留，动力学为软惩罚，Gazebo执行尚未验证。中心解的历史生成成本在原批记录，本次直接复用。\n";
        std::cout<<summary.dump(2)<<'\n';
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
