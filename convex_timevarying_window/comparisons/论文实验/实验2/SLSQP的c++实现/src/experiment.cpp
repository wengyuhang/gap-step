#include "json.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <sys/wait.h>
#include <unistd.h>

using Json=nlohmann::json;
namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
Json read(const fs::path& path){Json x;std::ifstream file(path);if(!file)throw std::runtime_error("Missing "+path.string());file>>x;return x;}
void save(const fs::path& path,const Json& x){const fs::path tmp=path.string()+".writing";std::ofstream(tmp)<<x.dump(2)<<'\n';fs::rename(tmp,path);}
void report(const fs::path& out,const Json& rows) {
    std::ofstream file(out/"REPORT.md");
    file<<"# SLSQP原生MINCO硬动力学实验\n\n"
        <<"每段17个均匀节点，六项95%x500硬动力学不等式。时间权重1，动力学软惩罚0，目标为完整七窗及返航时间。"
        <<"双方各8独立初值，每个方法8路同时执行，方法和条件顺序执行。计算时间为得到该方法全部候选和独立几何检查的实际墙钟。\n\n"
        <<"| 条件 | 方法 | 结果 | 完整参考／s | 8路计算墙钟／s | 合格初值 |\n|---|---|---|---:|---:|---:|\n";
    const std::vector<std::string> order{"static","mixed_base","a25_f3","a25_f6","a50_f1","a50_f3","a50_f6","a75_f1","a75_f3","a75_f6"};
    for(const auto& level:order)for(const auto& row:rows)if(row["level"]==level) {
        file<<"| "<<level<<" | "<<(row["method"]=="ours"?"自由点":"中心")<<" | "<<row["outcome"].get<std::string>()<<" | ";
        if(row["time_s"].is_null())file<<"—";else file<<std::fixed<<std::setprecision(3)<<row["time_s"].get<double>();
        file<<" | "<<std::fixed<<std::setprecision(3)<<row["compute_wall_s"].get<double>()<<" | "<<row["qualified"].get<int>()<<"/8 |\n";
    }
    file<<"\n初值沿用原包的冻结数据，历史生成成本单列。每轮SLSQP最多5000次，最多24轮安全补点，使用上一轮变量热启动。"
        <<"全部目标梯度和约束雅可比由CasADi生成。SLSQP使用原生编译的SciPy1.14.1 Kraft求解核，C++直接调用。"
        <<"动力学验收覆盖指定采样点，窗口使用原浮点包络复查及独立STL/顺序/端点检查，地面单列。\n";
}
int main(int argc,char** argv) {
    try {
        if(argc<4)throw std::runtime_error("Usage: experiment BIN CASES NEW_OUTPUT [LEVEL...]");
        const fs::path bin=fs::absolute(argv[1]),cases=fs::absolute(argv[2]),out=fs::absolute(argv[3]);
        if(fs::exists(out))throw std::runtime_error("Use a new output directory");
        fs::create_directories(out/"groups");
        std::vector<std::string> levels{"a25_f6","a50_f6","a75_f6","static","mixed_base","a25_f3","a50_f1","a50_f3","a75_f1","a75_f3"};
        if(argc>4){levels.clear();for(int i=4;i<argc;++i)levels.push_back(argv[i]);}
        save(out/"PROTOCOL.json",Json{{"levels",levels},{"methods",{"ours","center"}},{"workers",8},{"starts_per_method",8},
            {"center_release",false},{"trajectory","native MincoPV PVAJ3"},{"time_mapping","TOGT independent positive durations"},
            {"dynamic_nodes_per_piece",17},{"dynamic_scalar_constraints",816},{"dynamic_auxiliary_variables",0},
            {"dynamics","original six normalized residuals <=0; 95% x500 bounds"},{"objective","full flight time"},
            {"soft_dynamics_weight",0},{"finite_max_iterations",5000},{"slsqp_ftol",1e-8},{"exchange_rounds",24},
            {"initials","unchanged frozen per-piece times, center points, MINCO VAJ"},{"new_initial_generation",false},
            {"optimizer","SciPy1.14.1 Kraft SLSQP native library, C++ reverse communication"},
            {"derivatives","CasADi objective gradient and constraint Jacobian"},{"workspace_and_duration_domains","inherited unchanged"},
            {"finite_acceptance_tolerance",1e-5},{"scope","planning and nominal geometry assessment"},{"gazebo_executed",false}});
        Json rows=Json::array();const auto began=Clock::now();
        for(const auto& level:levels)for(const auto& method:{"ours","center"}) {
            const fs::path group=out/"groups"/level/method;
            save(out/"status.json",Json{{"state","running"},{"level",level},{"method",method},{"finished_methods",rows.size()},{"pid",getpid()}});
            const auto method_begin=Clock::now();const pid_t child=fork();
            if(child<0)throw std::runtime_error("fork failed");
            if(child==0) {
                setenv("OPENBLAS_NUM_THREADS","1",1);setenv("OMP_NUM_THREADS","1",1);setenv("MKL_NUM_THREADS","1",1);
                execl((bin/"matrix").c_str(),"matrix",bin.c_str(),(cases/"inputs").c_str(),(cases/"assets").c_str(),group.c_str(),"8",level.c_str(),
                    std::string(method)=="ours"?"--slsqp-free-eight":"--slsqp-center-eight",nullptr);_exit(127);
            }
            int status=0;waitpid(child,&status,0);
            if(!WIFEXITED(status) || WEXITSTATUS(status)!=0)throw std::runtime_error("Coordinator failed at "+level+"/"+method);
            const auto raw=read(group/"plans"/(level+"_"+method)/"result.json");const auto candidates=read(group/"candidate_summary.json");
            if(candidates.size()!=8)throw std::runtime_error("Wrong number of independent starts");
            int qualified=0;Json finite=Json::array();
            for(const auto& candidate:candidates) {
                qualified+=candidate["passed"].get<bool>();const auto result=read(candidate["source"].get<std::string>());
                if(result["dynamic_nodes_per_piece"]!=17 || result["dynamics_auxiliary_variables"]!=0 || result["soft_dynamic_penalty"].get<bool>())
                    throw std::runtime_error("Wrong hard dynamics formulation");
                finite.push_back(Json{{"seed",candidate["seed"]},{"found",candidate["found"]},{"passed",candidate["passed"]},
                    {"compute_s",candidate["observed_process_wall_s"]},{"termination_reason",result["termination_reason"]},
                    {"finite_calls",result["optimizer_calls"]},{"time_s",result.value("nominal_time_s",Json(nullptr))}});
            }
            save(group/"FINITE_RESULTS.json",finite);
            bool ground=false;
            if(raw["found"].get<bool>()) {
                const auto assessment=read(group/"plans"/(level+"_"+method)/"assessment.json");
                for(const auto& item:assessment["ground"])ground=ground || item["outcome"]=="collision";
            }
            rows.push_back(Json{{"level",level},{"method",method},{"outcome",raw["found"].get<bool>()?"窗口复查通过":"未找到"},
                {"time_s",raw.value("nominal_time_s",Json(nullptr))},{"compute_wall_s",std::chrono::duration<double>(Clock::now()-method_begin).count()},
                {"qualified",qualified},{"ground_collision",raw["found"].get<bool>()?Json(ground):Json(nullptr)},
                {"best_job",raw["selected_candidate_job"]},{"source",(group/"plans"/(level+"_"+method)/"result.json").string()}});
            save(out/"SUMMARY.json",rows);report(out,rows);
            std::cout<<"METHOD_COMPLETE "<<level<<' '<<method<<' '<<qualified<<std::endl;
        }
        save(out/"status.json",Json{{"state","complete"},{"finished_methods",rows.size()},{"elapsed_s",std::chrono::duration<double>(Clock::now()-began).count()},{"pid",getpid()}});
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
