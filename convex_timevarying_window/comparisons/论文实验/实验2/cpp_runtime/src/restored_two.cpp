#include "json.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <map>
#include <sys/wait.h>
#include <unistd.h>

namespace fs=std::filesystem;
using Json=nlohmann::json;
using Clock=std::chrono::steady_clock;
Json read(const fs::path& path){Json value;std::ifstream(path)>>value;return value;}
void save(const fs::path& path,const Json& value){const fs::path tmp=path.string()+".writing";std::ofstream(tmp)<<value.dump(2)<<'\n';fs::rename(tmp,path);}
int main(int argc,char** argv) {
    try {
        if(argc!=5)throw std::runtime_error("Usage: restored_two BIN INPUTS ASSETS NEW_BATCH");
        const fs::path binary=fs::absolute(argv[1]),inputs=fs::absolute(argv[2]),assets=fs::absolute(argv[3]),batch=fs::absolute(argv[4]),root=batch/"matrix";
        if(fs::exists(root))throw std::runtime_error("Use a new matrix directory");
        fs::create_directories(root/"groups");
        const std::vector<std::string> levels{"a50_f6","a75_f6"};
        const std::vector<std::string> pools{"0,2,4,6,8,10,12,14","16,17,18,19,20,21,22,23"};
        save(root/"PROTOCOL.json",Json{{"levels",levels},{"derivative_order",3},{"trajectory","native MincoPV derivative_order3 with independent actual PVAJ"},
            {"time_mapping","TOGT positive independent segment durations"},{"methods",{"ours","center"}},
            {"independent_starts_per_method",8},{"best_center_release",true},{"finite_max_iterations",5000},{"exchange_rounds",24},
            {"stop_at_iteration_limit",true},{"iteration_limit_action","stop initialization before verification or exchange"},
            {"dynamics_weight",1000},{"time_weight",1},{"workspace_constraints",true},{"duration_range_s",{.3,15.}},
            {"total_workers",16},{"workers_per_condition",8},{"cpu_pools",pools},{"all_planning_code","C++17"},
            {"scheduling_note","Both conditions start together. Eight single-thread solver processes per condition on disjoint CPU pools, sixteen total. Each condition runs8 free and8 center initializations, then one release of its best qualified center."},
            {"gazebo_executed",false}});
        const auto began=Clock::now();std::map<pid_t,std::string> active;
        for(size_t index=0;index<levels.size();++index) {
            const auto folder=root/"groups"/levels[index];
            const pid_t child=fork();if(child<0)throw std::runtime_error("fork failed");
            if(child==0) {
                setenv("WINDOW_MATRIX_CPUS",pools[index].c_str(),1);unsetenv("WINDOW_MATRIX_ELAPSED_OFFSET");
                execl((binary/"matrix").c_str(),"matrix",binary.c_str(),inputs.c_str(),assets.c_str(),folder.c_str(),"8",levels[index].c_str(),"--restored-pvaj",nullptr);
                _exit(127);
            }
            active[child]=levels[index];
        }
        while(!active.empty()) {
            int status=0;const pid_t child=waitpid(-1,&status,WNOHANG);
            if(child>0) {
                if(!WIFEXITED(status) || WEXITSTATUS(status)!=0)throw std::runtime_error("Condition coordinator failed");
                std::cout<<"CONDITION_COMPLETE "<<active.at(child)<<std::endl;active.erase(child);
            }
            Json groups=Json::object();
            for(const auto& level:levels) {
                const auto path=root/"groups"/level/"status.json";
                groups[level]=fs::exists(path)?read(path):Json{{"state","starting"}};
            }
            save(root/"status.json",Json{{"state",active.empty()?"complete":"running"},{"groups",groups},
                {"total_workers",16},{"pid",getpid()},{"elapsed_s",std::chrono::duration<double>(Clock::now()-began).count()}});
            if(!active.empty())std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        execl((binary/"restored_audit").c_str(),"restored_audit",batch.c_str(),assets.c_str(),nullptr);
        throw std::runtime_error("Cannot launch final audit");
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
