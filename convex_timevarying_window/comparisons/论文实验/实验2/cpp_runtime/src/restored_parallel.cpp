#include "json.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>

namespace fs=std::filesystem;
using Json=nlohmann::json;
Json read_json(const fs::path& path) {
    Json value;std::ifstream file(path);if(!file)throw std::runtime_error("Missing "+path.string());file>>value;return value;
}
void save_json(const fs::path& path,const Json& value) {
    const fs::path temporary=path.string()+".writing";std::ofstream(temporary)<<value.dump(2)<<'\n';fs::rename(temporary,path);
}
int main(int argc,char** argv) {
    try {
        if(argc!=7)throw std::runtime_error("Usage: restored_parallel ORIGINAL_BIN SECOND_BIN INPUTS ASSETS BATCH SECOND_WORKERS");
        const fs::path original=fs::absolute(argv[1]),binary=fs::absolute(argv[2]),inputs=fs::absolute(argv[3]),
            assets=fs::absolute(argv[4]),batch=fs::absolute(argv[5]),root=batch/"matrix";
        const fs::path first=root/"groups/a50_f6",second=root/"groups/a75_f6";
        if(fs::exists(second))throw std::runtime_error("Second condition already exists");
        auto protocol=read_json(root/"PROTOCOL.json");
        protocol["scheduling_note"]="a50_f6 started with16 workers; a75_f6 starts on13 disjoint idle CPUs while first condition finishes; numerical settings unchanged";
        protocol["second_group_workers"]=std::stoi(argv[6]);
        protocol["second_group_cpu_pool"]=std::getenv("WINDOW_MATRIX_CPUS");
        save_json(root/"PROTOCOL.json",protocol);
        const auto began=std::chrono::steady_clock::now();
        const pid_t child=fork();if(child<0)throw std::runtime_error("fork failed");
        if(child==0) {
            execl((binary/"matrix").c_str(),"matrix",binary.c_str(),inputs.c_str(),assets.c_str(),second.c_str(),argv[6],"a75_f6","--restored-pvaj",nullptr);
            _exit(127);
        }
        bool second_done=false;
        for(;;) {
            int status=0;
            if(!second_done && waitpid(child,&status,WNOHANG)==child) {
                if(!WIFEXITED(status) || WEXITSTATUS(status)!=0)throw std::runtime_error("Second coordinator failed");
                second_done=true;
            }
            const auto a=read_json(first/"status.json");
            const bool first_done=a["state"]=="planning_complete";
            if(!first_done && kill(a["coordinator_pid"].get<int>(),0)!=0)throw std::runtime_error("First coordinator exited unexpectedly");
            Json b=fs::exists(second/"status.json")?read_json(second/"status.json"):Json{{"state","starting"}};
            save_json(root/"status.json",Json{{"state",first_done && second_done?"complete":"parallel_running"},
                {"groups",{{"a50_f6",a},{"a75_f6",b}}},{"methods",first_done && second_done?4:0},{"pid",getpid()},
                {"parallel_stage_elapsed_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()}});
            if(first_done && second_done)break;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        execl((original/"restored_audit").c_str(),"restored_audit",batch.c_str(),assets.c_str(),nullptr);
        throw std::runtime_error("Cannot launch final audit");
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
