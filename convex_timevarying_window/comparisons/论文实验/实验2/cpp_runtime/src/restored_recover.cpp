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
Json read(const fs::path& path){Json value;std::ifstream(path)>>value;return value;}
void save(const fs::path& path,const Json& value){const fs::path tmp=path.string()+".writing";std::ofstream(tmp)<<value.dump(2)<<'\n';fs::rename(tmp,path);}
int main(int argc,char** argv) {
    try {
        if(argc!=6)throw std::runtime_error("Usage: restored_recover RECOVERY_BIN INPUTS ASSETS BATCH AUDIT_BIN");
        const fs::path binary=fs::absolute(argv[1]),inputs=fs::absolute(argv[2]),assets=fs::absolute(argv[3]),batch=fs::absolute(argv[4]),audit=fs::absolute(argv[5]);
        const fs::path root=batch/"matrix",first=root/"groups/a50_f6",second=root/"groups/a75_f6";
        const auto old=read(first/"status.json");
        if(old["state"]=="planning_complete" || kill(old["coordinator_pid"].get<int>(),0)==0)
            throw std::runtime_error("Recovery expects a stopped, incomplete first coordinator");
        const auto pending=first/"candidates/a50_f6_ours_7";
        if(fs::exists(pending/"result.json"))throw std::runtime_error("Pending candidate has a result; inspect before retry");
        const fs::path archive=first/"interrupted_attempts/a50_f6_ours_7_attempt1";
        fs::create_directories(archive.parent_path());fs::rename(pending,archive);
        fs::rename(first/"logs/a50_f6_ours_7.log",archive/"solver.log");
        const double gap=std::chrono::duration<double>(fs::file_time_type::clock::now()-fs::last_write_time(first/"status.json")).count();
        const double offset=old["elapsed_s"].get<double>()+gap;
        save(batch/"RECOVERY.json",Json{{"reason","session cleanup terminated waiting coordinator and last unfinished worker during scheduling handoff"},
            {"preserved_finished_candidates",15},{"retry_only","a50_f6_ours_7"},{"archived_attempt",archive.string()},
            {"first_group_elapsed_offset_s",offset},{"completed_candidates_restarted",false},{"original_center_wall_preserved",true},
            {"retry_cpu",0},{"second_group_continues",true}});
        auto protocol=read(root/"PROTOCOL.json");protocol["execution_recovery"]=read(batch/"RECOVERY.json");save(root/"PROTOCOL.json",protocol);
        const pid_t child=fork();if(child<0)throw std::runtime_error("fork failed");
        if(child==0) {
            setenv("WINDOW_MATRIX_CPUS","0",1);const auto text=std::to_string(offset);setenv("WINDOW_MATRIX_ELAPSED_OFFSET",text.c_str(),1);
            execl((binary/"matrix").c_str(),"matrix",binary.c_str(),inputs.c_str(),assets.c_str(),first.c_str(),"1","a50_f6","--restored-pvaj",nullptr);_exit(127);
        }
        bool first_done=false;
        for(;;) {
            int status=0;
            if(!first_done && waitpid(child,&status,WNOHANG)==child) {
                if(!WIFEXITED(status) || WEXITSTATUS(status)!=0)throw std::runtime_error("Retry coordinator failed");
                first_done=true;
            }
            const auto a=read(first/"status.json"),b=read(second/"status.json");const bool second_done=b["state"]=="planning_complete";
            if(!second_done && kill(b["coordinator_pid"].get<int>(),0)!=0)throw std::runtime_error("Second coordinator exited unexpectedly");
            save(root/"status.json",Json{{"state",first_done && second_done?"complete":"parallel_running"},
                {"groups",{{"a50_f6",a},{"a75_f6",b}}},{"methods",first_done && second_done?4:0},{"recovery",true},{"pid",getpid()}});
            if(first_done && second_done)break;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        execl((audit/"restored_audit").c_str(),"restored_audit",batch.c_str(),assets.c_str(),nullptr);
        throw std::runtime_error("Could not launch audit");
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
