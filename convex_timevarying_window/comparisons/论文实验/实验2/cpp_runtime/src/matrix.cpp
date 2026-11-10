#include "mesh.hpp"
#include <filesystem>
#include <map>
#include <set>
#include <thread>
#include <sys/file.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>

namespace fs=std::filesystem;
struct Job {std::string level,method;int seed;std::string release;};
struct Running {Job job;std::chrono::steady_clock::time_point began;int cpu;};
const std::vector<std::string> levels{"static","mixed_base","a25_f3","a25_f6","a50_f1","a50_f3","a50_f6","a75_f1","a75_f3","a75_f6"};
const std::vector<std::string> methods{"ours","center","dynamic_togt","dynamic_togt_center","wang2023"};
Json read(const fs::path& path){Json value;std::ifstream(path)>>value;return value;}
void save(const fs::path& path,const Json& value) {
    const fs::path temporary=path.string()+".writing";std::ofstream(temporary)<<value.dump(2)<<'\n';fs::rename(temporary,path);
}
std::string job_name(const Job& job){return job.level+"_"+job.method+"_"+std::to_string(job.seed);}

int main(int argc,char** argv) {
    try {
        if(argc!=6)throw std::runtime_error("Usage: matrix BINARY_DIRECTORY GRAPH_DIRECTORY ASSET_DIRECTORY RESULT_DIRECTORY CONCURRENCY");
        const fs::path binary=fs::absolute(argv[1]),graphs=fs::absolute(argv[2]),assets=fs::absolute(argv[3]),out=fs::absolute(argv[4]);
        const int concurrency=std::stoi(argv[5]);
        fs::create_directories(out);const int lock=open((out/"coordinator.lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC,0644);
        if(flock(lock,LOCK_EX|LOCK_NB))throw std::runtime_error("Coordinator already active");
        fs::create_directories(out/"candidates");fs::create_directories(out/"plans");fs::create_directories(out/"logs");
        Json candidates=fs::exists(out/"candidate_summary.json")?read(out/"candidate_summary.json"):Json::array();
        std::set<std::string> completed;for(const auto& row:candidates)completed.insert(row["job"]);
        std::vector<Job> queue;
        for(const auto& level:levels)for(const auto& method:methods)for(int seed=0;seed<(method=="wang2023"?1:8);++seed) {
            Job job{level,method,seed,""};if(!completed.count(job_name(job)))queue.push_back(job);
        }
        std::map<pid_t,Running> running;std::set<std::string> released;
        for(const auto& row:candidates)if(row["method"]=="release")released.insert(row["level"]);
        const auto began=std::chrono::steady_clock::now();
        auto launch=[&](const Job& job) {
            const auto folder=out/"candidates"/job_name(job);fs::create_directories(folder);
            const std::string graph=(graphs/(job.level+"_"+(job.method=="release"?"ours":job.method))).string();
            std::vector<std::string> args;
            if(job.method=="ours" || job.method=="center" || job.method=="release")
                args={(binary/"sip_planner").string(),graph,folder.string(),job.release.empty()?std::to_string(job.seed):"release="+job.release};
            else if(job.method=="wang2023")args={(binary/"wang_planner").string(),graph,folder.string(),"solve"};
            else args={(binary/"togt_planner").string(),graph,folder.string(),"solve",std::to_string(job.seed)};
            const std::vector<int> physical_cpus{0,2,4,6,8,10,12,14,16,17,18,19,20,21,22,23};
            std::set<int> occupied;
            for(const auto& entry:running)occupied.insert(entry.second.cpu);
            int cpu=-1;
            for(int available:physical_cpus)if(!occupied.count(available)){cpu=available;break;}
            if(cpu<0)throw std::runtime_error("No independent physical CPU available");
            const pid_t pid=fork();
            if(pid<0)throw std::runtime_error("Cannot start candidate");
            if(pid==0) {
                const int logfile=open((out/"logs"/(job_name(job)+".log")).c_str(),O_CREAT|O_WRONLY|O_TRUNC,0644);
                dup2(logfile,STDOUT_FILENO);dup2(logfile,STDERR_FILENO);close(logfile);
                cpu_set_t cpus;CPU_ZERO(&cpus);CPU_SET(cpu,&cpus);sched_setaffinity(0,sizeof(cpus),&cpus);
                std::vector<char*> raw;for(auto& value:args)raw.push_back(value.data());raw.push_back(nullptr);execv(raw[0],raw.data());_exit(127);
            }
            running.emplace(pid,Running{job,std::chrono::steady_clock::now(),cpu});
        };
        auto eligible_best=[&](const std::string& level,const std::string& method)->Json {
            Json best=nullptr;for(const auto& row:candidates)
                if(row["level"]==level && row["method"]==method && row["passed"].get<bool>())
                    if(best.is_null() || (method!="wang2023" && row["objective"].get<double>()<best["objective"].get<double>()))best=row;
            return best;
        };
        auto rows_for=[&](const std::string& level,const std::string& method) {
            Json result=Json::array();for(const auto& row:candidates)
                if(row["level"]==level && (row["method"]==method || (method=="ours" && row["method"]=="release")))result.push_back(row);
            return result;
        };
        auto publish=[&]() {
            Json summary=Json::array();
            for(const auto& level:levels)for(const auto& method:methods) {
                const auto rows=rows_for(level,method);if(rows.size()<static_cast<size_t>(method=="wang2023"?1:8))continue;
                Json best=nullptr;
                for(const auto& row:rows)if(row["passed"].get<bool>())
                    if(best.is_null() || (method!="wang2023" && row["objective"].get<double>()<best["objective"].get<double>()))best=row;
                const auto folder=out/"plans"/(level+"_"+method);fs::create_directories(folder);
                Json raw=best.is_null()?Json{{"found",false}}:read(best["source"].get<std::string>());
                raw["candidate_records"]=rows;raw["method"]=method;raw["case"]={{"layout","togt_whole"},{"level",level}};
                raw["scenario_variant"]="higher_swing_length_2p20";raw["selected_candidate_job"]=best.is_null()?Json(nullptr):best["job"];
                double total=0.;for(const auto& row:rows)total+=row.value("solve_s",0.);
                raw["solve_s"]=total;raw["build_s"]=0.;
                save(folder/"result.json",raw);
                Json geometry=nullptr;
                if(!best.is_null()) {
                    const auto source=fs::path(best["source"].get<std::string>()).parent_path()/"assessment.json";
                    geometry=read(source);save(folder/"assessment.json",geometry);
                }
                std::string outcome="未找到合格规划";
                if(!best.is_null())outcome=geometry["collision_detected"].get<bool>()?"碰撞":geometry["passed"].get<bool>()?"无碰撞完成":"未完成";
                summary.push_back(Json{{"level",level},{"variant",method},{"outcome",outcome},
                    {"full_trajectory_time_s",best.is_null()?Json(nullptr):raw["nominal_time_s"]},
                    {"planning_total_s",total},{"source_file",(folder/"result.json").string()}});
            }
            save(out/"PLANNING.json",summary);
        };
        while(!queue.empty() || !running.empty()) {
            while(!queue.empty() && static_cast<int>(running.size())<concurrency) {
                const auto job=queue.front();queue.erase(queue.begin());launch(job);
            }
            int status=0;const pid_t pid=waitpid(-1,&status,WNOHANG);
            if(pid>0) {
                const auto item=running.at(pid);running.erase(pid);const auto& job=item.job;
                const auto folder=out/"candidates"/job_name(job);const auto source=folder/"result.json";
                const int code=WIFEXITED(status)?WEXITSTATUS(status):128+WTERMSIG(status);
                Json row{{"job",job_name(job)},{"level",job.level},{"method",job.method},{"seed",job.seed},
                    {"exit_code",code},{"source",source.string()},{"found",false},{"passed",false},
                    {"observed_process_wall_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-item.began).count()}};
                if(code==0 && fs::exists(source)) {
                    const auto raw=read(source);row["found"]=raw["found"];row["solve_s"]=raw["solve_s"];
                    row["objective"]=raw.value("objective",0.);row["nominal_time_s"]=raw.value("nominal_time_s",0.);
                    if(raw["found"].get<bool>()) {
                        const auto geometry=mesh_evaluate(Trajectory::load(raw),read(assets/job.level/"scene.json"));
                        save(folder/"assessment.json",geometry);
                        const bool safe_method=job.method=="ours" || job.method=="center" || job.method=="release";
                        bool clear=true;for(const auto& check:geometry["checks"])clear=clear && check["outcome"]=="clear";
                        row["passed"]=!safe_method || (clear && geometry["ordered_exactly_once"].get<bool>() && geometry["endpoints_passed"].get<bool>());
                        row["geometry_verdict"]=geometry["verdict"];
                    }
                }
                candidates.push_back(row);save(out/"candidate_summary.json",candidates);completed.insert(job_name(job));
                std::cout<<"CANDIDATE "<<job_name(job)<<' '<<row["found"]<<' '<<row["passed"]<<std::endl;
                if(code!=0)throw std::runtime_error("Candidate process failed. Preserved output at "+folder.string());
                for(const auto& level:levels)if(!released.count(level)) {
                    int independent=0;for(const auto& row:candidates)
                        if(row["level"]==level && (row["method"]=="ours" || row["method"]=="center"))++independent;
                    if(independent==16) {
                        released.insert(level);const auto best=eligible_best(level,"center");
                        if(!best.is_null())queue.insert(queue.begin(),Job{level,"release",8,best["source"]});
                    }
                }
                publish();
            } else std::this_thread::sleep_for(std::chrono::milliseconds(100));
            save(out/"status.json",Json{{"state","planning"},{"completed",candidates.size()},{"running",running.size()},{"queued",queue.size()},
                {"coordinator_pid",getpid()},{"elapsed_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()}});
        }
        publish();save(out/"status.json",Json{{"state","planning_complete"},{"completed",candidates.size()},{"coordinator_pid",getpid()},
            {"elapsed_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()}});
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
