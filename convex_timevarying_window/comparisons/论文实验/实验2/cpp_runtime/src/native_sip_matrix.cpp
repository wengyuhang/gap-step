#include "json.hpp"
#include <chrono>
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <iomanip>
#include <limits>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/wait.h>
#include <unistd.h>

namespace fs = std::filesystem;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

Json load(const fs::path &file) {
    std::ifstream stream(file);
    if (!stream) throw std::runtime_error("Missing " + file.string());
    Json value;
    stream >> value;
    return value;
}

void save(const fs::path &file, const Json &value) {
    const fs::path temporary = file.string() + ".writing";
    std::ofstream stream(temporary);
    if (!stream) throw std::runtime_error("Cannot write " + temporary.string());
    stream << value.dump(2) << '\n';
    stream.close();
    fs::rename(temporary, file);
}

std::string contents(const fs::path &file) {
    std::ifstream stream(file);
    if (!stream) throw std::runtime_error("Missing " + file.string());
    return std::string(std::istreambuf_iterator<char>(stream), {});
}

struct Job {
    std::string name;
    std::vector<std::string> arguments;
    fs::path log;
    Clock::time_point started;
};

pid_t launch(const Job &job) {
    const pid_t pid = fork();
    if (pid < 0) throw std::runtime_error("fork failed");
    if (pid == 0) {
        const int fd = open(job.log.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
        if (fd < 0) _exit(125);
        dup2(fd, STDOUT_FILENO);
        dup2(fd, STDERR_FILENO);
        close(fd);
        setenv("OPENBLAS_NUM_THREADS", "1", 1);
        setenv("OMP_NUM_THREADS", "1", 1);
        setenv("MKL_NUM_THREADS", "1", 1);
        setenv("SIP_DUMP_INPUT", "1", 1);
        auto arguments = job.arguments;
        std::vector<char *> raw;
        for (auto &value : arguments) raw.push_back(value.data());
        raw.push_back(nullptr);
        execv(raw.front(), raw.data());
        _exit(126);
    }
    return pid;
}

int exitCode(int status) {
    return WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
}

int execute(const std::vector<std::string> &arguments, const fs::path &log) {
    Job job{"", arguments, log, Clock::now()};
    const pid_t pid = launch(job);
    int status;
    while (waitpid(pid, &status, 0) < 0) {
        if (errno != EINTR) throw std::runtime_error("waitpid failed");
    }
    return exitCode(status);
}

Json parallel(std::vector<Job> jobs, const fs::path &statusFile) {
    const auto batch_began=Clock::now();
    std::map<pid_t, Job> active;
    Json completed = Json::array();
    for (auto &job : jobs) {
        job.started = Clock::now();
        active.emplace(launch(job), job);
    }
    auto update = [&] {
        Json running = Json::array();
        for (const auto &entry : active)
            running.push_back({{"pid", entry.first}, {"job", entry.second.name}});
        save(statusFile, {{"active", running}, {"completed", completed}});
    };
    update();
    while (!active.empty()) {
        int status;
        pid_t pid;
        do { pid = waitpid(-1, &status, 0); } while (pid < 0 && errno == EINTR);
        if (pid < 0) throw std::runtime_error("waitpid failed");
        const auto found = active.find(pid);
        if (found == active.end()) throw std::runtime_error("Unknown child");
        const double elapsed = std::chrono::duration<double>(Clock::now() - found->second.started).count();
        completed.push_back({{"job", found->second.name}, {"exit_code", exitCode(status)}, {"wall_s", elapsed},{"finished_after_s",std::chrono::duration<double>(Clock::now()-batch_began).count()}});
        std::cout << found->second.name << " finished " << exitCode(status) << '\n' << std::flush;
        active.erase(found);
        update();
    }
    return completed;
}

int candidate(int argc, char **argv) {
    if (argc != 7 && argc != 8) throw std::runtime_error("candidate SIP MESH INPUT SCENE OUTPUT [SEED]");
    const fs::path output = argv[6];
    fs::create_directories(output);
    const auto began = Clock::now();
    const int code = execute({argv[2], argv[4], output.string(), argc == 8 ? argv[7] : "0", "--soft-mincopv", argv[5], "1000"}, output / "sip.log");
    Json result{{"sip_exit_code", code}, {"accepted", false}, {"classification", "execution_error"}};
    if (fs::exists(output / "result.json")) {
        const auto raw = load(output / "result.json");
        result["sip_found"] = raw.value("found", false);
        result["finite_solver"] = raw.value("finite_solver", std::string("unknown"));
        result["termination_reason"] = raw.value("termination_reason", std::string());
        result["optimizer_calls"] = Json::array();
        for (const auto &call : raw.at("optimizer_calls"))
            result["optimizer_calls"].push_back({{"round", call.at("round")}, {"status", call.at("status")},
                {"iterations", call.at("iterations")}, {"solver_s", call.at("solver_s")},
                {"finite_violation", call.at("constraint_violation_inf")}});
        if (raw.value("found", false) && code == 0) {
            result["nominal_time_s"] = raw.at("nominal_time_s");
            result["objective"] = raw.at("objective");
            const int checked = execute({argv[3], (output / "result.json").string(), argv[5], (output / "mesh_assessment.json").string()}, output / "mesh.log");
            result["mesh_exit_code"] = checked;
            if (checked == 0) {
                const auto assessment = load(output / "mesh_assessment.json");
                result["assessment"] = assessment;
                const bool accepted = assessment.value("passed", false) &&
                    !assessment.value("collision_detected", true) &&
                    assessment.value("ordered_exactly_once", false) && assessment.value("endpoints_passed", false);
                result["accepted"] = accepted;
                result["classification"] = accepted ? "nominal_safe_complete" :
                    assessment.value("collision_detected", false) ? "collision" : "task_incomplete";
            }
        } else result["classification"] = "not_found";
    }
    result["wall_s"] = std::chrono::duration<double>(Clock::now() - began).count();
    save(output / "CANDIDATE.json", result);
    return code;
}



std::string time_cell(const Json& method) {
    if (!method.value("found", false)) {
        for(const auto& failure:method["failed_candidates"])if(failure["classification"]=="collision")return "碰撞";
        for(const auto& failure:method["failed_candidates"])if(failure["classification"]=="task_incomplete")return "未完成";
        for(const auto& failure:method["failed_candidates"])if(failure["classification"]=="execution_error")return "执行异常";
        return "未找到";
    }
    std::ostringstream out; out << std::fixed << std::setprecision(3) << method.at("nominal_time_s").get<double>();
    return out.str();
}

int run_case(int argc, char** argv) {
    if (argc != 6) throw std::runtime_error("INPUTS SCENE NEW_OUTPUT SIP MESH");
    const fs::path inputs=fs::absolute(argv[1]), scene=fs::absolute(argv[2]), out=fs::absolute(argv[3]);
    fs::create_directories(out.parent_path());
    if (!fs::create_directory(out)) throw std::runtime_error("Output already exists");
    const auto free_input=load(inputs/"ours"/"manifest.json"),center_input=load(inputs/"center"/"manifest.json");
    if(free_input["initials"].size()!=8 || center_input["initials"].size()!=8 ||
        free_input["windows"]!=center_input["windows"] || free_input["vehicle"]!=center_input["vehicle"] ||
        !free_input["free_points"].get<bool>() || center_input["free_points"].get<bool>())
        throw std::runtime_error("Paired input protocol mismatch");
    const int pieces=free_input["n"],free_start=free_input["velocity_start"],center_start=center_input["velocity_start"];
    for(int seed=0;seed<8;++seed) {
        const auto a=free_input["initials"][seed]["original"].get<std::vector<double>>();
        const auto b=center_input["initials"][seed]["original"].get<std::vector<double>>();
        if(!std::equal(a.begin(),a.begin()+pieces,b.begin()) ||
            !std::equal(a.begin()+free_start,a.end(),b.begin()+center_start,b.end()) ||
            free_input["initials"][seed]["parameters"]!=center_input["initials"][seed]["parameters"] ||
            !std::all_of(a.begin()+pieces,a.begin()+free_start,[](double value){return value==0.;}))
            throw std::runtime_error("Paired raw physical initial trajectories differ");
    }
    std::vector<Job> jobs;
    for (std::string method : {"ours", "center"}) for (int seed=0;seed<8;++seed) {
        const std::string name=method+"_"+std::to_string(seed);
        jobs.push_back({name,{fs::absolute(argv[0]).string(),"--candidate",fs::absolute(argv[4]).string(),
            fs::absolute(argv[5]).string(),(inputs/method).string(),scene.string(),(out/name).string(),std::to_string(seed)},out/(name+".log"),{}});
    }
    const double initialization=load(inputs/"INITIALIZATION_AUDIT.json").at("preparation_wall_s");
    const auto began=Clock::now();
    save(out/"STATUS.json",{{"state","running"},{"pid",getpid()}});
    const auto records=parallel(jobs,out/"CANDIDATE_STATUS.json");
    Json methods=Json::object();
    for (std::string method : {"ours","center"}) {
        Json selected={{"found",false},{"successful_candidates",0},{"failed_candidates",Json::array()}};
        double best=INFINITY, elapsed=0.;
        for (const auto& record:records)
            if(record["job"].get<std::string>().rfind(method+"_",0)==0)
                elapsed=std::max(elapsed,record["finished_after_s"].get<double>());
        for (int seed=0;seed<8;++seed) {
            const auto name=method+"_"+std::to_string(seed);
            const auto result=load(out/name/"CANDIDATE.json");
            if(!result.value("accepted",false)) {
                selected["failed_candidates"].push_back({{"seed",seed},{"classification",result.value("classification",std::string("execution_error"))},
                    {"termination_reason",result.value("termination_reason",std::string())}});
                continue;
            }
            selected["successful_candidates"]=selected["successful_candidates"].get<int>()+1;
            const double value=result["objective"];
            if(value<best) {
                best=value; selected["found"]=true; selected["nominal_time_s"]=result["nominal_time_s"];
                selected["objective"]=value; selected["selected"]=name;
            }
        }
        selected["initialization_wall_s"]=initialization;
        selected["parallel_candidates_wall_s"]=elapsed;
        selected["total_planning_wall_s"]=initialization+elapsed;
        const auto& method_input=method=="ours"?free_input:center_input;
        selected["finite_solver"]=method_input.value("native_finite_solver",std::string("ipopt"));
        selected["native_soft_formulation"]=method_input.value("native_soft_formulation",std::string("residual_epigraph"));
        methods[method]=selected;
    }
    save(out/"SUMMARY.json",{{"methods",methods},{"jobs",records},{"wall_s",std::chrono::duration<double>(Clock::now()-began).count()},
        {"planning_only",true},{"shared_raw_initials_checked",true},{"independent_initializations",true},{"quadrature_intervals",16}});
    save(out/"STATUS.json",{{"state","complete"},{"pid",getpid()}});
    return 0;
}

int campaign(int argc,char** argv,bool robust=false) {
    if(argc<8)throw std::runtime_error("--campaign INPUT_ROOT NEW_OUTPUT SIP MESH PREPARE LEVEL...");
    const fs::path source=fs::absolute(argv[2]),out=fs::absolute(argv[3]);
    fs::create_directories(out.parent_path());
    if(!fs::create_directory(out))throw std::runtime_error("Campaign output already exists");
    Json completed=Json::array();
    for(int k=7;k<argc;++k) {
        const std::string level=argv[k]; const auto inputs=out/"inputs"/level;
        save(out/"STATUS.json",{{"state","running"},{"level",level},{"completed",completed},{"pid",getpid()}});
        std::vector<std::string> prepare{fs::absolute(argv[6]).string(),source.string(),level,inputs.string()};
        if(robust)prepare.push_back("--robust-ipopt-profile");
        if(execute(prepare,out/(level+".prepare.log"))!=0)
            throw std::runtime_error("Initialization failed: "+level);
        if(execute({fs::absolute(argv[0]).string(),inputs.string(),(source/"assets"/level/"scene.json").string(),
            (out/level).string(),fs::absolute(argv[4]).string(),fs::absolute(argv[5]).string()},out/(level+".run.log"))!=0)
            throw std::runtime_error("Candidate execution failed: "+level);
        const auto summary=load(out/level/"SUMMARY.json");
        completed.push_back({{"level",level},{"methods",summary.at("methods")}});
        save(out/"SUMMARY.json",completed);
        std::ofstream report(out/"REPORT.md");
        report<<"# 冻结场景规划复测\n\nC++软动力学目标；求解器与表达式见各组SUMMARY.json；PVAJ MINCO；每段16区间；每方法8个共同原始初值。\n"
              <<"安全球0.20米，物理碰撞球0.15米。合格须通过有限约束、原连续几何复查和独立物理/顺序/终点检查；未实飞。\n\n"
              <<"|赛道|自由点时间/s|中心时间/s|自由点合格初值|中心合格初值|自由点计算/s|中心计算/s|\n|---|---:|---:|---:|---:|---:|---:|\n";
        report<<std::fixed<<std::setprecision(3);
        for(const auto& row:completed) {
            const auto a=row["methods"]["ours"],b=row["methods"]["center"];
            report<<'|'<<row["level"].get<std::string>()<<'|'<<time_cell(a)<<'|'<<time_cell(b)<<'|'
                <<a["successful_candidates"].get<int>()<<"/8|"<<b["successful_candidates"].get<int>()<<"/8|"
                <<a["total_planning_wall_s"].get<double>()<<'|'<<b["total_planning_wall_s"].get<double>()<<"|\n";
        }
        std::cout<<level<<" complete\n"<<std::flush;
    }
    save(out/"STATUS.json",{{"state","complete"},{"completed",completed},{"pid",getpid()}});
    return 0;
}

int main(int argc,char** argv) {
    try {
        if(argc>1 && std::string(argv[1])=="--candidate")return candidate(argc,argv);
        if(argc>1 && std::string(argv[1])=="--campaign")return campaign(argc,argv);
        if(argc>1 && std::string(argv[1])=="--campaign-robust")return campaign(argc,argv,true);
        return run_case(argc,argv);
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
