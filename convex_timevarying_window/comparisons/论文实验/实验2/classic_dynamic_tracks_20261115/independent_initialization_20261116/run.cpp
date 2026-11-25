#include "json.hpp"
#include <chrono>
#include <cerrno>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
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
        completed.push_back({{"job", found->second.name}, {"exit_code", exitCode(status)}, {"wall_s", elapsed}});
        std::cout << found->second.name << " finished " << exitCode(status) << '\n' << std::flush;
        active.erase(found);
        update();
    }
    return completed;
}

int candidate(int argc, char **argv) {
    if (argc != 7) throw std::runtime_error("candidate SIP MESH INPUT SCENE OUTPUT");
    const fs::path output = argv[6];
    fs::create_directories(output);
    const auto began = Clock::now();
    const int code = execute({argv[2], argv[4], output.string(), "0", "--soft-mincopv", argv[5], "1000"}, output / "sip.log");
    Json result{{"sip_exit_code", code}, {"accepted", false}, {"classification", "execution_error"}};
    if (fs::exists(output / "result.json")) {
        const auto raw = load(output / "result.json");
        result["sip_found"] = raw.value("found", false);
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


int main(int argc,char** argv){try{
 if(argc!=4)throw std::runtime_error("Usage: PREPARED SCENE NEW_OUTPUT");fs::path inputs=fs::absolute(argv[1]),scene=fs::absolute(argv[2]),out=fs::absolute(argv[3]);if(fs::exists(out))throw std::runtime_error("Output exists");fs::create_directories(out);std::vector<Job> jobs;
 for(std::string method:{"ours","center"})for(int seed=0;seed<8;++seed){std::string name=method+"_"+std::to_string(seed);jobs.push_back({name,{"/tmp/fixed_target_build_20261116/fixed_target_run","--candidate","/tmp/long_solver_repair_build_20261116/sip_planner","/tmp/fixed_target_build_20261116/runtime/mesh_check",(inputs/name).string(),scene.string(),(out/name).string()},out/(name+".log"),{}});}
 const auto started=Clock::now();save(out/"STATUS.json",{{"state","running"},{"pid",getpid()}});auto records=parallel(jobs,out/"CANDIDATE_STATUS.json");Json methods=Json::object();
 for(std::string method:{"ours","center"}){Json selected={{"found",false},{"successful_candidates",0}};double best=INFINITY;for(int seed=0;seed<8;++seed){std::string name=method+"_"+std::to_string(seed);auto c=load(out/name/"CANDIDATE.json");if(!c.value("accepted",false))continue;selected["successful_candidates"]=selected["successful_candidates"].get<int>()+1;double value=c["objective"];if(value<best){best=value;selected["found"]=true;selected["nominal_time_s"]=c["nominal_time_s"];selected["objective"]=value;selected["selected"]=name;}}methods[method]=selected;}
 save(out/"SUMMARY.json",{{"methods",methods},{"jobs",records},{"wall_s",std::chrono::duration<double>(Clock::now()-started).count()}});save(out/"STATUS.json",{{"state","complete"},{"pid",getpid()}});
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;}
