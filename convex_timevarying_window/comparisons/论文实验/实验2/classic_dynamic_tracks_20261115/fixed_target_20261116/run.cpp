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

int main(int argc, char **argv) {
    try {
        if (argc > 1 && std::string(argv[1]) == "--candidate") return candidate(argc, argv);
        if (argc != 6) throw std::runtime_error("Usage: RUN ORIGINAL_BIN FIXED_BIN ORIGINAL_INPUTS CONFIG NEW_OUTPUT");
        const fs::path originalBin = fs::absolute(argv[1]), fixedBin = fs::absolute(argv[2]),
            inputs = fs::absolute(argv[3]), config = fs::absolute(argv[4]), output = fs::absolute(argv[5]);
        if (fs::exists(output)) throw std::runtime_error("Output exists");
        fs::create_directories(output);
        const int lock = open((output / "coordinator.lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0644);
        if (flock(lock, LOCK_EX | LOCK_NB)) throw std::runtime_error("Coordinator already running");
        const auto protocol = load(config / "protocol.json");
        save(output / "PROTOCOL.json", protocol);
        const auto began = Clock::now();
        Json conditions = Json::array();
        for (const auto &levelValue : protocol.at("levels")) {
            const std::string level = levelValue;
            const fs::path directory = output / level, scene = inputs / "assets" / level / "scene.json";
            const std::string originalScene = contents(scene);
            fs::create_directories(directory / "seeds");
            const auto conditionBegan = Clock::now();
            save(output / "STATUS.json", {{"state", "running"}, {"level", level}, {"stage", "target_time_initialization"}, {"completed_conditions", conditions.size()}, {"pid", getpid()}});
            std::vector<Job> initializers;
            for (int index = 0; index < 8; ++index) {
                const auto target = directory / "seeds" / std::to_string(index);
                initializers.push_back({"seed_" + std::to_string(index),
                    {(originalBin / "dynamic_togt_planner").string(), scene.string(), target.string(), "center", (config / "time_seed").string(),
                     (inputs / "vehicle.json").string(), "--initial-manifest=" + (inputs / "inputs" / (level + "_center") / "manifest.json").string(),
                     "--initial-index=" + std::to_string(index)}, directory / "seeds" / (std::to_string(index) + ".log"), {}});
            }
            const auto initializationJobs = parallel(initializers, directory / "INITIALIZATION_STATUS.json");
            const double sharedInitialization = std::chrono::duration<double>(Clock::now() - conditionBegan).count();
            std::vector<Job> candidates;
            Json preparation = Json::array();
            for (int index = 0; index < 8; ++index) {
                const auto seedFile = directory / "seeds" / std::to_string(index) / "result.json";
                if (!fs::exists(seedFile) || !load(seedFile).value("found", false)) continue;
                for (const std::string method : {"ours", "center"}) {
                    const std::string name = method + "_" + std::to_string(index);
                    const auto graph = directory / "prepared" / name;
                    fs::create_directories(graph.parent_path());
                    const auto started = Clock::now();
                    const int code = execute({(fixedBin / "fixed_seed").string(), (inputs / "inputs" / (level + "_" + method)).string(),
                        seedFile.string(), scene.string(), graph.string()}, directory / "prepared" / (name + ".log"));
                    preparation.push_back({{"candidate", name}, {"exit_code", code}, {"wall_s", std::chrono::duration<double>(Clock::now() - started).count()}});
                    if (code) continue;
                    const auto target = directory / "candidates" / name;
                    fs::create_directories(target.parent_path());
                    candidates.push_back({name, {fs::absolute(argv[0]).string(), "--candidate", (originalBin / "sip_planner").string(),
                        (fixedBin / "runtime" / "mesh_check").string(), graph.string(), scene.string(), target.string()}, directory / "candidates" / (name + ".log"), {}});
                }
            }
            const auto setupFinished = Clock::now();
            save(output / "STATUS.json", {{"state", "running"}, {"level", level}, {"stage", "sip_planning"}, {"completed_conditions", conditions.size()}, {"pid", getpid()}});
            const auto candidateJobs = parallel(candidates, directory / "CANDIDATE_STATUS.json");
            Json methods = Json::object();
            for (const std::string method : {"ours", "center"}) {
                double bestObjective = std::numeric_limits<double>::infinity(), methodWaiting = 0.;
                Json summary{{"found", false}, {"successful_candidates", 0}, {"candidates", Json::array()}};
                for (const auto &job : candidateJobs) {
                    const std::string name = job.at("job");
                    if (name.rfind(method + "_", 0) != 0) continue;
                    methodWaiting = std::max(methodWaiting, job.at("wall_s").get<double>());
                    const auto folder = directory / "candidates" / name;
                    Json record = fs::exists(folder / "CANDIDATE.json") ? load(folder / "CANDIDATE.json") : Json{{"accepted", false}, {"classification", "execution_error"}};
                    record["name"] = name;
                    record["directory"] = fs::absolute(folder).string();
                    summary["candidates"].push_back(record);
                    if (!record.value("accepted", false)) continue;
                    summary["successful_candidates"] = summary["successful_candidates"].get<int>() + 1;
                    if (record.at("objective").get<double>() < bestObjective) {
                        bestObjective = record.at("objective");
                        summary["found"] = true;
                        summary["selected"] = name;
                        summary["nominal_time_s"] = record.at("nominal_time_s");
                        summary["objective"] = record.at("objective");
                        summary["result_directory"] = record.at("directory");
                    }
                }
                summary["shared_target_initialization_wall_s"] = sharedInitialization;
                summary["all_preparation_wall_s"] = std::chrono::duration<double>(setupFinished - conditionBegan).count() - sharedInitialization;
                summary["parallel_candidate_wait_wall_s"] = methodWaiting;
                summary["elapsed_to_final_selection_s"] = std::chrono::duration<double>(setupFinished - conditionBegan).count() + methodWaiting;
                methods[method] = summary;
            }
            if (contents(scene) != originalScene) throw std::runtime_error("Target scene changed during solving");
            Json condition{{"level", level}, {"target_scene", fs::absolute(scene).string()}, {"scene_bytes_unchanged", true}, {"methods", methods},
                {"initialization_jobs", initializationJobs}, {"preparation", preparation}, {"candidate_jobs", candidateJobs},
                {"wall_s", std::chrono::duration<double>(Clock::now() - conditionBegan).count()}};
            save(directory / "SUMMARY.json", condition);
            conditions.push_back(condition);
            save(output / "SUMMARY.json", {{"conditions", conditions}, {"wall_s", std::chrono::duration<double>(Clock::now() - began).count()}});
            std::cout << "CONDITION_COMPLETE " << level << std::endl;
        }
        save(output / "STATUS.json", {{"state", "complete"}, {"completed_conditions", conditions.size()}, {"pid", getpid()}});
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
