#include "mesh.hpp"
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sys/file.h>
#include <sys/wait.h>
#include <unistd.h>
namespace fs = std::filesystem;
Json read(const fs::path &p) {
    std::ifstream f(p);
    if (!f)
        throw std::runtime_error("Missing " + p.string());
    Json j;
    f >> j;
    return j;
}
void save(const fs::path &p, const Json &j) {
    std::ofstream f(p.string() + ".writing");
    f << j.dump(2) << '\n';
    f.close();
    fs::rename(p.string() + ".writing", p);
}
double execute(const std::vector<std::string> &args, const fs::path &log) {
    const auto start = std::chrono::steady_clock::now();
    const pid_t pid = fork();
    if (pid < 0)
        throw std::runtime_error("fork failed");
    if (!pid) {
        int fd = open(log.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
        dup2(fd, 1);
        dup2(fd, 2);
        close(fd);
        std::vector<char *> v;
        for (const auto &s : args)
            v.push_back(const_cast<char *>(s.c_str()));
        v.push_back(nullptr);
        execv(v[0], v.data());
        _exit(127);
    }
    int status;
    waitpid(pid, &status, 0);
    if (!WIFEXITED(status) || WEXITSTATUS(status))
        throw std::runtime_error("Process failed " + log.string());
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}
int main(int argc, char **argv) {
    try {
        if (argc != 6)
            throw std::runtime_error(
                "Usage: run_baselines BIN INPUTS WANG_GRAPHS AUTHOR_PARAMS OUTPUT");
        const fs::path bin = fs::absolute(argv[1]), inputs = fs::absolute(argv[2]),
                       wang = fs::absolute(argv[3]), params = fs::absolute(argv[4]),
                       out = fs::absolute(argv[5]);
        fs::create_directories(out);
        const int lock =
            open((out / "baselines_coordinator.lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0644);
        if (flock(lock, LOCK_EX | LOCK_NB))
            throw std::runtime_error("Baseline coordinator already running");
        const auto levels = read(inputs / "PROTOCOL.json").at("levels").get<std::vector<std::string>>();
        Json rows = Json::array();
        for (const auto &level : levels) {
            auto scene = read(inputs / "assets" / level / "scene.json");
            for (const std::string method : {"dynamic_togt", "dynamic_togt_center", "wang2023"}) {
                save(out / "BASELINES_STATUS.json", Json{{"state", "running"},
                                                         {"level", level},
                                                         {"method", method},
                                                         {"completed_methods", rows.size()},
                                                         {"pid", getpid()}});
                const auto folder = out / level / "baselines" / method;
                fs::create_directories(folder);
                const auto file = folder / "result.json", timing = folder / "TIMING.json";
                if (!fs::exists(timing)) {
                    std::vector<std::string> args;
                    if (method == "wang2023")
                        args = {(bin / "wang_planner").string(),
                                (wang / (level + "_wang2023")).string(), folder.string(), "solve"};
                    else
                        args = {(bin / "dynamic_togt_planner").string(),
                                (inputs / "assets" / level / "scene.json").string(),
                                folder.string(),
                                method == "dynamic_togt" ? "free" : "center",
                                params.string(),
                                (inputs / "vehicle.json").string()};
                    double wall = execute(args, folder / "process.log");
                    const auto check_begin = std::chrono::steady_clock::now();
                    const auto result = read(file);
                    if (result.value("found", false))
                        save(folder / "assessment.json",
                             mesh_evaluate(Trajectory::load(result), scene));
                    else if (!result["durations"].empty()) {
                        auto partial_scene = scene;
                        partial_scene["windows"].erase(partial_scene["windows"].end() - 1);
                        save(folder / "partial_assessment.json",
                             mesh_evaluate(Trajectory::load(result), partial_scene));
                    }
                    const double check_s = std::chrono::duration<double>(
                                               std::chrono::steady_clock::now() - check_begin)
                                               .count();
                    save(timing, Json{{"process_wall_s", wall},
                                      {"verification_wall_s", check_s},
                                      {"total_wall_s", wall + check_s}});
                }
                const auto result = read(file), t = read(timing);
                const bool complete = result.value("found", false);
                std::string outcome = "未找到完整规划";
                Json a = nullptr;
                if (complete) {
                    a = read(folder / "assessment.json");
                    outcome = a["collision_detected"].get<bool>() ? "碰撞"
                              : !a["ordered_exactly_once"].get<bool>() ||
                                      !a["endpoints_passed"].get<bool>()
                                  ? "未完成"
                              : a["passed"].get<bool>() ? "通过"
                                                        : "证据不足";
                } else if (fs::exists(folder / "partial_assessment.json"))
                    a = read(folder / "partial_assessment.json");
                rows.push_back(Json{
                    {"level", level},
                    {"method", method},
                    {"outcome", outcome},
                    {"trajectory_time_s",
                     complete ? result.value("nominal_time_s", Json(nullptr)) : Json(nullptr)},
                    {"partial_time_s", !complete && !result["durations"].empty()
                                           ? result["nominal_time_s"]
                                           : Json(nullptr)},
                    {"collision_detected", a.is_null() ? Json(nullptr) : a["collision_detected"]},
                    {"compute_wall_s", t["total_wall_s"]},
                    {"source", file.string()}});
                save(out / "BASELINE_SUMMARY.json", rows);
                std::cout << "BASELINE_COMPLETE " << level << ' ' << method << ' ' << outcome
                          << std::endl;
            }
        }
        save(out / "BASELINES_STATUS.json",
             Json{{"state", "complete"}, {"completed_methods", rows.size()}, {"pid", getpid()}});
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
