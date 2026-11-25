#include "json.hpp"
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sys/file.h>
#include <sys/wait.h>
#include <unistd.h>
namespace fs = std::filesystem;
using Json = nlohmann::json;
void save(const fs::path &p, const Json &j) {
    std::ofstream tmp(p.string() + ".writing");
    tmp << j.dump(2) << '\n';
    tmp.close();
    fs::rename(p.string() + ".writing", p);
}
int main(int argc, char **argv) {
    try {
        if (argc != 4)
            throw std::runtime_error("Usage: run_ours BIN INPUTS OUTPUT");
        const fs::path bin = fs::absolute(argv[1]), input = fs::absolute(argv[2]),
                       out = fs::absolute(argv[3]);
        fs::create_directories(out);
        int lock =
            open((out / "ours_coordinator.lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0644);
        if (flock(lock, LOCK_EX | LOCK_NB))
            throw std::runtime_error("Ours coordinator already running");
        Json protocol;
        std::ifstream(input / "PROTOCOL.json") >> protocol;
        const auto levels = protocol.at("levels").get<std::vector<std::string>>();
        int completed = 0;
        for (const auto &level : levels) {
            save(out / "OURS_STATUS.json", Json{{"state", "running"},
                                                {"level", level},
                                                {"completed_conditions", completed},
                                                {"pid", getpid()}});
            fs::path target = out / level;
            fs::create_directories(target);
            const auto status_file = target / "status.json";
            if (fs::exists(status_file)) {
                Json s;
                std::ifstream(status_file) >> s;
                if (s["state"] != "planning_complete")
                    throw std::runtime_error(
                        "Existing condition is incomplete; diagnose before restarting");
            } else {
                pid_t pid = fork();
                if (pid < 0)
                    throw std::runtime_error("fork failed");
                if (!pid) {
                    int fd =
                        open((target / "matrix.log").c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
                    dup2(fd, 1);
                    dup2(fd, 2);
                    close(fd);
                    const std::string binary = (bin / "matrix").string(), b = bin.string(),
                                      g = (input / "inputs").string(),
                                      a = (input / "assets").string(), o = target.string();
                    execl(binary.c_str(), binary.c_str(), b.c_str(), g.c_str(), a.c_str(),
                          o.c_str(), "16", level.c_str(), "--main-xy-eight", nullptr);
                    _exit(127);
                }
                int code;
                waitpid(pid, &code, 0);
                if (!WIFEXITED(code) || WEXITSTATUS(code))
                    throw std::runtime_error("Matrix failed for " + level);
            }
            ++completed;
            std::cout << "OURS_COMPLETE " << level << std::endl;
        }
        save(out / "OURS_STATUS.json",
             Json{{"state", "complete"}, {"completed_conditions", completed}, {"pid", getpid()}});
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
