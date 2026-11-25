#include "json.hpp"
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sys/file.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>
namespace fs = std::filesystem;
using J = nlohmann::json;
J read(fs::path p) {
  std::ifstream f(p);
  J j;
  f >> j;
  return j;
}
void save(fs::path p, J j) { std::ofstream(p) << j.dump(2) << '\n'; }
int main(int argc, char **argv) {
  if (argc != 5)
    return 1;
  fs::path inputs = fs::absolute(argv[1]), binary = fs::absolute(argv[2]),
           out = fs::absolute(argv[3]);
  int workers = std::stoi(argv[4]);
  fs::create_directories(out);
  int lock = open((out / "coordinator.lock").c_str(),
                  O_CREAT | O_RDWR | O_CLOEXEC, 0644);
  if (lock < 0 || flock(lock, LOCK_EX | LOCK_NB))
    return 2;
  auto levels =
      read(inputs / "PROTOCOL.json")["levels"].get<std::vector<std::string>>();
  struct Job {
    pid_t pid;
    std::string level;
  };
  std::vector<Job> active;
  size_t next = 1;
  J exits = J::object();
  while (next < levels.size() || !active.empty()) {
    while (next < levels.size() && int(active.size()) < workers) {
      std::string lv = levels[next++];
      fs::path folder = out / lv;
      fs::create_directories(folder);
      if (fs::exists(folder / "result.json")) {
        std::cerr << "Refusing repeated condition " << lv << '\n';
        return 3;
      }
      pid_t pid = fork();
      if (pid == 0) {
        int fd = open((folder / "process.log").c_str(),
                      O_CREAT | O_WRONLY | O_TRUNC, 0644);
        dup2(fd, 1);
        dup2(fd, 2);
        close(fd);
        std::string scene = (inputs / "assets" / lv / "scene.json").string(),
                    vehicle = (inputs / "vehicle.json").string(),
                    dest = folder.string();
        execl(binary.c_str(), binary.c_str(), scene.c_str(), vehicle.c_str(),
              dest.c_str(), "400", nullptr);
        _exit(127);
      }
      active.push_back({pid, lv});
      std::cout << "START " << lv << " " << pid << std::endl;
    }
    for (size_t k = 0; k < active.size();) {
      int status = 0;
      pid_t done = waitpid(active[k].pid, &status, WNOHANG);
      if (done > 0) {
        exits[active[k].level] =
            WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
        std::cout << "EXIT " << active[k].level << " " << exits[active[k].level]
                  << std::endl;
        active.erase(active.begin() + k);
      } else
        ++k;
    }
    J jobs = J::array(), finished = J::array();
    for (auto &job : active)
      jobs.push_back({{"pid", job.pid}, {"level", job.level}});
    for (auto &lv : levels)
      if (fs::exists(out / lv / "result.json"))
        finished.push_back(lv);
    save(out / "STATUS.json", {{"state", "running"},
                               {"pid", getpid()},
                               {"active", jobs},
                               {"completed", finished},
                               {"exits", exits},
                               {"static_external", true},
                               {"configured_workers", workers}});
    usleep(250000);
  }
  bool static_done = fs::exists(out / "static/result.json");
  save(out / "STATUS.json", {{"state", static_done ? "planning_complete"
                                                   : "waiting_external_static"},
                             {"pid", getpid()},
                             {"exits", exits},
                             {"configured_workers", workers},
                             {"geometry_audit", "pending"}});
  return 0;
}
