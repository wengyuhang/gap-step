#pragma once
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <spawn.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

extern char** environ;

class Process {
    pid_t pid = -1;
    int reader = -1;
public:
    Process() = default;
    Process(const Process&) = delete;
    Process& operator=(const Process&) = delete;
    ~Process() { stop(); }

    void start(std::vector<std::string> arguments, const std::string& output = "", bool pipe_output = false) {
        int descriptors[2] = {-1, -1};
        if (pipe_output && pipe2(descriptors, O_CLOEXEC)) throw std::runtime_error("Cannot open process pipe");
        const int file = output.empty() ? -1 : open(output.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
        const int errors = open("/dev/null", O_WRONLY | O_CLOEXEC);
        posix_spawn_file_actions_t actions;
        posix_spawn_file_actions_init(&actions);
        if (pipe_output) posix_spawn_file_actions_adddup2(&actions, descriptors[1], STDOUT_FILENO);
        else if (file >= 0) posix_spawn_file_actions_adddup2(&actions, file, STDOUT_FILENO);
        posix_spawn_file_actions_adddup2(&actions, errors, STDERR_FILENO);
        posix_spawnattr_t attributes;
        posix_spawnattr_init(&attributes);
        posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP);
        posix_spawnattr_setpgroup(&attributes, 0);
        std::vector<char*> argv;
        for (auto& value : arguments) argv.push_back(value.data());
        argv.push_back(nullptr);
        const int error = posix_spawnp(&pid, argv[0], &actions, &attributes, argv.data(), environ);
        posix_spawn_file_actions_destroy(&actions);
        posix_spawnattr_destroy(&attributes);
        if (file >= 0) close(file);
        close(errors);
        if (pipe_output) { close(descriptors[1]); reader = descriptors[0]; }
        if (error) throw std::runtime_error("Cannot start subprocess " + arguments.front());
    }

    int release_fd() { const int result = reader; reader = -1; return result; }
    int wait() {
        if (pid < 0) return 0;
        int status = 0;
        while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
        pid = -1;
        return WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
    }
    void stop() {
        if (pid > 0) {
            kill(-pid, SIGTERM);
            int status = 0;
            while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
            pid = -1;
        }
        if (reader >= 0) { close(reader); reader = -1; }
    }
};

inline void run_process(const std::vector<std::string>& arguments) {
    Process process;
    process.start(arguments, "/dev/null");
    if (process.wait() != 0) throw std::runtime_error("Subprocess failed: " + arguments.front());
}

inline std::string capture_process(const std::vector<std::string>& arguments) {
    Process process;
    process.start(arguments, "", true);
    FILE* stream = fdopen(process.release_fd(), "r");
    char* line = nullptr;
    size_t size = 0;
    std::string output;
    while (getline(&line, &size, stream) >= 0) output += line;
    free(line);
    fclose(stream);
    if (process.wait() != 0) throw std::runtime_error("Subprocess failed: " + arguments.front());
    return output;
}
