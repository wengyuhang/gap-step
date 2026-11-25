#include "mesh.hpp"
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/file.h>
#include <sys/wait.h>
#include <unistd.h>
namespace fs = std::filesystem;
Json read(const fs::path &p) {
    Json j;
    std::ifstream file(p);
    if (!file)
        throw std::runtime_error("Missing " + p.string());
    file >> j;
    return j;
}
void save(const fs::path &p, const Json &j) {
    fs::create_directories(p.parent_path());
    std::ofstream(p) << j.dump(2) << '\n';
}
void execute(const fs::path &binary, const fs::path &scene, const fs::path &out,
             const std::string &mode, const fs::path &params, const fs::path &vehicle) {
    fs::create_directories(out);
    const pid_t pid = fork();
    if (pid < 0)
        throw std::runtime_error("fork failed");
    if (pid == 0) {
        int fd = open((out / "process.log").c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
        dup2(fd, 1);
        dup2(fd, 2);
        close(fd);
        execl(binary.c_str(), binary.c_str(), scene.c_str(), out.c_str(), mode.c_str(),
              params.c_str(), vehicle.c_str(), nullptr);
        _exit(127);
    }
    int status = 0;
    waitpid(pid, &status, 0);
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
        throw std::runtime_error("Solver process failed " + out.string());
}
int main(int argc, char **argv) {
    try {
        if (argc != 6 && argc != 7)
            throw std::runtime_error("Usage: batch BINARY TEN_LEVEL_INPUTS POINT7_INPUTS "
                                     "AUTHOR_PARAMS OUTPUT [POINT_INSET_M]");
        const fs::path binary = fs::absolute(argv[1]), old = fs::absolute(argv[2]),
                       point7 = fs::absolute(argv[3]), params = fs::absolute(argv[4]),
                       out = fs::absolute(argv[5]);
        fs::create_directories(out);
        int lock = open((out / "coordinator.lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0644);
        if (flock(lock, LOCK_EX | LOCK_NB))
            throw std::runtime_error("Batch already running");
        auto vehicle = read(point7 / "vehicle.json");
        if (argc == 7) {
            const double inset = std::stod(argv[6]);
            if (!std::isfinite(inset) ||
                inset < vehicle.at("planning_proxy_radius_m").get<double>())
                throw std::runtime_error(
                    "Point inset must be at least the existing collision proxy radius");
            vehicle["nominal_point_inset_m"] = inset;
            vehicle["extra_point_clearance_m"] =
                inset - vehicle.at("planning_proxy_radius_m").get<double>();
            vehicle["finite_safety_radius_m"] =
                inset + .085 + vehicle.at("finite_numerical_margin_m").get<double>();
        }
        const fs::path vehicle_path = out / "vehicle.json";
        save(vehicle_path, vehicle);
        const auto point_scene = read(point7 / "assets/static/scene.json");
        const std::vector<std::string> levels{"static", "mixed_base", "a25_f3", "a25_f6", "a50_f1",
                                              "a50_f3", "a50_f6",     "a75_f1", "a75_f3", "a75_f6"};
        Json rows = Json::array();
        for (const auto &level : levels) {
            auto scene =
                level == "static" ? point_scene : read(old / "assets" / level / "scene.json");
            scene["windows"][6] = point_scene["windows"][6];
            scene["vehicle"] = vehicle;
            scene["scenario_variant"] = "six_physical_moving_gates_original_static_singleball7";
            const fs::path input = out / "scenes" / (level + ".json");
            save(input, scene);
            save(out / "status.json", Json{{"state", "running"},
                                           {"level", level},
                                           {"finished_methods", rows.size()},
                                           {"pid", getpid()},
                                           {"gazebo", false}});
            for (const std::string method : {"dynamic_togt_center", "dynamic_togt"}) {
                const auto folder = out / level / method, file = folder / "result.json";
                if (!fs::exists(file))
                    execute(binary, input, folder, method == "dynamic_togt" ? "free" : "center",
                            params, vehicle_path);
                const auto result = read(file);
                Json assessment = nullptr;
                double check_s = 0.;
                std::string verdict = "未找到规划";
                if (result["found"].get<bool>()) {
                    assessment = mesh_evaluate(Trajectory::load(result), scene);
                    check_s = assessment["check_s"];
                    save(folder / "assessment.json", assessment);
                    verdict = assessment["collision_detected"].get<bool>() ? "碰撞"
                              : !assessment["ordered_exactly_once"].get<bool>() ||
                                      !assessment["endpoints_passed"].get<bool>()
                                  ? "未完成"
                              : assessment["passed"].get<bool>() ? "通过"
                                                                 : "证据不足";
                }
                rows.push_back(Json{
                    {"level", level},
                    {"method", method},
                    {"outcome", verdict},
                    {"trajectory_time_s", result["nominal_time_s"]},
                    {"solve_wall_s", result["total_compute_s"]},
                    {"compute_with_check_s", result["total_compute_s"].get<double>() + check_s},
                    {"iterations", result.value("iterations", -1)},
                    {"source", file.string()}});
                save(out / "SUMMARY.json", rows);
            }
            std::cout << "COMPLETE " << level << std::endl;
        }
        for (const std::string mode : {"free", "center"}) {
            const auto reference_folder = out / "static" / ("original_" + mode);
            if (!fs::exists(reference_folder / "result.json"))
                execute(binary.parent_path() / "togt_original_static_reference",
                        out / "scenes/static.json", reference_folder, mode, params, vehicle_path);
        }
        Json static_equivalence = Json::array();
        for (const std::string mode : {"free", "center"}) {
            const auto actual = read(out / "static" /
                                     (mode == "free" ? "dynamic_togt" : "dynamic_togt_center") /
                                     "result.json"),
                       reference = read(out / "static" / ("original_" + mode) / "result.json");
            double error = 0., initial = 0.;
            for (size_t i = 0; i < actual["variables"].size(); ++i) {
                error = std::max(error, std::abs(actual["variables"][i].get<double>() -
                                                 reference["variables"][i].get<double>()));
                initial =
                    std::max(initial, std::abs(actual["initial_variables"][i].get<double>() -
                                               reference["initial_variables"][i].get<double>()));
            }
            if (error > 1e-10 || initial > 1e-10)
                throw std::runtime_error("Original static solver mismatch");
            static_equivalence.push_back(Json{{"mode", mode},
                                              {"passed", true},
                                              {"final_variable_error", error},
                                              {"initial_variable_error", initial}});
        }
        save(out / "STATIC_SOURCE_EQUIVALENCE.json", static_equivalence);
        std::ofstream f(out / "REPORT.md");
        f << "# "
             "基于作者TOGT源代码的动态适配\n\n2026-11-"
             "15，十档，仅规划。六实体门及原第七SingleBall航点。中心与自由点均使用作者TrajData距离/"
             "速度单初值，各自直接调用TrajSolver求解。自由点直接联合优化逐段时长和选点。作者原L-"
             "BFGS参数与动力学罚项沿用，动态扩展为累计到达时刻的门位置与链式时间梯度。\n\n";
        f << "每侧选点内缩为" << vehicle.at("nominal_point_inset_m").get<double>()
          << "米，机体碰撞球半径为" << vehicle.at("planning_proxy_radius_m").get<double>()
          << "米。中心/自由各独立单初值。\n\n";
        f << "|幅度比|频率倍率|TOGT自由点/s|TOGT中心/s|\n|---|---|---:|---:|\n";
        for (const auto &level : levels) {
            f << '|'
              << (level == "static"                                      ? "0"
                  : level == "mixed_base" || level.substr(0, 3) == "a25" ? "0.25"
                  : level.substr(0, 3) == "a50"                          ? "0.50"
                                                                         : "0.75")
              << '|' << (level == "static" || level == "mixed_base" ? "1" : level.substr(5)) << '|';
            for (const std::string method : {"dynamic_togt", "dynamic_togt_center"})
                for (const auto &row : rows)
                    if (row["level"] == level && row["method"] == method) {
                        if (row["outcome"] == "通过")
                            f << std::fixed << std::setprecision(3)
                              << row["trajectory_time_s"].get<double>();
                        else
                            f << row["outcome"].get<std::string>();
                        f << '|';
                    }
            f << '\n';
        }
        f << "\n## "
             "计算记录\n\n|条件|方法|轨迹时间/s|迭代数|求解墙钟/s|含复查/"
             "s|结果|\n|---|---|---:|---:|---:|---:|---|\n";
        for (const auto &row : rows)
            f << '|' << row["level"].get<std::string>() << '|' << row["method"].get<std::string>()
              << '|' << row["trajectory_time_s"].get<double>() << '|'
              << row["iterations"].get<int>() << '|' << row["solve_wall_s"].get<double>() << '|'
              << row["compute_with_check_s"].get<double>() << '|'
              << row["outcome"].get<std::string>() << "|\n";
        f << "\n静态自由与中心分别和未修改作者TrajSolver对照，初值及最终变量逐值一致，见STATIC_"
             "SOURCE_EQUIVALENCE.json。窗框/门序/航点/"
             "PVAJ复查保存在各方法assessment."
             "json，地面按既有口径单独记录。完整时间包含第七航点到作者终点的末段。旧成绩保留，新表"
             "独立记录。\n";
        save(out / "status.json", Json{{"state", "complete"},
                                       {"methods", rows.size()},
                                       {"conditions", levels.size()},
                                       {"pid", getpid()},
                                       {"gazebo", false}});
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
