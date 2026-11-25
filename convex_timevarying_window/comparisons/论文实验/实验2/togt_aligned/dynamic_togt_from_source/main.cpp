#include "adapter.hpp"
#include <iostream>
int main(int argc, char **argv) {
    try {
        if (argc < 4)
            throw std::runtime_error("Usage: dynamic_togt_planner SCENE OUTPUT free|center "
                                     "[AUTHOR_PARAMETERS VEHICLE] [--center-result=RESULT]");
        const std::string mode = argv[3];
        if (mode != "free" && mode != "center")
            throw std::runtime_error("Expected free or center");
        std::vector<std::string> positional;
        std::filesystem::path center_file, initial_file;
        int initial_index = 0;
        for (int i = 4; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg.rfind("--center-result=", 0) == 0)
                center_file = arg.substr(16);
            else if (arg.rfind("--initial-manifest=", 0) == 0)
                initial_file = arg.substr(19);
            else if (arg.rfind("--initial-index=", 0) == 0)
                initial_index = std::stoi(arg.substr(16));
            else
                positional.push_back(arg);
        }
        if (positional.size() > 2 ||
            (!initial_file.empty() && (mode != "center" || !center_file.empty())))
            throw std::runtime_error("Unexpected dynamic TOGT argument");
        const auto params = positional.empty() ? std::filesystem::path(TOGT_UPSTREAM_PARAMETER_DIR)
                                               : std::filesystem::path(positional[0]);
        const auto vehicle_file = positional.size() < 2 ? std::filesystem::path(TOGT_X500_CONFIG)
                                                        : std::filesystem::path(positional[1]);
        const auto began = std::chrono::steady_clock::now();
        auto scene = togt_source::read(argv[1]);
        const auto vehicle = positional.size() < 2 && scene.contains("vehicle")
                                 ? scene.at("vehicle")
                                 : togt_source::read(vehicle_file);
#ifdef TOGT_STATIC_REFERENCE
        constexpr bool motion = false;
#else
        constexpr bool motion = true;
#endif
        togt_source::Planner planner(scene, vehicle, params, mode == "center", motion);
        double initial_cost = 0.;
        if (!center_file.empty()) {
            const auto center = togt_source::read(center_file);
            planner.setCenterInitial(center);
            initial_cost = center.value("total_compute_s", 0.);
        }
        if (!initial_file.empty())
            planner.setArchivedTimes(togt_source::read(initial_file), initial_index);
        auto result = planner.solve();
        if (!initial_file.empty()) {
            result["initialization_origin"] = "explicit archived independent segment timing";
            result["initial_sample_index"] = initial_index;
        }
        if (!center_file.empty())
            result["initialization_origin"] = "explicit optional center-result initialization";
        result["scene_source"] = std::filesystem::absolute(argv[1]).string();
        result["scenario_variant"] = scene.value("scenario_variant", std::string("custom"));
        result["case"] = {{"layout", "togt_whole"},
                          {"level", scene.value("level", std::string("custom"))}};
        result["build_and_solve_s"] =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
        result["center_generation_s"] = initial_cost;
        result["total_compute_s"] = initial_cost + result["build_and_solve_s"].get<double>();
        togt_source::save(std::filesystem::path(argv[2]) / "result.json", result);
        std::cout << mode << ' ' << result["found"] << ' ' << result["nominal_time_s"] << ' '
                  << result["total_compute_s"] << '\n';
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
