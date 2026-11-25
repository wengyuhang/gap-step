#include "json.hpp"
#include "sip_exchange_policy.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <set>
namespace fs = std::filesystem;
using Json = nlohmann::json;
Json load(const fs::path &file) {
    std::ifstream stream(file);
    if (!stream) throw std::runtime_error("Missing " + file.string());
    Json value;
    stream >> value;
    return value;
}
void require(bool value, const std::string &message) {
    if (!value) throw std::runtime_error(message);
}
int main(int argc, char **argv) {
    try {
        if (argc != 4) throw std::runtime_error("Usage: ORIGINAL_INPUTS RESULT_DIRECTORY OUTPUT_AUDIT_JSON");
        const fs::path inputs = fs::absolute(argv[1]), results = fs::absolute(argv[2]);
        require(load(results / "STATUS.json").at("state") == "complete", "Batch incomplete");
        const auto protocol = load(results / "PROTOCOL.json");
        const auto summary = load(results / "SUMMARY.json");
        require(summary.at("conditions").size() == protocol.at("levels").size(), "Condition count mismatch");
        Json conditions = Json::array();
        int initializerCount = 0, initializerFailed = 0, candidateCount = 0, acceptedCount = 0, finiteCount = 0, acceptableCount = 0;
        for (const auto &condition : summary.at("conditions")) {
            const std::string level = condition.at("level");
            const auto originalScene = inputs / "assets" / level / "scene.json";
            require(fs::path(condition.at("target_scene").get<std::string>()) == originalScene, "Different target scene");
            require(condition.at("scene_bytes_unchanged").get<bool>(), "Scene bytes changed");
            Json methods = Json::object();
            int generatedSeeds = 0;
            for (int index = 0; index < 8; ++index) {
                const auto seed = load(results / level / "seeds" / std::to_string(index) / "result.json");
                ++initializerCount;
                require(fs::path(seed.at("scene_source").get<std::string>()) == originalScene, "Initializer used different scene");
                require(seed.at("initial_sample_index") == index, "Seed index changed");
                if (seed.value("found", false)) ++generatedSeeds;
                else ++initializerFailed;
            }
            for (const std::string method : {"ours", "center"}) {
                const auto original = load(inputs / "inputs" / (level + "_" + method) / "manifest.json");
                const auto &selected = condition.at("methods").at(method);
                require(selected.at("candidates").size() == static_cast<size_t>(generatedSeeds), "Generated seeds and SIP candidates differ");
                int accepted = 0;
                double minimumObjective = std::numeric_limits<double>::infinity();
                for (int index = 0; index < 8; ++index) {
                    const std::string name = method + "_" + std::to_string(index);
                    const auto folder = results / level / "candidates" / name;
                    const auto prepared = results / level / "prepared" / name;
                    const auto seed = load(results / level / "seeds" / std::to_string(index) / "result.json");
                    if (!seed.value("found", false)) {
                        require(!fs::exists(folder / "result.json") && !fs::exists(prepared / "manifest.json"), "Failed initializer was advanced");
                        continue;
                    }
                    const auto graph = load(prepared / "manifest.json");
                    const auto record = load(folder / "CANDIDATE.json");
                    const auto raw = load(folder / "result.json");
                    ++candidateCount;
                    require(graph.at("windows") == original.at("windows"), "Prepared window motion changed");
                    require(graph.at("initialization").at("source_initial_index") == index, "Restoration seed mismatch");
                    require(graph.at("initials").size() == 1, "Unexpected added initializations");
                    require(!graph.contains("initial_finite_warm_start") && !graph.contains("initial_geometry_witnesses"), "Cross-request warm state injected");
                    require(graph.at("free_points").get<bool>() == (method == "ours"), "Center/free point variables mismatch");
                    for (const auto &item : original.items()) {
                        if (item.key() == "initials" || item.key() == "initialization" || item.key() == "solver_options") continue;
                        require(graph.at(item.key()) == item.value(), "Non-initialization field changed " + item.key());
                    }
                    const int pieces = graph.at("n"), start = graph.at("velocity_start");
                    const auto originalVariables = graph.at("initials").at(0).at("original");
                    for (int piece = 0; piece < pieces; ++piece)
                        require(std::abs(std::exp(originalVariables.at(piece).get<double>()) - seed.at("durations").at(piece).get<double>()) < 1e-10, "Initial duration mismatch");
                    for (int k = pieces; k < start; ++k)
                        require(originalVariables.at(k).get<double>() == 0., "Point not initialized at center");
                    require(graph.at("solver_options").at("ipopt.max_iter") == 5000, "Iteration limit changed");
                    for (const std::string key : {"bound_push", "bound_frac", "slack_bound_push", "slack_bound_frac", "warm_start_bound_push", "warm_start_mult_bound_push", "warm_start_slack_bound_push", "warm_start_bound_frac", "warm_start_slack_bound_frac"})
                        require(graph.at("solver_options").at("ipopt." + key).get<double>() == 1e-9, "Startup option mismatch");
                    const auto restoration = load(prepared / "initial_diagnostic.json");
                    require(restoration.at("coefficient_error").get<double>() < 1e-5 && restoration.at("center_error_m").get<double>() < 1e-8, "PVAJ restoration failed");
                    for (const auto &call : raw.at("optimizer_calls")) {
                        ++finiteCount;
                        if (call.at("status") == "Solved_To_Acceptable_Level") ++acceptableCount;
                        if (call.at("warm_start_used").get<bool>())
                            require(call.at("dual_source_status") == "Solve_Succeeded" && call.at("previous_solve_status") == "Solve_Succeeded", "Invalid dual warm start");
                    }
                    if (record.at("accepted").get<bool>()) {
                        ++accepted;
                        ++acceptedCount;
                        require(raw.value("found", false), "Accepted unverified SIP result");
                        const auto mesh = load(folder / "mesh_assessment.json");
                        require(mesh.at("passed").get<bool>() && !mesh.at("collision_detected").get<bool>() &&
                            mesh.at("ordered_exactly_once").get<bool>() && mesh.at("endpoints_passed").get<bool>(), "Invalid independent verification");
                        for (const auto &call : raw.at("optimizer_calls"))
                            require(sip_exchange::optimizer_success(call.at("status").get<std::string>()) && call.at("constraint_violation_inf").get<double>() <= 1e-5, "Accepted unconverged finite problem");
                        minimumObjective = std::min(minimumObjective, raw.at("objective").get<double>());
                    }
                }
                require(selected.at("successful_candidates") == accepted, "Candidate success count mismatch");
                require(selected.at("found").get<bool>() == (accepted > 0), "Method verdict mismatch");
                if (accepted) require(std::abs(selected.at("objective").get<double>() - minimumObjective) < 1e-12, "Selection is not minimum original objective");
                methods[method] = {{"accepted", accepted}, {"tested", 8}, {"same_initialization_rule", true}, {"target_motion_unchanged", true}};
            }
            for (int index = 0; index < 8; ++index) {
                const auto seed = load(results / level / "seeds" / std::to_string(index) / "result.json");
                if (!seed.value("found", false)) continue;
                const auto free = load(results / level / "prepared" / ("ours_" + std::to_string(index)) / "manifest.json");
                const auto center = load(results / level / "prepared" / ("center_" + std::to_string(index)) / "manifest.json");
                const auto &a = free.at("initials").at(0).at("original"), &b = center.at("initials").at(0).at("original");
                const int pieces = free.at("n"), fstart = free.at("velocity_start"), cstart = center.at("velocity_start");
                for (int k = 0; k < pieces; ++k) require(a.at(k) == b.at(k), "Paired times differ");
                for (int k = cstart; k < b.size(); ++k) require(a.at(fstart + k - cstart) == b.at(k), "Paired PVAJ initial values differ");
            }
            conditions.push_back({{"level", level}, {"methods", methods}, {"paired_physical_time_and_pvaj_equal", true}});
        }
        Json audit{{"passed", true}, {"initializers", initializerCount}, {"initializer_failures", initializerFailed}, {"logical_method_attempts", initializerCount*2}, {"candidates", candidateCount}, {"accepted_candidates", acceptedCount},
            {"finite_solver_calls", finiteCount}, {"solved_to_acceptable_level_calls", acceptableCount}, {"finite_acceptance_rule", "original optimizer_success plus finite feasibility, strict Solve_Succeeded only for multiplier reuse"}, {"conditions", conditions}};
        std::ofstream(argv[3]) << audit.dump(2) << '\n';
        std::cout << "Verified " << initializerCount << " target-only initializers and " << candidateCount << " candidates\n";
    } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
    return 0;
}
