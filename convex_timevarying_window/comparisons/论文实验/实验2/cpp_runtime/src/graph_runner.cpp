#include <casadi/casadi.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

using casadi::DM;
using casadi::Function;

std::vector<double> read_vector(const std::string& path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("Cannot open " + path);
    std::vector<double> values;
    std::string token;
    while (stream >> token) values.push_back(std::stod(token));
    return values;
}

casadi::Dict read_options(const std::string& path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("Cannot open solver options");
    casadi::Dict options;
    std::string key, type, value;
    while (stream >> key >> type >> value) {
        if (type == "bool") options[key] = value == "true";
        else if (type == "int") options[key] = std::stoi(value);
        else if (type == "double") options[key] = std::stod(value);
        else if (type == "string") options[key] = value;
        else throw std::runtime_error("Unknown solver option type " + type);
    }
    return options;
}

int main(int argc, char** argv) {
    try {
        if (argc != 3) throw std::runtime_error("Usage: window_native DIRECTORY verify|solve");
        const std::string directory = argv[1];
        const std::string mode = argv[2];
        if (mode == "verify") {
            Function original = Function::load(directory + "/checks.casadi");
            auto symbols = original.mx_in();
            casadi::Dict options{{"jit",true},{"compiler","shell"},
                {"jit_options",casadi::Dict{{"flags",std::vector<std::string>{"-O3"}}}}};
            Function compiled("native_checks", symbols, original(symbols), options);
            const std::vector<DM> inputs{DM(read_vector(directory + "/initial.txt")),
                DM(read_vector(directory + "/parameters.txt"))};
            const auto values = compiled(inputs);
            std::ostringstream report;
            report << "{\"passed\":true,\"relative_max_errors\":[";
            for (size_t index = 0; index < values.size(); ++index) {
                const auto expected = read_vector(directory + "/expected_" + std::to_string(index) + ".txt");
                const auto actual = values[index].nonzeros();
                if (expected.size() != actual.size()) throw std::runtime_error("Sparsity mismatch");
                double maximum = 0.;
                for (size_t k = 0; k < actual.size(); ++k) {
                    if (!std::isfinite(actual[k])) throw std::runtime_error("Nonfinite compiled evaluation");
                    maximum = std::max(maximum,std::abs(actual[k]-expected[k])/(1.+std::abs(expected[k])));
                }
                if (maximum > 1e-8) throw std::runtime_error("Compiled value or derivative mismatch");
                if (index) report << ',';
                report << std::setprecision(17) << maximum;
            }
            const auto start = std::chrono::steady_clock::now();
            for (int k = 0; k < 30; ++k) compiled(inputs);
            const double milliseconds = std::chrono::duration<double,std::milli>(
                std::chrono::steady_clock::now()-start).count()/30.;
            report << "],\"compiled_evaluation_ms\":" << milliseconds << "}\n";
            std::ofstream(directory + "/native_validation.json") << report.str();
            std::cout << "Native parity passed, combined evaluation " << milliseconds << " ms\n";
        } else if (mode == "solve") {
            auto options = read_options(directory + "/solver_options.txt");
            options["jit"] = true;
            options["compiler"] = "shell";
            options["jit_options"] = casadi::Dict{{"flags",std::vector<std::string>{"-O3"}}};
            Function solver = casadi::nlpsol("native_finite_problem","ipopt",
                Function::load(directory + "/nlp.casadi"),options);
            casadi::DMDict inputs{{"x0",DM(read_vector(directory+"/initial.txt"))},
                {"p",DM(read_vector(directory+"/parameters.txt"))},
                {"lbx",DM(read_vector(directory+"/lower_x.txt"))},
                {"ubx",DM(read_vector(directory+"/upper_x.txt"))},
                {"lbg",DM(read_vector(directory+"/lower_g.txt"))},
                {"ubg",DM(read_vector(directory+"/upper_g.txt"))}};
            const auto solution = solver(inputs);
            for (const auto& key : {"x","lam_x","lam_g"}) {
                std::ofstream output(directory+"/solution_"+key+".txt");
                for (double value : solution.at(key).nonzeros()) output << std::setprecision(17) << value << '\n';
            }
            const auto stats = solver.stats();
            std::ofstream report(directory+"/solver_status.txt");
            report << stats.at("return_status") << '\n' << stats.at("iter_count") << '\n';
        } else throw std::runtime_error("Unknown mode " + mode);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
