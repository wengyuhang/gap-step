#include "json.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
namespace fs = std::filesystem;
using Json = nlohmann::json;
Json load(const fs::path &file) {
    std::ifstream stream(file);
    if (!stream) throw std::runtime_error("Missing " + file.string());
    Json value;
    stream >> value;
    return value;
}
int main(int argc, char **argv) {
    try {
        if (argc != 4) throw std::runtime_error("Usage: ORIGINAL_INPUTS PROTOCOL_JSON NEW_REPORT_DIRECTORY");
        const fs::path input = argv[1], output = argv[3];
        fs::create_directories(output);
        const auto protocol = load(argv[2]);
        Json all = Json::array();
        std::ofstream report(output / "MOTION_TABLE.md");
        report << "# 固定赛道逐窗运动\n\n所有数值直接从原目标scene.json读取。下表中的同一扇门在整次求解中使用同一函数。静止门也列出。\n\n";
        report << "门局部坐标的第三轴为平面法向。平移基函数为m(t)=(0,-A sin(2πft+φ),0)。摆动基函数为θ(t)=asin(A/L)sin(2πft+φ)，m(t)=(L(1-cosθ),-Lsinθ,0)。二者再绕局部第三轴转motion_angle_rad，最后乘门的固定世界旋转矩阵。故方向/外支点、A、f、φ共同确定各窗独立运动。\n\n";
        for (const auto &levelValue : protocol.at("levels")) {
            const std::string level = levelValue;
            const auto scene = load(input / "assets" / level / "scene.json");
            report << "## " << level << "\n\n|窗|形状|运动|方向/支点角度°|幅度m|频率Hz|相位rad|摆长m|\n|---|---|---|---:|---:|---:|---:|---:|\n";
            std::set<std::string> signatures;
            Json gates = Json::array();
            int moving = 0;
            for (size_t index = 0; index < scene.at("windows").size(); ++index) {
                const auto &window = scene.at("windows").at(index);
                const double amplitude = window.at("amplitude"), frequency = window.at("frequency"), phase = window.at("phase"),
                    angle = window.value("motion_angle_rad", 0.), length = window.at("length");
                const bool active = amplitude != 0. && frequency != 0.;
                const std::string kind = active ? window.at("swing").get<bool>() ? "摆动" : "平移" : "静止";
                const Json signature{{"swing",window.at("swing")},{"amplitude",amplitude},{"frequency",frequency},{"phase",phase},{"angle",angle},{"length",length}};
                if (active) { ++moving; signatures.insert(signature.dump()); }
                gates.push_back({{"gate",index+1},{"shape",window.at("circle").get<bool>()?"circle":"square"},{"motion",kind},
                    {"amplitude_m",amplitude},{"frequency_hz",frequency},{"phase_rad",phase},{"motion_angle_rad",angle},{"length_m",length},
                    {"center",window.at("center")},{"rotation",window.at("rotation")}});
                report << index+1 << '|' << (window.at("circle").get<bool>()?"圆":"方") << '|' << kind << '|' << std::fixed << std::setprecision(3)
                       << angle*180./3.141592653589793 << '|' << amplitude << '|' << frequency << '|' << phase << '|' << length << "|\n";
            }
            const bool independent = signatures.size() == static_cast<size_t>(moving);
            if (!independent) throw std::runtime_error("Duplicate full moving-window law in " + level);
            all.push_back({{"level",level},{"gate_count",gates.size()},{"moving_gate_count",moving},{"unique_moving_motion_laws",signatures.size()},{"all_moving_motion_signatures_distinct",independent},{"gates",gates}});
            report << "\n移动窗" << moving << "扇，完整运动参数组合各不相同。\n\n";
        }
        std::ofstream(output / "MOTION_AUDIT.json") << all.dump(2) << '\n';
    } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
    return 0;
}
