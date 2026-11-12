#include "minimum_thrust.hpp"

int main(int argc, char** argv) {
    try {
        if (argc != 3) throw std::runtime_error("Usage: minimum_thrust_check TRAJECTORY_JSON OUTPUT_JSON");
        Json input; std::ifstream(argv[1]) >> input;
        Json checks=Json::array(); double minimum=INFINITY; bool passed=true;
        constexpr double mass=2.0643076923076924, gravity=9.8066;
        for (size_t piece=0; piece<input.at("durations").size(); ++piece) {
            Curve curve;
            for(int power=0;power<8;++power)for(int axis=0;axis<3;++axis)
                curve(power,axis)=input["coefficients"][piece][power][axis];
            auto check=minimum_thrust::check(curve,input["durations"][piece],mass,gravity);
            check["piece"]=piece; minimum=std::min(minimum,check["minimum_total_thrust_n"].get<double>());
            passed=passed && check["passed"].get<bool>(); checks.push_back(check);
        }
        std::ofstream(argv[2]) << Json{{"passed",passed},{"minimum_total_thrust_n",minimum},
            {"required_total_thrust_n",minimum_thrust::collective_floor},{"checks",checks}}.dump(2) << '\n';
    } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
    return 0;
}
