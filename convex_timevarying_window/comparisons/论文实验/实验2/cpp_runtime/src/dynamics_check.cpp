#include "dynamics_exchange.hpp"
#include <fstream>

int main(int argc,char** argv) {
    try {
        if(argc!=4)throw std::runtime_error("Usage: dynamics_check TRAJECTORY_JSON METADATA_JSON OUTPUT_JSON");
        Json input,metadata;std::ifstream(argv[1])>>input;std::ifstream(argv[2])>>metadata;
        const dynamics_exchange::Model model(metadata);
        Json checks=Json::array();bool passed=true;double elapsed=0.;
        for(size_t piece=0;piece<input.at("durations").size();++piece) {
            Curve curve;for(int k=0;k<8;++k)for(int axis=0;axis<3;++axis)curve(k,axis)=input["coefficients"][piece][k][axis];
            auto check=dynamics_exchange::check_segment(model,curve,input["durations"][piece],metadata["verification_residual_tolerance"]);
            check["piece"]=piece;
            if(input.contains("probe_fractions")) {
                dynamics_exchange::Segment segment(model,curve,input["durations"][piece]);
                Json values=Json::array();for(const auto& u:input["probe_fractions"])values.push_back(segment.at(u));
                check["probe_residuals"]=values;
            }
            passed=passed&&check["passed"].get<bool>();elapsed+=check["check_s"].get<double>();checks.push_back(check);
        }
        std::ofstream(argv[3])<<Json{{"passed",passed},{"check_s",elapsed},{"checks",checks}}.dump(2)<<'\n';
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
    return 0;
}
