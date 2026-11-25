#include "numpy_passage.hpp"

int main(int argc,char** argv) {
    try {
        if(argc!=3) throw std::runtime_error("Usage: frame_check INPUT_JSON OUTPUT_JSON");
        Json input;std::ifstream(argv[1])>>input;
        Json checks=Json::array();bool passed=true;
        for(size_t gate=0;gate<input.at("windows").size();++gate) {
            Window window=load_window(input["windows"][gate]);double start=0.;
            for(size_t piece=0;piece<input.at("durations").size();++piece) {
                Curve coefficients;for(int i=0;i<8;++i) for(int j=0;j<3;++j) coefficients(i,j)=input["coefficients"][piece][i][j];
                const double duration=input["durations"][piece];
                Json result=numpy_passage(coefficients,duration,start,window,input.at("radius"));
                result["gate"]=gate;result["piece"]=piece;passed=passed&&result["passed"].get<bool>();checks.push_back(result);
                start+=duration;
            }
        }
        std::ofstream(argv[2])<<Json{{"passed",passed},{"checks",checks}}.dump(2)<<'\n';
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
