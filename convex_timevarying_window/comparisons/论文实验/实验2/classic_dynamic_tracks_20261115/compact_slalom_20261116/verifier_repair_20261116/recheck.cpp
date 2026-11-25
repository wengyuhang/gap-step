#include "numpy_passage.hpp"
#include <chrono>
#include <filesystem>
namespace fs=std::filesystem;
Json read(const fs::path& p) {std::ifstream f(p);if(!f)throw std::runtime_error("Missing "+p.string());Json j;f>>j;return j;}
int main(int argc,char** argv) {
 try {
  if(argc!=3)throw std::runtime_error("Usage: SCENE_RUN_DIRECTORY OUTPUT_JSON");
  fs::path root=argv[1];Json output=Json::array();
  for(const std::string method:{"ours","center"})for(int seed=0;seed<8;++seed) {
   const std::string name=method+"_"+std::to_string(seed);
   const auto input=read(root/"paired_v1/compact_slalom/prepared"/name/"manifest.json");
   const auto curve=read(root/"finite_diagnostics"/name/"result.json");
   const double radius=input.at("verification_radius");
   for(int budget:{16384}) {
    auto began=std::chrono::steady_clock::now();Json failed=Json::array();int total=0,maximum=0;
    for(int gate=0;gate<input.at("windows").size();++gate) {
     Window w=load_window(input.at("windows").at(gate));double start=0.;
     for(int piece=0;piece<curve.at("durations").size();++piece) {
      Curve c;for(int k=0;k<8;++k)for(int axis=0;axis<3;++axis)c(k,axis)=curve.at("coefficients").at(piece).at(k).at(axis);
      double duration=curve.at("durations").at(piece);
      auto check=numpy_passage(c,duration,start,w,radius,false,budget);start+=duration;
      const int boxes=check.at("boxes");total+=boxes;maximum=std::max(maximum,boxes);
      if(!check.at("passed").get<bool>()){check["gate"]=gate;check["piece"]=piece;failed.push_back(check);}
     }
    }
    const double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
    output.push_back({{"candidate",name},{"budget",budget},{"all_passed",failed.empty()},{"wall_s",wall},{"total_subdivisions",total},{"maximum_subdivisions",maximum},{"failed_checks",failed}});
   }
  }
  std::ofstream(argv[2])<<Json{{"scope","same stored finite trajectories; changed verification work budget only; no new optimization"},{"results",output}}.dump(2)<<'\n';
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;
}
