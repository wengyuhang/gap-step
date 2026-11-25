#include "geometry.hpp"
#include <filesystem>
namespace fs=std::filesystem;
Json read(const fs::path& path){std::ifstream f(path);if(!f)throw std::runtime_error("Missing "+path.string());Json j;f>>j;return j;}
int main(int argc,char** argv) {
 try {
  if(argc<4)throw std::runtime_error("Usage: SCENE OUTPUT_JSON TRAJECTORY_JSON...");
  const auto scene=read(argv[1]);const double radius=scene.at("planning_safety_sphere_radius_m");
  Json results=Json::array();
  for(int arg=3;arg<argc;++arg) {
   const auto trajectory=read(argv[arg]);Json violations=Json::array();int sampleCount=0;
   for(int gate=0;gate<scene.at("windows").size();++gate) {
    const auto window=load_window(scene.at("windows").at(gate));double start=0.,minimum=INFINITY;Json worst;
    for(int piece=0;piece<trajectory.at("durations").size();++piece) {
     Curve world;for(int p=0;p<8;++p)for(int axis=0;axis<3;++axis)world(p,axis)=trajectory.at("coefficients").at(piece).at(p).at(axis);
     const Curve local=world*window.rotation;
     const double duration=trajectory.at("durations").at(piece);const int steps=std::max(1,int(std::ceil(duration/.001)));
     for(int sample=0;sample<=steps;++sample) {
      ++sampleCount;const double u=double(sample)/steps,t=start+u*duration;
      const Vector q=evaluate(local,u)-window.rotation.transpose()*window.center-window.motion(t);
      const double s=-scene.at("windows").at(gate).at("sign").get<double>()*q.z();
      if(std::abs(s)>=radius || !window.contains(q.head<2>()))continue;
      const double desired_side=piece<=gate?-1.:1.;const double margin=desired_side*s;
      if(margin<minimum){minimum=margin;worst={{"gate",gate+1},{"piece",piece},{"at_s",t},{"u",u},{"signed_plane_distance_m",s},{"required_side",desired_side},{"local_xy_m",{q.x(),q.y()}},{"margin_m",margin}};}
     }
     start+=duration;
    }
    if(minimum< -1e-8)violations.push_back(worst);
   }
   results.push_back({{"source",fs::absolute(argv[arg]).string()},{"original_found",trajectory.value("found",false)},
    {"passed_sampled_proposal",violations.empty()},{"samples",sampleCount},{"violating_gates",violations}});
  }
  std::ofstream(argv[2])<<Json{{"scope","read-only evaluation of proposed local side rule; no optimization"},{"max_step_s",.001},
   {"region","physical aperture projection and absolute signed plane distance below planning safety sphere radius"},
   {"rule","entry side through incoming piece; exit side from outgoing piece"},{"results",results}}.dump(2)<<'\n';
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;
}
