#include "numpy_passage.hpp"
#include <filesystem>
namespace fs=std::filesystem;
Json read(const fs::path& p){std::ifstream f(p);if(!f)throw std::runtime_error("Missing "+p.string());Json j;f>>j;return j;}
int main(int argc,char** argv) {
 try {
  if(argc<4)throw std::runtime_error("Usage: SCENE OUTPUT_JSON DIAGNOSTIC_RESULT...");
  const auto scene=read(argv[1]);Json rows=Json::array(),geometry=Json::array();
  for(int gate=0;gate+1<scene.at("windows").size();++gate) {
   const auto current=load_window(scene.at("windows").at(gate));
   const auto next=load_window(scene.at("windows").at(gate+1));
   geometry.push_back({{"gate",gate+1},{"next_gate",gate+2},{"next_nominal_center_normal_distance_m",std::abs(current.rotation.col(2).dot(next.center-current.center))}});
  }
  for(int arg=3;arg<argc;++arg) {
   const auto snapshot=read(argv[arg]);std::vector<double> durations=snapshot.at("durations");
   std::vector<double> crossings;double total=0.;for(int k=0;k<durations.size()-1;++k){total+=durations[k];crossings.push_back(total);}
   Json events=Json::array();
   for(int gate=0;gate<scene.at("windows").size();++gate) {
    const auto window=load_window(scene.at("windows").at(gate));double start=0.;
    for(int piece=0;piece<durations.size();++piece) {
     Curve curve;for(int k=0;k<8;++k)for(int axis=0;axis<3;++axis)curve(k,axis)=snapshot.at("coefficients").at(piece).at(k).at(axis);
     const auto local=numpy_local_curve(curve,window);
     for(double u:numpy_roots(local.col(2))) {
      if(!(u>1e-6 && u<1.-1e-6))continue;
      const double at=start+u*durations[piece];
      const Vector q=evaluate(local,u)-window.motion(at);
      if(!window.contains(q.head<2>()))continue;
      const double delta=std::min(1e-5,.1*std::min(u,1.-u));
      const double before=evaluate(local,u-delta).z(),after=evaluate(local,u+delta).z();
      double normal_rate=0.;for(int k=7;k>=1;--k)normal_rate=normal_rate*u+k*local(k,2);
      events.push_back({{"gate",gate+1},{"piece",piece},{"fraction",u},{"at_s",at},{"planned_crossing_s",crossings.at(gate)},
       {"after_planned_crossing_s",at-crossings.at(gate)},{"local_xy_m",{q.x(),q.y()}},
       {"frame_centerline_distance_m",std::sqrt(numpy_frame_distance(window,q))},
       {"relative_plane_normal_speed_mps",normal_rate/durations[piece]},
       {"plane_sign_change",before*after<0.},{"plane_values_before_after_m",{before,after}}});
     }
     start+=durations[piece];
    }
   }
   rows.push_back({{"source",fs::absolute(argv[arg]).string()},{"nominal_time_s",snapshot.at("nominal_time_s")},{"additional_interior_opening_plane_roots",events}});
  }
  std::ofstream(argv[2])<<Json{{"geometry",geometry},{"snapshots",rows},{"evidence","read-only polynomial roots of restored finite snapshots, not accepted plans"}}.dump(2)<<'\n';
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;
}
