// Visual playback only: static model poses, never a flight / collision test.
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <sdf/Element.hh>

class RecordedPoseReplay final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
  struct Sample { double t; gz::math::Vector3d p; gz::math::Quaterniond q; };
  std::vector<Sample> samples;
  gz::sim::Entity entity{gz::sim::kNullEntity};
  double start{30.0};
 public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    start=sdf->Get<double>("motion_start_time",30.).first;
    const char *path=std::getenv("REPLAY_REFERENCE");
    if (!path) throw std::runtime_error("Missing REPLAY_REFERENCE");
    std::ifstream input(path); std::string line; std::getline(input,line);
    while(std::getline(input,line)) {
      std::replace(line.begin(),line.end(),',',' '); std::istringstream row(line);
      Sample s; double vx,vy,vz,w,x,y,z;
      if (!(row>>s.t>>s.p.X()>>s.p.Y()>>s.p.Z()>>vx>>vy>>vz>>w>>x>>y>>z))
        throw std::runtime_error("Bad recorded pose row");
      s.q=gz::math::Quaterniond(w,x,y,z); s.q.Normalize(); samples.push_back(s);
    }
    if(samples.size()<2) throw std::runtime_error("Missing recorded poses");
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if(entity==gz::sim::kNullEntity)
      ecm.Each<gz::sim::components::Model,gz::sim::components::Name>(
        [&](const gz::sim::Entity &id,const auto *,const auto *name) {
          if(name->Data()=="x500_0") {entity=id; return false;} return true;
        });
    if(entity==gz::sim::kNullEntity) return;
    const double t=std::clamp(std::chrono::duration<double>(info.simTime).count()-start,
                              samples.front().t,samples.back().t);
    auto it=std::upper_bound(samples.begin(),samples.end(),t,
                            [](double t,const Sample &s){return t<s.t;});
    auto i=std::clamp<std::size_t>(it-samples.begin(),1,samples.size()-1);
    const auto &a=samples[i-1];const auto &b=samples[i];
    double f=(t-a.t)/(b.t-a.t);
    auto q=gz::math::Quaterniond::Slerp(f,a.q,b.q);
    auto p=a.p+f*(b.p-a.p);
    // CSV describes base_link; model root is 0.24 m below it in body coordinates.
    p-=q.RotateVector(gz::math::Vector3d(0,0,.24));
    gz::sim::Model(entity).SetWorldPoseCmd(ecm,gz::math::Pose3d(p,q));
  }
};
GZ_ADD_PLUGIN(RecordedPoseReplay,gz::sim::System,
              RecordedPoseReplay::ISystemConfigure,RecordedPoseReplay::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(RecordedPoseReplay,"RecordedPoseReplay")
