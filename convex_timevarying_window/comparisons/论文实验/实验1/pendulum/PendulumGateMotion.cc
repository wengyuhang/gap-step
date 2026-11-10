#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>
namespace paper_experiment {
class PendulumGateMotion final: public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 public: void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,gz::sim::EntityComponentManager &,gz::sim::EventManager &) override {
  model=gz::sim::Model(entity);pivot=sdf->Get<gz::math::Vector3d>("pivot");epoch=sdf->Get<double>("epoch");length=sdf->Get<double>("length");
  std::ifstream f(sdf->Get<std::string>("table"));std::array<double,5> r;
  while(f>>r[0]>>r[1]>>r[2]>>r[3]>>r[4]) rows.push_back(r);
  if(rows.empty())throw std::runtime_error("pendulum motion table missing");
 }
 public: void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
  if(!model.Valid(ecm))return;
  double t=std::max(0.,std::chrono::duration<double>(info.simTime).count()-epoch);
  auto it=std::upper_bound(rows.begin(),rows.end(),t,[](double v,const std::array<double,5>&r){return v<r[0];});
  auto r=*(it==rows.begin()?it:it-1);double h=t-r[0];double theta=((r[1]*h+r[2])*h+r[3])*h+r[4];
  gz::math::Vector3d p=pivot+gz::math::Vector3d(0.,length*std::sin(theta),-length*std::cos(theta));
  // R = Rx(theta) * R0; R0's columns are world Y, Z, X.
  gz::math::Quaterniond base(0.5,0.5,0.5,0.5),roll(gz::math::Vector3d(theta,0.,0.));
  model.SetWorldPoseCmd(ecm,gz::math::Pose3d(p,roll*base));
 }
 private: gz::sim::Model model{gz::sim::kNullEntity};gz::math::Vector3d pivot;double epoch=30.,length=2.;std::vector<std::array<double,5>> rows;
};}
GZ_ADD_PLUGIN(paper_experiment::PendulumGateMotion,gz::sim::System,paper_experiment::PendulumGateMotion::ISystemConfigure,paper_experiment::PendulumGateMotion::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(paper_experiment::PendulumGateMotion,"paper_experiment::PendulumGateMotion")
