#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>
namespace paper_experiment {
class MixedGateMotion final:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate {
 public:void Configure(const gz::sim::Entity&e,const std::shared_ptr<const sdf::Element>&s,gz::sim::EntityComponentManager&,gz::sim::EventManager&) override {
 model=gz::sim::Model(e);center=s->Get<gz::math::Vector3d>("center0");rpy=s->Get<gz::math::Vector3d>("base_rpy");a=s->Get<gz::math::Vector3d>("axis0");b=s->Get<gz::math::Vector3d>("axis1");frequency=s->Get<double>("frequency");amplitude=s->Get<double>("amplitude");length=s->Get<double>("length");epoch=s->Get<double>("epoch");kind=s->Get<std::string>("kind");}
 public:void PreUpdate(const gz::sim::UpdateInfo&i,gz::sim::EntityComponentManager&ecm) override {
 if(!model.Valid(ecm))return;double t=std::max(0.,std::chrono::duration<double>(i.simTime).count()-epoch);double s=std::sin(2*M_PI*frequency*t);gz::math::Vector3d p=center;
 if(kind=="translation")p+=amplitude*s*a;else {double th=std::asin(amplitude/length)*s;p+=length*std::sin(th)*a+length*(1-std::cos(th))*b;}
 model.SetWorldPoseCmd(ecm,gz::math::Pose3d(p,gz::math::Quaterniond(rpy)));}
 private:gz::sim::Model model{gz::sim::kNullEntity};gz::math::Vector3d center,rpy,a,b;double frequency,amplitude,length,epoch;std::string kind;
};}
GZ_ADD_PLUGIN(paper_experiment::MixedGateMotion,gz::sim::System,paper_experiment::MixedGateMotion::ISystemConfigure,paper_experiment::MixedGateMotion::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(paper_experiment::MixedGateMotion,"paper_experiment::MixedGateMotion")
