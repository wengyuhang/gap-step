#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <string>
#include <sstream>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double_v.pb.h>
#include <sdf/Element.hh>
namespace togt_aligned {
std::ofstream log_file(const std::shared_ptr<const sdf::Element>&s,const std::string&header){std::ofstream f;if(const char*dir=std::getenv("TOGT_GAZEBO_LOG_DIR")){f.open(std::string(dir)+"/"+s->Get<std::string>("record_file"));f.precision(17);f<<header<<'\n';}return f;}
class GateMotion final:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate{
 gz::sim::Model model{gz::sim::kNullEntity};gz::math::Vector3d center;gz::math::Quaterniond rotation;double frequency,amplitude,length,phase,last=-1.;bool swing;std::ofstream log;
 public:void Configure(const gz::sim::Entity&e,const std::shared_ptr<const sdf::Element>&s,gz::sim::EntityComponentManager&,gz::sim::EventManager&)override{
 model=gz::sim::Model(e);center=s->Get<gz::math::Vector3d>("center0");rotation=gz::math::Quaterniond(s->Get<gz::math::Vector3d>("rpy"));frequency=s->Get<double>("frequency");amplitude=s->Get<double>("amplitude");length=s->Get<double>("length");phase=s->Get<double>("phase");swing=s->Get<bool>("swing");log=log_file(s,"time,x,y,z");}
 public:void PreUpdate(const gz::sim::UpdateInfo&i,gz::sim::EntityComponentManager&ecm)override{if(!model.Valid(ecm)||i.paused)return;double t=std::chrono::duration<double>(i.simTime).count(),a=sin(6.283185307179586*frequency*t+phase);gz::math::Vector3d offset(0.,-amplitude*a,0.);if(swing){double theta=asin(amplitude/length)*a;offset.Set(length*(1.-cos(theta)),-length*sin(theta),0.);}const auto p=center+rotation.RotateVector(offset);model.SetWorldPoseCmd(ecm,gz::math::Pose3d(p,rotation));if(log&&t-last>=.01){log<<t<<','<<p.X()<<','<<p.Y()<<','<<p.Z()<<'\n';last=t;}}
};
class QuadAForces final:public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate{
 gz::sim::Model model{gz::sim::kNullEntity};gz::sim::Link body{gz::sim::kNullEntity};gz::transport::Node node;std::array<double,4>forces{};std::array<std::array<double,4>,4>mixer;double maximum,last=-1.;std::mutex mutex;std::ofstream log;
 public:void Configure(const gz::sim::Entity&e,const std::shared_ptr<const sdf::Element>&s,gz::sim::EntityComponentManager&ecm,gz::sim::EventManager&)override{
 model=gz::sim::Model(e);body=gz::sim::Link(model.LinkByName(ecm,"body"));maximum=s->Get<double>("rotor_max");const double hover=s->Get<double>("mass")*s->Get<double>("gravity")/4.;forces.fill(hover);
 for(int r=0;r<4;++r){std::istringstream v(s->Get<std::string>("mixer"+std::to_string(r)));for(int c=0;c<4;++c)v>>mixer[r][c];}
 node.Subscribe(s->Get<std::string>("command_topic"),&QuadAForces::Command,this);log=log_file(s,"time,x,y,z,f0,f1,f2,f3");}
 void Command(const gz::msgs::Double_V&msg){if(msg.data_size()!=4)return;std::lock_guard<std::mutex>lock(mutex);for(int k=0;k<4;++k){if(!std::isfinite(msg.data(k)))return;}for(int k=0;k<4;++k)forces[k]=std::clamp(msg.data(k),0.,maximum);}
 public:void PreUpdate(const gz::sim::UpdateInfo&i,gz::sim::EntityComponentManager&ecm)override{if(i.paused||!body.Valid(ecm))return;const auto pose=body.WorldPose(ecm);if(!pose)return;std::array<double,4>f,w{};{std::lock_guard<std::mutex>lock(mutex);f=forces;}for(int r=0;r<4;++r)for(int k=0;k<4;++k)w[r]+=mixer[r][k]*f[k];body.AddWorldWrench(ecm,pose->Rot().RotateVector({0.,0.,w[0]}),pose->Rot().RotateVector({w[1],w[2],w[3]}));double t=std::chrono::duration<double>(i.simTime).count();if(log&&t-last>=.01){log<<t<<','<<pose->Pos().X()<<','<<pose->Pos().Y()<<','<<pose->Pos().Z();for(auto v:f)log<<','<<v;log<<'\n';last=t;}}
};
}
GZ_ADD_PLUGIN(togt_aligned::GateMotion,gz::sim::System,togt_aligned::GateMotion::ISystemConfigure,togt_aligned::GateMotion::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(togt_aligned::GateMotion,"togt_aligned::GateMotion")
GZ_ADD_PLUGIN(togt_aligned::QuadAForces,gz::sim::System,togt_aligned::QuadAForces::ISystemConfigure,togt_aligned::QuadAForces::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(togt_aligned::QuadAForces,"togt_aligned::QuadAForces")
