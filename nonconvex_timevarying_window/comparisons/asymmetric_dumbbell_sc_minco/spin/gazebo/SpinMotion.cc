#include <algorithm>
#include <chrono>
#include <cmath>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

// Scene adapter only: no drone pose, control or planning changes.
class NonconvexSpinMotion final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
 public: void Configure(const gz::sim::Entity &entity,
     const std::shared_ptr<const sdf::Element> &sdf,
     gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
 {
   model = gz::sim::Model(entity);
   center = sdf->Get<gz::math::Vector3d>("center0");
   amplitude = sdf->Get<gz::math::Vector3d>("translation_amplitude");
   base = gz::math::Quaterniond(sdf->Get<gz::math::Vector3d>("base_rpy"));
   frequency = sdf->Get<double>("frequency");
   spin = sdf->Get<double>("spin_hz");
   phase = sdf->Get<double>("phase");
   spinPhase = sdf->Get<double>("spin_phase");
   epoch = sdf->Get<double>("motion_start_time");
 }
 public: void PreUpdate(const gz::sim::UpdateInfo &info,
     gz::sim::EntityComponentManager &ecm) override
 {
   if (!model.Valid(ecm)) return;
   const double t = std::max(0., std::chrono::duration<double>(info.simTime).count()-epoch);
   const double tau = 6.283185307179586476925286766559;
   const auto p = center + amplitude*std::sin(tau*frequency*t+phase);
   const auto q = base*gz::math::Quaterniond(0., 0., tau*spin*t+spinPhase);
   model.SetWorldPoseCmd(ecm, gz::math::Pose3d(p, q));
 }
 private:
   gz::sim::Model model{gz::sim::kNullEntity};
   gz::math::Vector3d center, amplitude;
   gz::math::Quaterniond base;
   double frequency=0., spin=0., phase=0., spinPhase=0., epoch=30.;
};
GZ_ADD_PLUGIN(NonconvexSpinMotion, gz::sim::System,
             gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
