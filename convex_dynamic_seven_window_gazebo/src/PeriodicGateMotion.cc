#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

namespace convex_dynamic_course
{
class PeriodicGateMotion final :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &,
      gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    this->center = ReadVector(_sdf, "center0");
    this->baseRpy = ReadVector(_sdf, "base_rpy");
    this->translationAmplitude = ReadVector(_sdf, "translation_amplitude");
    this->rotationAmplitude = ReadVector(_sdf, "rotation_amplitude");
    this->translationPeriod = _sdf->Get<double>("translation_period");
    this->rotationPeriod = _sdf->Get<double>("rotation_period");
    this->phase = _sdf->Get<double>("phase");
    this->motionStartTime = _sdf->Get<double>("motion_start_time", 0.0).first;
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->model.Valid(_ecm))
      return;
    const double simulationTime = std::chrono::duration<double>(_info.simTime).count();
    const double t = std::max(0.0, simulationTime - this->motionStartTime);
    constexpr double twoPi = 6.283185307179586476925286766559;
    gz::math::Vector3d position = this->center;
    gz::math::Vector3d rpy = this->baseRpy;
    const std::array<double, 3> translationPhase{0.0, 0.7, 1.4};
    const std::array<double, 3> rotationPhase{0.0, 0.9, 1.8};
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
      position[axis] += this->translationAmplitude[axis] *
          std::sin(twoPi * t / this->translationPeriod + this->phase +
                   translationPhase[axis]);
      rpy[axis] += this->rotationAmplitude[axis] *
          std::sin(twoPi * t / this->rotationPeriod + this->phase +
                   rotationPhase[axis]);
    }
    this->model.SetWorldPoseCmd(_ecm, gz::math::Pose3d(position,
        gz::math::Quaterniond(rpy)));
  }

  private: static gz::math::Vector3d ReadVector(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name)
  {
    return _sdf->Get<gz::math::Vector3d>(_name);
  }

  private: gz::sim::Model model{gz::sim::kNullEntity};
  private: gz::math::Vector3d center{0, 0, 0};
  private: gz::math::Vector3d baseRpy{0, 0, 0};
  private: gz::math::Vector3d translationAmplitude{0, 0, 0};
  private: gz::math::Vector3d rotationAmplitude{0, 0, 0};
  private: double translationPeriod{1.0};
  private: double rotationPeriod{1.0};
  private: double phase{0.0};
  private: double motionStartTime{0.0};
};
}

GZ_ADD_PLUGIN(convex_dynamic_course::PeriodicGateMotion,
              gz::sim::System,
              convex_dynamic_course::PeriodicGateMotion::ISystemConfigure,
              convex_dynamic_course::PeriodicGateMotion::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(convex_dynamic_course::PeriodicGateMotion,
                    "convex_dynamic_course::PeriodicGateMotion")
