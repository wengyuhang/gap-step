#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <sdf/Element.hh>

namespace convex_dynamic_course
{
/// Kinematically replay a reference until the contact monitor sends UDP.
/// The target model is dynamic throughout. Once a contact sensor reports a
/// collision this system stops writing pose and velocity commands, so the
/// physics engine, gravity, and collision impulse control the subsequent fall.
class TrajectoryReplay final :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  private: struct Sample
  {
    double time;
    gz::math::Vector3d position;
    gz::math::Vector3d velocity;
    gz::math::Quaterniond orientation;
    gz::math::Vector3d bodyAngularVelocity;
  };

  public: void Configure(const gz::sim::Entity &,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &,
      gz::sim::EventManager &) override
  {
    this->targetName = _sdf->Get<std::string>("target_model", "x500_0").first;
    this->motionStartTime = _sdf->Get<double>("motion_start_time", 30.0).first;
    const std::string environment = _sdf->Get<std::string>(
        "reference_environment_variable", "REPLAY_REFERENCE").first;
    const char *reference = std::getenv(environment.c_str());
    if (reference == nullptr || std::string(reference).empty())
      throw std::runtime_error("TrajectoryReplay needs environment variable " + environment);
    this->Load(reference);
    this->releasePort = _sdf->Get<int>("release_udp_port", 18680).first;
    this->releaseSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (this->releaseSocket < 0)
      throw std::runtime_error("TrajectoryReplay could not create release socket");
    const int flags = fcntl(this->releaseSocket, F_GETFL, 0);
    fcntl(this->releaseSocket, F_SETFL, flags | O_NONBLOCK);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(static_cast<uint16_t>(this->releasePort));
    if (bind(this->releaseSocket, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0)
      throw std::runtime_error("TrajectoryReplay could not bind release UDP port");
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->samples.empty())
      return;
    char message[16];
    if (!this->released && recv(this->releaseSocket, message, sizeof(message), 0) > 0)
      this->released = true;
    if (this->released)
      return;
    if (this->targetEntity == gz::sim::kNullEntity)
      this->FindTarget(_ecm);
    if (this->targetEntity == gz::sim::kNullEntity)
      return;

    const double simTime = std::chrono::duration<double>(_info.simTime).count();
    const double referenceTime = std::clamp(simTime - this->motionStartTime,
        0.0, this->samples.back().time);
    const Sample state = this->Interpolate(referenceTime);
    gz::sim::Model model(this->targetEntity);
    if (this->firstCommand)
    {
      gzmsg << "TrajectoryReplay commanding initial pose " << state.position << "\n";
      this->firstCommand = false;
    }
    const gz::sim::Entity linkEntity = model.CanonicalLink(_ecm);
    if (linkEntity == gz::sim::kNullEntity)
      return;
    gz::sim::Link link(linkEntity);
    // The PX4 x500 root model is an SDF wrapper; its canonical base link is
    // the physical body and the quantity reported by PX4 local position.
    // Command that link directly instead of the non-physical wrapper pose.
    const gz::math::Pose3d pose(state.position, state.orientation);
    auto *poseCommand = _ecm.Component<gz::sim::components::WorldPoseCmd>(linkEntity);
    if (poseCommand == nullptr)
      _ecm.CreateComponent(linkEntity, gz::sim::components::WorldPoseCmd(pose));
    else
      poseCommand->SetData(pose, [](const auto &a, const auto &b) { return a == b; });
    // Gazebo's Link setters expect values in the link frame. Velocity is
    // supplied in ENU/world coordinates by the trajectory CSV.
    link.SetLinearVelocity(_ecm, state.orientation.Inverse().RotateVector(state.velocity));
    link.SetAngularVelocity(_ecm, state.bodyAngularVelocity);
  }

  private: void FindTarget(const gz::sim::EntityComponentManager &_ecm)
  {
    _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
        [this](const gz::sim::Entity &_entity,
            const gz::sim::components::Model *,
            const gz::sim::components::Name *_name) {
          if (_name->Data() == this->targetName)
          {
            this->targetEntity = _entity;
            gzmsg << "TrajectoryReplay acquired model " << this->targetName << "\n";
            return false;
          }
          return true;
        });
  }

  private: void Load(const std::string &_path)
  {
    std::ifstream stream(_path);
    if (!stream)
      throw std::runtime_error("cannot open replay reference: " + _path);
    std::string line;
    std::getline(stream, line);  // Header.
    while (std::getline(stream, line))
    {
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream row(line);
      Sample sample{};
      double qw, qx, qy, qz;
      if (!(row >> sample.time >> sample.position.X() >> sample.position.Y() >> sample.position.Z()
          >> sample.velocity.X() >> sample.velocity.Y() >> sample.velocity.Z()
          >> qw >> qx >> qy >> qz
          >> sample.bodyAngularVelocity.X() >> sample.bodyAngularVelocity.Y()
          >> sample.bodyAngularVelocity.Z()))
        throw std::runtime_error("invalid replay row in " + _path);
      sample.orientation = gz::math::Quaterniond(qw, qx, qy, qz);
      sample.orientation.Normalize();
      this->samples.push_back(sample);
    }
    if (this->samples.size() < 2)
      throw std::runtime_error("replay reference needs at least two samples");
  }

  private: Sample Interpolate(double _time) const
  {
    const auto upper = std::upper_bound(this->samples.begin(), this->samples.end(), _time,
        [](double value, const Sample &sample) { return value < sample.time; });
    const std::size_t high = std::clamp<std::size_t>(
        static_cast<std::size_t>(upper - this->samples.begin()), 1, this->samples.size() - 1);
    const Sample &a = this->samples[high - 1];
    const Sample &b = this->samples[high];
    const double alpha = std::clamp((_time - a.time) / (b.time - a.time), 0.0, 1.0);
    Sample output{};
    output.time = _time;
    output.position = a.position + alpha * (b.position - a.position);
    output.velocity = a.velocity + alpha * (b.velocity - a.velocity);
    output.orientation = gz::math::Quaterniond::Slerp(alpha, a.orientation, b.orientation);
    output.bodyAngularVelocity = a.bodyAngularVelocity + alpha *
        (b.bodyAngularVelocity - a.bodyAngularVelocity);
    return output;
  }

  private: std::string targetName;
  private: std::vector<Sample> samples;
  private: gz::sim::Entity targetEntity{gz::sim::kNullEntity};
  private: double motionStartTime{30.0};
  private: bool released{false};
  private: int releaseSocket{-1};
  private: int releasePort{18680};
  private: bool firstCommand{true};
};
}

GZ_ADD_PLUGIN(convex_dynamic_course::TrajectoryReplay,
              gz::sim::System,
              convex_dynamic_course::TrajectoryReplay::ISystemConfigure,
              convex_dynamic_course::TrajectoryReplay::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(convex_dynamic_course::TrajectoryReplay,
                    "convex_dynamic_course::TrajectoryReplay")
