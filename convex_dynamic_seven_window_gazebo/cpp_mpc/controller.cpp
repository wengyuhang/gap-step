#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <casadi/casadi.hpp>
#include <common/mavlink.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr int kNx = 13;
constexpr int kNu = 4;
constexpr int kN = 20;
constexpr double kNodeDt = 0.05;
constexpr double kMotionStart = 30.0;
constexpr double kMass = 2.0643076923076924;
constexpr double kGravity = 9.8066;
constexpr double kMotorConstant = 8.54858e-6;
constexpr double kMotorMin = 150.0;
constexpr double kMotorMax = 1000.0;

struct Reference {
  double time{};
  std::array<double, kNx> state{};
  std::array<double, kNu> input{};
};

struct Vehicle {
  bool heartbeat{}, armed{}, local{}, attitude{};
  uint32_t boot_ms{};
  double x{}, y{}, z{}, vx{}, vy{}, vz{};
  double roll{}, pitch{}, yaw{}, p{}, q{}, r{};
};

std::vector<Reference> load_reference(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open reference: " + path);
  std::vector<Reference> rows;
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty()) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream values(line);
    Reference row;
    if (!(values >> row.time)) throw std::runtime_error("invalid reference row");
    for (double& value : row.state) values >> value;
    for (double& value : row.input) values >> value;
    rows.push_back(row);
  }
  if (rows.size() < 2) throw std::runtime_error("reference has fewer than two rows");
  return rows;
}

std::pair<size_t, double> bracket(const std::vector<Reference>& rows, double time) {
  const auto it = std::lower_bound(rows.begin(), rows.end(), time,
      [](const Reference& row, double value) { return row.time < value; });
  if (it == rows.begin()) return {0, 0.0};
  if (it == rows.end()) return {rows.size() - 2, 1.0};
  const size_t lower = static_cast<size_t>(it - rows.begin() - 1);
  return {lower, (time - rows[lower].time) / (rows[lower + 1].time - rows[lower].time)};
}

std::array<double, kNx> state_at(const std::vector<Reference>& rows, double time) {
  const auto [index, alpha] = bracket(rows, time);
  std::array<double, kNx> result{};
  for (int i = 0; i < kNx; ++i)
    result[i] = rows[index].state[i] + alpha * (rows[index + 1].state[i] - rows[index].state[i]);
  return result;
}

std::array<double, kNu> input_at(const std::vector<Reference>& rows, double time) {
  const auto [index, alpha] = bracket(rows, time);
  std::array<double, kNu> result{};
  for (int i = 0; i < kNu; ++i)
    result[i] = rows[index].input[i] + alpha * (rows[index + 1].input[i] - rows[index].input[i]);
  return result;
}

class MavlinkUdp {
 public:
  MavlinkUdp() {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) throw std::runtime_error("UDP socket creation failed");
    int reuse = 1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(14550);
    local.sin_addr.s_addr = INADDR_ANY;
    if (bind(fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0)
      throw std::runtime_error("cannot bind UDP port 14550");
    fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);
  }

  ~MavlinkUdp() { if (fd_ >= 0) close(fd_); }

  void receive(Vehicle& vehicle) {
    uint8_t buffer[4096];
    socklen_t peer_size = sizeof(peer_);
    ssize_t count;
    while ((count = recvfrom(fd_, buffer, sizeof(buffer), 0,
                             reinterpret_cast<sockaddr*>(&peer_), &peer_size)) > 0) {
      peer_known_ = true;
      for (ssize_t i = 0; i < count; ++i) {
        mavlink_message_t message;
        if (!mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &parser_status_)) continue;
        if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          mavlink_heartbeat_t value;
          mavlink_msg_heartbeat_decode(&message, &value);
          vehicle.heartbeat = true;
          vehicle.armed = value.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
        } else if (message.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
          mavlink_local_position_ned_t value;
          mavlink_msg_local_position_ned_decode(&message, &value);
          vehicle.local = true;
          vehicle.boot_ms = value.time_boot_ms;
          vehicle.x = value.x; vehicle.y = value.y; vehicle.z = value.z;
          vehicle.vx = value.vx; vehicle.vy = value.vy; vehicle.vz = value.vz;
        } else if (message.msgid == MAVLINK_MSG_ID_ATTITUDE) {
          mavlink_attitude_t value;
          mavlink_msg_attitude_decode(&message, &value);
          vehicle.attitude = true;
          vehicle.boot_ms = std::max(vehicle.boot_ms, value.time_boot_ms);
          vehicle.roll = value.roll; vehicle.pitch = value.pitch; vehicle.yaw = value.yaw;
          vehicle.p = value.rollspeed; vehicle.q = value.pitchspeed; vehicle.r = value.yawspeed;
        }
      }
    }
  }

  void heartbeat() {
    mavlink_message_t message;
    mavlink_msg_heartbeat_pack(254, MAV_COMP_ID_MISSIONPLANNER, &message,
        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0);
    send(message);
  }

  void set_offboard() {
    mavlink_message_t message;
    mavlink_msg_set_mode_pack(254, MAV_COMP_ID_MISSIONPLANNER, &message,
        1, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6u << 16);
    send(message);
  }

  void arm() {
    mavlink_message_t message;
    mavlink_msg_command_long_pack(254, MAV_COMP_ID_MISSIONPLANNER, &message,
        1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
    send(message);
  }

  void request_interval(uint32_t message_id, float interval_us) {
    mavlink_message_t message;
    mavlink_msg_command_long_pack(254, MAV_COMP_ID_MISSIONPLANNER, &message,
        1, 1, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        static_cast<float>(message_id), interval_us, 0, 0, 0, 0, 0);
    send(message);
  }

  void position(const std::array<double, 3>& position, float yaw) {
    mavlink_message_t message;
    constexpr uint16_t mask = 8 + 16 + 32 + 64 + 128 + 256 + 2048;
    mavlink_msg_set_position_target_local_ned_pack(254, MAV_COMP_ID_MISSIONPLANNER,
        &message, 0, 1, 1, MAV_FRAME_LOCAL_NED, mask,
        position[0], position[1], position[2], 0, 0, 0, 0, 0, 0, yaw, 0);
    send(message);
  }

  void rates(const std::array<double, 3>& body_rate, double thrust) {
    mavlink_message_t message;
    const float attitude[4] = {1, 0, 0, 0};
    const float thrust_body[3] = {0, 0, 0};
    mavlink_msg_set_attitude_target_pack(254, MAV_COMP_ID_MISSIONPLANNER, &message,
        0, 1, 1, 128, attitude, body_rate[0], body_rate[1], body_rate[2], thrust,
        thrust_body);
    send(message);
  }

 private:
  void send(const mavlink_message_t& message) {
    if (!peer_known_) return;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t size = mavlink_msg_to_send_buffer(buffer, &message);
    sendto(fd_, buffer, size, 0, reinterpret_cast<sockaddr*>(&peer_), sizeof(peer_));
  }

  int fd_{-1};
  sockaddr_in peer_{};
  bool peer_known_{};
  mavlink_status_t parser_status_{};
};

std::array<double, 4> quaternion(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll/2), sr = std::sin(roll/2);
  const double cp = std::cos(pitch/2), sp = std::sin(pitch/2);
  const double cy = std::cos(yaw/2), sy = std::sin(yaw/2);
  return {cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
          cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy};
}

double thrust_command(double force) {
  const double speed = std::sqrt(std::max(force, 0.0) / (4*kMotorConstant));
  return std::clamp((speed-kMotorMin)/(kMotorMax-kMotorMin), 0.0, 1.0);
}
}

int main(int argc, char** argv) {
  try {
    if (argc < 6) {
      std::cerr << "usage: controller reference solver rollout telemetry frequency [duration]\n";
      return 2;
    }
    auto reference = load_reference(argv[1]);
    auto solver = casadi::Function::load(argv[2]);
    auto rollout = casadi::Function::load(argv[3]);
    const double frequency = std::stod(argv[5]);
    const double duration = argc > 6 ? std::min(std::stod(argv[6]), reference.back().time)
                                     : reference.back().time;
    const double period = 1.0/frequency;

    MavlinkUdp link;
    Vehicle vehicle;
    const auto wait_start = std::chrono::steady_clock::now();
    while (!(vehicle.heartbeat && vehicle.local && vehicle.attitude)) {
      link.receive(vehicle); link.heartbeat();
      if (std::chrono::duration<double>(std::chrono::steady_clock::now()-wait_start).count() > 15)
        throw std::runtime_error("PX4 telemetry timeout");
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    link.request_interval(MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5000);
    link.request_interval(MAVLINK_MSG_ID_ATTITUDE, 5000);

    const std::array<double, 3> origin_ned = {
        4.0-vehicle.x, -16.0-vehicle.y, 5.76-vehicle.z};
    auto start_global = state_at(reference, 0.0);
    const std::array<double, 3> start = {
        start_global[0]-origin_ned[0], start_global[1]-origin_ned[1],
        start_global[2]-origin_ned[2]};
    const double yaw0 = std::atan2(
        2*(start_global[6]*start_global[9]+start_global[7]*start_global[8]),
        1-2*(start_global[8]*start_global[8]+start_global[9]*start_global[9]));

    for (int i=0; i<120; ++i) {
      link.receive(vehicle); link.position(start, yaw0); link.heartbeat();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    link.set_offboard();
    auto last_arm = std::chrono::steady_clock::time_point{};
    while (!vehicle.armed) {
      link.receive(vehicle); link.position(start, yaw0); link.heartbeat();
      const auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(now-last_arm).count() > 1) {
        link.arm(); last_arm = now;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    while (vehicle.boot_ms < 30000) {
      link.receive(vehicle); link.position(start, yaw0); link.heartbeat();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    std::ofstream telemetry(argv[4]);
    telemetry << "reference_time_s,actual_n_m,actual_e_m,actual_d_m,"
                 "reference_n_m,reference_e_m,reference_d_m,error_m,solve_ms\n";
    std::vector<double> guess(4*kN, kMass*kGravity/4);
    const std::vector<double> lower(4*kN, 0.0), upper(4*kN, 8.54858);
    std::array<double, 3> rate{0,0,0};
    double thrust = thrust_command(kMass*kGravity), next = 0, last_heartbeat = -1;
    size_t calls = 0;
    double solve_sum = 0, solve_max = 0;
    uint32_t clock_boot_ms = vehicle.boot_ms;
    auto clock_wall = std::chrono::steady_clock::now();

    while (true) {
      const uint32_t previous_boot_ms = vehicle.boot_ms;
      link.receive(vehicle);
      const auto now = std::chrono::steady_clock::now();
      if (vehicle.boot_ms != previous_boot_ms) {
        clock_boot_ms = vehicle.boot_ms;
        clock_wall = now;
      }
      // PX4 publishes the requested state at 125 Hz. Interpolate its simulation
      // timestamp between packets so a 100 Hz controller does not alternate
      // between 8 and 16 ms periods. The world itself remains at RTF=1.
      const double relative = std::max(0.0, clock_boot_ms/1000.0-kMotionStart +
          std::chrono::duration<double>(now-clock_wall).count());
      if (relative > duration) break;
      if (relative+1e-9 >= next) {
        const auto attitude = quaternion(vehicle.roll, vehicle.pitch, vehicle.yaw);
        const std::array<double, kNx> x0 = {vehicle.x,vehicle.y,vehicle.z,
            vehicle.vx,vehicle.vy,vehicle.vz,attitude[0],attitude[1],attitude[2],
            attitude[3],vehicle.p,vehicle.q,vehicle.r};
        std::vector<double> parameter(x0.begin(), x0.end());
        std::array<double, kNx> desired{};
        for (int k=0; k<=kN; ++k) {
          auto state = state_at(reference, std::min(relative+k*kNodeDt, duration));
          for (int i=0; i<3; ++i) state[i] -= origin_ned[i];
          if (state[6]*attitude[0]+state[7]*attitude[1]+state[8]*attitude[2]+state[9]*attitude[3] < 0)
            for (int i=6; i<10; ++i) state[i] *= -1;
          if (k == 0) desired = state;
          parameter.insert(parameter.end(), state.begin(), state.end());
        }
        for (int k=0; k<kN; ++k) {
          auto input = input_at(reference, std::min(relative+k*kNodeDt, duration));
          parameter.insert(parameter.end(), input.begin(), input.end());
        }
        std::map<std::string, casadi::DM> arguments = {
            {"x0", casadi::DM(guess)}, {"p", casadi::DM(parameter)},
            {"lbx", casadi::DM(lower)}, {"ubx", casadi::DM(upper)}};
        const auto solve_start = std::chrono::steady_clock::now();
        auto answer = solver(arguments);
        const double solve_ms = std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-solve_start).count();
        solve_sum += solve_ms; solve_max = std::max(solve_max, solve_ms); ++calls;
        auto controls = answer.at("x").get_elements();
        const bool finite = std::all_of(controls.begin(), controls.end(),
            [](double value){return std::isfinite(value);});
        if (finite) {
          for (int k=0; k<kN-1; ++k)
            for (int i=0; i<4; ++i) guess[4*k+i] = controls[4*(k+1)+i];
          for (int i=0; i<4; ++i) guess[4*(kN-1)+i] = controls[4*(kN-1)+i];
          auto states = rollout(std::vector<casadi::DM>{
              casadi::DM(std::vector<double>(x0.begin(),x0.end())),
              casadi::DM(controls)}).at(0).get_elements();
          rate = {states[23], states[24], states[25]};
          thrust = thrust_command(controls[0]+controls[1]+controls[2]+controls[3]);
        }
        const double dn=vehicle.x-desired[0], de=vehicle.y-desired[1], dd=vehicle.z-desired[2];
        telemetry << relative << ',' << vehicle.x << ',' << vehicle.y << ',' << vehicle.z
                  << ',' << desired[0] << ',' << desired[1] << ',' << desired[2]
                  << ',' << std::sqrt(dn*dn+de*de+dd*dd) << ',' << solve_ms << '\n';
        link.rates(rate, thrust);
        next = std::max(next+period, relative+0.6*period);
      }
      if (relative-last_heartbeat > 0.5) { link.heartbeat(); last_heartbeat=relative; }
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    std::cout << "calls=" << calls << " frequency=" << calls/duration
              << " mean_solve_ms=" << solve_sum/calls << " max_solve_ms=" << solve_max << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
