#include "model.hpp"
#include <iostream>
using State = Eigen::Matrix<double, 13, 1>;
struct Reference {
  Model model;
  double T, h;
  int N;
  std::vector<State> states;
  std::vector<Eigen::Vector4d> controls;
  std::vector<std::array<State, 5>> sub;
  Reference(const Json &r)
      : model(r["vehicle"]), T(r["nominal_time_s"]), N(r["nodes"]) {
    h = T / N;
    for (auto &s : r["states"]) {
      State x;
      for (int k = 0; k < 13; ++k)
        x[k] = s[k];
      states.push_back(x);
    }
    for (auto &u : r["controls"]) {
      Eigen::Vector4d v;
      for (int k = 0; k < 4; ++k)
        v[k] = u[k];
      controls.push_back(v);
    }
    for (int i = 0; i < N; ++i) {
      std::array<State, 5> a;
      a[0] = states[i];
      for (int k = 0; k < 4; ++k)
        a[k + 1] = model.rk4(a[k], controls[i], h / 4);
      sub.push_back(a);
    }
  }
  State sample(double t) const {
    if (t >= T)
      return states.back();
    int i = std::min(N - 1, int(t / h));
    double local = std::max(0., t - i * h);
    int k = std::min(3, int(local / (h / 4)));
    return model.rk4(sub[i][k], controls[i], local - k * h / 4);
  }
};
int main(int argc, char **argv) {
  try {
    if (argc != 4)
      throw std::runtime_error("audit RESULT SCENE OUTPUT");
    auto began = std::chrono::steady_clock::now();
    auto r = read(argv[1]), scene = read(argv[2]);
    if (!r.value("found", false)) {
      save(argv[3], {{"verdict", "未找到"},
                     {"passed", false},
                     {"solver_qualified", false},
                     {"check_s", 0.},
                     {"reason", "finite CPC constraints not satisfied; "
                                "geometric qualification not claimed"}});
      return 0;
    }
    Reference ref(r);
    int steps = std::ceil(ref.T / .001);
    double dt = ref.T / steps;
    std::vector<Vector> position;
    position.reserve(steps + 1);
    double ground = INFINITY, qerror = 0;
    for (int k = 0; k <= steps; ++k) {
      auto x = ref.sample(k * dt);
      position.push_back(x.head<3>());
      ground = std::min(ground, x[2] - .15);
      qerror = std::max(qerror, std::abs(x.segment<4>(6).norm() - 1.));
    }
    Json checks = Json::array(), events = Json::array(),
         targets = Json::array();
    bool collision = false, ordered = true;
    double previous = -INFINITY;
    for (auto &row : scene["windows"]) {
      Window w = load_window(row);
      Json crossings = Json::array(), ignored_contacts = Json::array();
      double closest = INFINITY, closestTime = 0;
      for (int k = 0; k <= steps; ++k) {
        double t = k * dt;
        double d = (position[k] - w.center - w.rotation * w.motion(t)).norm();
        if (d < closest) {
          closest = d;
          closestTime = t;
        }
      }
      targets.push_back({{"gate", row["name"]},
                         {"minimum_sample_center_distance_m", closest},
                         {"time_s", closestTime},
                         {"within_0p3_m", closest <= .3 + 1e-5}});
      if (w.waypoint) {
        bool reached =
            closest <= w.waypoint_radius + 1e-5 && closestTime > previous;
        ordered = ordered && reached;
        previous = closestTime;
        events.push_back({{"gate", row["name"]},
                          {"kind", "navigation_waypoint"},
                          {"passed", reached},
                          {"time_s", closestTime},
                          {"distance_m", closest}});
        continue;
      }
      Mesh mesh(row["mesh"]);
      double clearance = INFINITY, witness = 0;
      int checked = 0;
      for (int k = 0; k <= steps; ++k) {
        double t = k * dt;
        Vector local =
            w.rotation.transpose() * (position[k] - w.center) - w.motion(t);
        double bound = (mesh.low - local)
                           .cwiseMax(local - mesh.high)
                           .cwiseMax(Vector::Zero())
                           .norm();
        if (bound <= .15 + .05) {
          double d = mesh.distance(local) - .15;
          ++checked;
          if (d < clearance) {
            clearance = d;
            witness = t;
          }
        }
        if (k == 0)
          continue;
        double before = w.rotation.col(2).dot(position[k - 1] - w.center),
               after = w.rotation.col(2).dot(position[k] - w.center);
        if (before * after >= 0.)
          continue;
        double a = (k - 1) * dt, b = t;
        for (int it = 0; it < 42; ++it) {
          double middle = (a + b) / 2;
          double value =
              w.rotation.col(2).dot(ref.sample(middle).head<3>() - w.center);
          if (before * value > 0) {
            a = middle;
            before = value;
          } else
            b = middle;
        }
        double at = (a + b) / 2;
        auto state = ref.sample(at);
        Vector q = w.rotation.transpose() * (state.head<3>() - w.center) -
                   w.motion(at);
        if (!w.contains(q.head<2>()))
          continue;
        if (!scene.value("stationary_terminal_contact_is_crossing", true) &&
            at >= ref.T - ref.h - 1e-9 &&
            std::abs(w.rotation.col(2).dot(ref.states.back().head<3>() -
                                           w.center)) < 1e-7 &&
            (ref.states[ref.N - 1].head<3>() - ref.states.back().head<3>())
                    .norm() < 1e-5 &&
            ref.states[ref.N - 1].segment<3>(3).norm() < 1e-5 &&
            ref.states.back().segment<3>(3).norm() < 1e-5 &&
            ref.states[ref.N - 1].tail<3>().norm() < 1e-5 &&
            ref.states.back().tail<3>().norm() < 1e-5 &&
            (ref.controls.back() -
             Eigen::Vector4d::Constant(ref.model.mass * ref.model.gravity / 4))
                    .norm() < 1e-5) {
          ignored_contacts.push_back(
              {{"time_s", at},
               {"reason", "stationary_terminal_hover_interval_under_existing_"
                          "scene_rule"}});
          continue;
        }
        bool duplicate =
            !crossings.empty() &&
            std::abs(crossings.back()["time_s"].get<double>() - at) < 1e-6;
        if (!duplicate)
          crossings.push_back(
              {{"time_s", at},
               {"forward", row["sign"].get<double>() *
                                   state.segment<3>(3).dot(w.rotation.col(2)) <
                               -1e-8},
               {"local_point", {q[0], q[1]}}});
      }
      bool hit = clearance < -1e-8;
      collision = collision || hit;
      checks.push_back(
          {{"gate", row["name"]},
           {"collision", hit},
           {"checked_samples", checked},
           {"minimum_sample_clearance_m",
            std::isfinite(clearance) ? Json(clearance) : Json(nullptr)},
           {"time_s", witness}});
      bool single =
          crossings.size() == 1 && crossings[0]["forward"].get<bool>();
      ordered = ordered && single;
      if (single) {
        double at = crossings[0]["time_s"];
        ordered = ordered && at > previous;
        previous = at;
      }
      events.push_back({{"gate", row["name"]},
                        {"events", crossings},
                        {"ignored_terminal_contacts", ignored_contacts}});
    }
    double endpoint = 0;
    for (int end = 0; end < 2; ++end) {
      auto x = end ? ref.states.back() : ref.states.front();
      auto u = end ? ref.controls.back() : ref.controls.front();
      auto target = scene[end ? "goal" : "start"].get<std::vector<double>>();
      endpoint = std::max(
          endpoint,
          (x.head<3>() - Vector(target[0], target[1], target[2])).norm());
      endpoint = std::max(endpoint, x.segment<3>(3).norm());
      endpoint = std::max(endpoint, x.tail<3>().norm());
      endpoint =
          std::max(endpoint, ref.model.derivative(x, u).segment<3>(3).norm());
    }
    double defect = 0, drift = 0;
    State forward = ref.states[0];
    for (int i = 0; i < ref.N; ++i) {
      defect = std::max(
          defect, (ref.sub[i][4] - ref.states[i + 1]).cwiseAbs().maxCoeff());
      for (int j = 0; j < 16; ++j)
        forward = ref.model.rk4(forward, ref.controls[i], ref.h / 16);
      drift = std::max(
          drift, (forward.head<3>() - ref.states[i + 1].head<3>()).norm());
    }
    bool found = r["found"],
         passed = found && !collision && ordered && endpoint < 1e-5;
    std::string verdict = !found                         ? "未找到"
                          : collision                    ? "碰撞"
                          : !ordered || endpoint >= 1e-5 ? "未完成"
                                                         : "通过";
    save(argv[3],
         {{"verdict", verdict},
          {"passed", passed},
          {"collision_detected", collision},
          {"ordered_exactly_once", ordered},
          {"endpoints_passed", endpoint < 1e-5},
          {"endpoint_error", endpoint},
          {"solver_qualified", found},
          {"checks", checks},
          {"opening_events", events},
          {"center_targets", targets},
          {"sampling_step_s", dt},
          {"quaternion_norm_error", qerror},
          {"maximum_RK4_node_defect", defect},
          {"finer_rollout_max_position_drift_m", drift},
          {"ground_collision", ground < 0},
          {"full_trajectory_time_s", ref.T},
          {"check_s", std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - began)
                          .count()},
          {"evidence",
           "<=1ms native RK4 samples/STL sphere distance and bracketed "
           "finite-aperture events; NOT continuous certificate"}});
    std::cout << verdict << " " << ref.T << " drift " << drift << std::endl;
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
