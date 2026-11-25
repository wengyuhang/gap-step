#include "model.hpp"
#include <iostream>
int main(int argc, char **argv) {
  try {
    auto v = read(argv[1]);
    Model m(v);
    double ode_error = 0, step_error = 0, motion_error = 0;
    Eigen::Matrix4d author;
    double l = v["arm_length_m"].get<double>() / std::sqrt(2.),
           ct = v["torque_coefficient"];
    author << 1, 1, 1, 1, l, -l, -l, l, -l, -l, l, l, ct, -ct, ct, -ct;
    int order[4] = {1, 3, 2, 0};
    double mixer_error = 0;
    for (int i = 0; i < 4; ++i)
      mixer_error =
          std::max(mixer_error, (m.mixer.col(i) - author.col(order[i])).norm());
    for (int k = 0; k < 20; ++k) {
      Eigen::Matrix<double, 13, 1> x;
      for (int j = 0; j < 13; ++j)
        x[j] = .3 * std::sin(.7 * k + .2 * j);
      x[6] += 1.;
      x.segment<4>(6).normalize();
      Eigen::Vector4d u;
      for (int j = 0; j < 4; ++j)
        u[j] = 2 + .4 * std::sin(k + j);
      Eigen::Quaterniond q(x[6], x[7], x[8], x[9]),
          omega(0, x[10], x[11], x[12]), force(0, 0, 0, u.sum() / m.mass);
      auto rotated = q * force * q.conjugate(), qd = q * omega;
      Eigen::Vector4d ua;
      for (int j = 0; j < 4; ++j)
        ua[order[j]] = u[j];
      Eigen::Matrix<double, 13, 1> expected;
      expected.head<3>() = x.segment<3>(3);
      expected.segment<3>(3) = Vector(rotated.x(), rotated.y(), rotated.z()) -
                               Vector(0, 0, m.gravity);
      expected.segment<4>(6) << .5 * qd.w(), .5 * qd.x(), .5 * qd.y(),
          .5 * qd.z();
      expected.tail<3>() =
          (author.block<3, 4>(1, 0) * ua -
           x.tail<3>().cross(m.inertia.cwiseProduct(x.tail<3>())))
              .cwiseQuotient(m.inertia);
      auto actual = m.ode(std::vector<DM>{dm(x), dm(u)})[0].get_elements();
      for (int j = 0; j < 13; ++j)
        ode_error = std::max(ode_error, std::abs(actual[j] - expected[j]));
      double h = .005 + .001 * k;
      auto y = x;
      for (int j = 0; j < 4; ++j)
        y = m.rk4(y, u, h / 4);
      auto z = m.step(std::vector<DM>{dm(x), dm(u), DM(h)})[0].get_elements();
      for (int j = 0; j < 13; ++j)
        step_error = std::max(step_error, std::abs(y[j] - z[j]));
    }
    auto scene = read(argv[2]);
    for (auto &r : scene["windows"]) {
      auto w = load_window(r);
      MX t = MX::sym("t");
      auto p = center(w, t);
      Function f("motion", {t}, {p, MX::jacobian(p, t)});
      for (double at : {0., .31, 1.27, 4.9}) {
        auto a = f(std::vector<DM>{DM(at)});
        Vector expected = w.center + w.rotation * w.motion(at),
               vel = w.rotation * w.motion_velocity(at);
        for (int j = 0; j < 3; ++j) {
          motion_error = std::max(
              motion_error, std::abs(a[0].get_elements()[j] - expected[j]));
          motion_error =
              std::max(motion_error, std::abs(a[1].get_elements()[j] - vel[j]));
        }
      }
    }
    bool passed = ode_error < 1e-10 && step_error < 1e-10 &&
                  mixer_error < 1e-12 && motion_error < 1e-10;
    Json result = {{"passed", passed},
                   {"author_permuted_mixer_error", mixer_error},
                   {"author_quaternion_dynamics_error", ode_error},
                   {"numeric_RK4_vs_CasADi_error", step_error},
                   {"window_position_velocity_error", motion_error},
                   {"physical_to_author_motor_indices", {1, 3, 2, 0}}};
    save(argv[3], result);
    std::cout << result.dump(2) << '\n';
    return passed ? 0 : 1;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
