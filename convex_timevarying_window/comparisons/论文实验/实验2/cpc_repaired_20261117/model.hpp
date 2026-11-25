// SPDX-License-Identifier: GPL-3.0-only
// C++ adaptation of uzh-rpg/rpg_time_optimal (Foehn, Romero, Scaramuzza).
// Original source and license retained under
// 复现/cpc_dynamic/vendor/rpg_time_optimal.
#pragma once
#include "mesh.hpp"
#include <casadi/casadi.hpp>
#include <filesystem>
#include <fstream>
using casadi::DM;
using casadi::Function;
using casadi::MX;
using casadi::SX;
namespace fs = std::filesystem;
inline Json read(const fs::path &p) {
  Json j;
  std::ifstream f(p);
  if (!f)
    throw std::runtime_error(p.string());
  f >> j;
  return j;
}
inline void save(const fs::path &p, const Json &j) {
  fs::create_directories(p.parent_path());
  std::ofstream(p) << j.dump(2) << '\n';
}
inline DM dm(const Eigen::MatrixXd &m) {
  DM v = DM::zeros(m.rows(), m.cols());
  for (int i = 0; i < m.rows(); ++i)
    for (int k = 0; k < m.cols(); ++k)
      v(i, k) = m(i, k);
  return v;
}
struct Model {
  static constexpr int integration_substeps = 16;
  Function ode, step, substep, rawstep;
  double mass, gravity;
  Eigen::Vector3d inertia;
  Eigen::Matrix4d mixer;
  explicit Model(const Json &v) {
    mass = v["mass"];
    gravity = v["gravity"];
    for (int i = 0; i < 3; ++i)
      inertia[i] = v["inertia"][i];
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j)
        mixer(i, j) = v["mixer"][i][j];
    if (mixer.fullPivLu().rank() != 4)
      throw std::runtime_error("Invalid mixer");
    SX x = SX::sym("x", 13), u = SX::sym("u", 4), dt = SX::sym("dt");
    SX q = x(casadi::Slice(6, 10)), w = x(casadi::Slice(10, 13));
    SX qr = q / SX::sqrt(SX::dot(q,q));
    SX torque = SX::mtimes(dm(mixer.block<3, 4>(1, 0)), u);
    SX rotated = SX::vertcat(
        {2 * (qr(1) * qr(3) + qr(0) * qr(2)), 2 * (qr(2) * qr(3) - qr(0) * qr(1)),
         qr(0) * qr(0) - qr(1) * qr(1) - qr(2) * qr(2) + qr(3) * qr(3)});
    SX qdot =
        .5 * SX::vertcat({-SX::dot(q(casadi::Slice(1, 4)), w),
                          q(0) * w + SX::cross(q(casadi::Slice(1, 4)), w)});
    SX I = SX(dm(inertia.asDiagonal().toDenseMatrix()));
    SX invI = SX(dm(inertia.cwiseInverse().asDiagonal().toDenseMatrix()));
    SX dx = SX::vertcat(
        {x(casadi::Slice(3, 6)),
         rotated * SX::sum1(u) / mass +
             SX(DM(std::vector<double>{0, 0, -gravity})),
         qdot, SX::mtimes(invI, torque - SX::cross(w, SX::mtimes(I, w)))});
    SX rawdx=dx;
    SX rawrot=SX::vertcat({2*(q(1)*q(3)+q(0)*q(2)),2*(q(2)*q(3)-q(0)*q(1)),q(0)*q(0)-q(1)*q(1)-q(2)*q(2)+q(3)*q(3)});
    rawdx(casadi::Slice(3,6))=rawrot*SX::sum1(u)/mass+SX(DM(std::vector<double>{0,0,-gravity}));
    Function rawode("rawode",{x,u},{rawdx});
    SX coarse=x;
    for(int k=0;k<4;++k){SX h=dt/4; auto a=rawode(std::vector<SX>{coarse,u})[0],b=rawode(std::vector<SX>{coarse+h*a/2,u})[0],c=rawode(std::vector<SX>{coarse+h*b/2,u})[0],d=rawode(std::vector<SX>{coarse+h*c,u})[0];coarse+=h*(a+2*b+2*c+d)/6;}
    rawstep=Function("rawstep",{x,u,dt},{coarse});
    ode = Function("ode", {x, u}, {dx});
    auto advance = [&](SX a, SX h) {
      auto k1 = ode(std::vector<SX>{a, u})[0],
           k2 = ode(std::vector<SX>{a + h * k1 / 2, u})[0],
           k3 = ode(std::vector<SX>{a + h * k2 / 2, u})[0],
           k4 = ode(std::vector<SX>{a + h * k3, u})[0];
      SX result = a + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
      return result;
    };
    substep = Function("rk4", {x, u, dt}, {advance(x, dt)});
    SX next = x;
    for (int k = 0; k < integration_substeps; ++k)
      next = advance(next, dt / integration_substeps);
    step = Function("rk4_four", {x, u, dt}, {next});
  }
  Eigen::Matrix<double, 13, 1> derivative(const Eigen::Matrix<double, 13, 1> &x,
                                          const Eigen::Vector4d &u) const {
    Eigen::Matrix<double, 13, 1> d;
    Eigen::Vector4d q = x.segment<4>(6).normalized();
    auto w = x.tail<3>();
    d.head<3>() = x.segment<3>(3);
    d.segment<3>(3) =
        Vector(2 * (q[1] * q[3] + q[0] * q[2]), 2 * (q[2] * q[3] - q[0] * q[1]),
               q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) *
            u.sum() / mass -
        Vector(0, 0, gravity);
    auto raw_q=x.segment<4>(6);
    d[6] = -.5 * raw_q.tail<3>().dot(w);
    d.segment<3>(7) = .5 * (raw_q[0] * w + raw_q.tail<3>().cross(w));
    d.tail<3>() =
        (mixer.block<3, 4>(1, 0) * u - w.cross(inertia.cwiseProduct(w)))
            .cwiseQuotient(inertia);
    return d;
  }
  Eigen::Matrix<double, 13, 1> rk4(const Eigen::Matrix<double, 13, 1> &x,
                                   const Eigen::Vector4d &u, double h) const {
    auto k1 = derivative(x, u), k2 = derivative(x + h * k1 / 2, u),
         k3 = derivative(x + h * k2 / 2, u), k4 = derivative(x + h * k3, u);
    Eigen::Matrix<double,13,1> result = x + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6;
    return result;
  }
};
inline MX center(const Window &w, const MX &t) {
  MX phase = 2 * pi * w.frequency * t + w.phase, m;
  if (w.swing) {
    MX angle = std::asin(w.amplitude / w.length) * MX::sin(phase);
    m = MX::vertcat(
        {w.length * (1 - MX::cos(angle)), -w.length * MX::sin(angle), MX(0)});
  } else
    m = MX::vertcat({MX(0), -w.amplitude * MX::sin(phase), MX(0)});
  double c = std::cos(w.motion_angle), s = std::sin(w.motion_angle);
  m = MX::vertcat({c * m(0) - s * m(1), s * m(0) + c * m(1), m(2)});
  return MX(dm(w.center)) + MX::mtimes(dm(w.rotation), m);
}
