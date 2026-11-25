#pragma once

#include <array>

#include "drolib/solver/banded_system.hpp"
#include "drolib/type/types.hpp"
#include "drolib/polynomial/piecewise_polynomial.hpp"

namespace togt_pv {

// Exact inverse of the unit-interval endpoint PVAJ interpolation matrix.
inline constexpr long double pvaj_unit_hermite_inverse[8][8] = {
 {1,0,0,0,0,0,0,0}, {0,1,0,0,0,0,0,0},
 {0,0,.5L,0,0,0,0,0}, {0,0,0,1.L/6,0,0,0,0},
 {-35,-20,-5,-2.L/3,35,-15,2.5L,-1.L/6},
 {84,45,10,1,-84,39,-7,.5L},
 {-70,-36,-7.5L,-2.L/3,70,-34,6.5L,-.5L},
 {20,10,2,1.L/6,-20,10,-2,1.L/6}
};

// Seventh-degree minimum-snap spline with fixed PV, PVA, or PVAJ at
// interior knots and fixed PVAJ at both endpoints.
class MincoPV {
 public:
  MincoPV(const drolib::PVAJ& start, const drolib::PVAJ& finish, int pieces,
          int derivative_order = 1);
  ~MincoPV() { system_.destroy(); }
  MincoPV(const MincoPV&) = delete;
  MincoPV& operator=(const MincoPV&) = delete;

  // Derivative columns are grouped by order: all V, then all A, then all J.
  bool fit(const Eigen::Matrix3Xd& points, const Eigen::Matrix3Xd& derivatives,
           const Eigen::VectorXd& durations);
  const Eigen::MatrixX3d& coefficients() const { return coefficients_; }
  const Eigen::MatrixX3d& normalizedCoefficients() const { return normalized_coefficients_; }
  drolib::PiecewisePolynomial<7> trajectory() const;
  double addSnapEnergy(double weight, Eigen::MatrixX3d& grad_coefficients,
                       Eigen::VectorXd& grad_durations) const;
  void propagate(const Eigen::MatrixX3d& grad_coefficients,
                 Eigen::Matrix3Xd& grad_points, Eigen::Ref<Eigen::Matrix3Xd> grad_derivatives,
                 Eigen::VectorXd& grad_durations) const;

 private:
  const int derivative_order_;
  std::array<int, 8> knot_orders_{4, 5, 1, 0, 0, 1, 2, 3};
  drolib::PVAJ start_;
  drolib::PVAJ finish_;
  drolib::BandedSystem system_;
  Eigen::VectorXd durations_;
  Eigen::MatrixX3d coefficients_;
  Eigen::MatrixX3d normalized_coefficients_;
  Eigen::Matrix3Xd prescribed_points_, prescribed_derivatives_;
};

}  // namespace togt_pv
