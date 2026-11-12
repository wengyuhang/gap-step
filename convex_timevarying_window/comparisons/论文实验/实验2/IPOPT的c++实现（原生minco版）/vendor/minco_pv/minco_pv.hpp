#pragma once

#include <array>

#include "drolib/solver/banded_system.hpp"
#include "drolib/type/types.hpp"
#include "drolib/polynomial/piecewise_polynomial.hpp"

namespace togt_pv {

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
};

}  // namespace togt_pv
