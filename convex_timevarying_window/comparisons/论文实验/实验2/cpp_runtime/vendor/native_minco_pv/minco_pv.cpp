#include "minco_pv.hpp"

#include <stdexcept>
#include <cmath>

namespace togt_pv {
namespace {

constexpr int kCoeffs = 8;
using Basis = Eigen::Matrix<double, 1, kCoeffs>;

constexpr std::array<double, 6> kFactorial{1, 1, 2, 6, 24, 120};

Basis basis(double t, int derivative) {
  Basis row = Basis::Zero();
  double power = 1.0;
  for (int column = derivative; column < kCoeffs; ++column) {
    double factor = 1.0;
    for (int order = 0; order < derivative; ++order) {
      factor *= column - order;
    }
    row(column) = factor * power;
    power *= t;
  }
  return row;
}

}  // namespace

MincoPV::MincoPV(const drolib::PVAJ& start, const drolib::PVAJ& finish, int pieces,
                 int derivative_order)
    : derivative_order_(derivative_order), start_(start), finish_(finish),
      durations_(pieces), coefficients_(kCoeffs * pieces, 3),
      normalized_coefficients_(kCoeffs * pieces, 3) {
  if (derivative_order_ < 1 || derivative_order_ > 3) {
    throw std::invalid_argument("MINCO knot derivative order must be 1, 2, or 3");
  }
  // Each additional prescribed derivative replaces the highest remaining
  // continuity condition, leaving C5, C4, or C3 respectively.
  for (int order = 1; order <= derivative_order_; ++order) {
    knot_orders_[3 - order] = order;
  }
  system_.create(kCoeffs * pieces, kCoeffs, kCoeffs);
}

bool MincoPV::fit(const Eigen::Matrix3Xd& points, const Eigen::Matrix3Xd& derivatives,
                  const Eigen::VectorXd& durations) {
  const int knots = durations_.size() - 1;
  if (durations.size() != durations_.size() || points.cols() != knots ||
      derivatives.cols() != derivative_order_ * knots || !durations.allFinite() ||
      (durations.array() <= 0.0).any()) {
    return false;
  }
  durations_ = durations;
  if (derivative_order_ == 3) {
    prescribed_points_ = points;
    prescribed_derivatives_ = derivatives;
    for (int piece = 0; piece <= knots; ++piece) {
      const long double t = durations_(piece);
      for (int axis = 0; axis < 3; ++axis) {
        long double endpoint[8];
        for (int side = 0; side < 2; ++side) {
          const int knot = piece + side - 1;
          for (int order = 0; order < 4; ++order) {
            const double state = knot < 0 ? start_(axis, order) : knot >= knots ?
              finish_(axis, order) : order == 0 ? points(axis, knot) :
              derivatives(axis, (order-1)*knots+knot);
            endpoint[side*4+order] = state * std::pow(t, order);
          }
        }
        for (int power = 0; power < 8; ++power) {
          long double value = 0.;
          if (power < 4) value = pvaj_unit_hermite_inverse[power][power] * endpoint[power];
          else {
            value = pvaj_unit_hermite_inverse[power][4] * (endpoint[4]-endpoint[0]);
            for (int order = 1; order < 4; ++order)
              value += pvaj_unit_hermite_inverse[power][order] * endpoint[order] +
                pvaj_unit_hermite_inverse[power][4+order] * endpoint[4+order];
          }
          normalized_coefficients_(8*piece+power,axis) = static_cast<double>(value);
          coefficients_(8*piece+power,axis) = static_cast<double>(value/std::pow(t,power));
        }
      }
    }
    return coefficients_.allFinite() && normalized_coefficients_.allFinite();
  }
  system_.reset();
  coefficients_.setZero();
  for (int order = 0; order < 4; ++order) {
    system_(order, order) = kFactorial[order];
    coefficients_.row(order) = start_.col(order).transpose();
  }

  const int last = knots;
  for (int piece = 0; piece < last; ++piece) {
    const int base = kCoeffs * piece;
    for (int equation = 0; equation < 8; ++equation) {
      const int row = base + 4 + equation;
      const int order = knot_orders_[equation];
      const Basis end = basis(durations_(piece), order);
      for (int power = order; power < kCoeffs; ++power) {
        system_(row, base + power) = end(power);
      }
      if (equation < 3 - derivative_order_ || equation >= 4) {
        system_(row, base + kCoeffs + order) = -kFactorial[order];
      }
    }
    coefficients_.row(base + 7) = points.col(piece).transpose();
    for (int order = 1; order <= derivative_order_; ++order) {
      coefficients_.row(base + 7 - order) = derivatives.col((order - 1) * knots + piece).transpose();
    }
  }

  const int base = kCoeffs * last;
  for (int order = 0; order < 4; ++order) {
    const Basis end = basis(durations_(last), order);
    for (int power = order; power < kCoeffs; ++power) {
      system_(base + 4 + order, base + power) = end(power);
    }
    coefficients_.row(base + 4 + order) = finish_.col(order).transpose();
  }

  system_.factorizeLU();
  system_.solve(coefficients_);
  for(int piece=0;piece<=knots;++piece)for(int power=0;power<8;++power)
    normalized_coefficients_.row(8*piece+power)=coefficients_.row(8*piece+power)*std::pow(durations_(piece),power);
  return coefficients_.allFinite() && normalized_coefficients_.allFinite();
}

drolib::PiecewisePolynomial<7> MincoPV::trajectory() const {
  drolib::PiecewisePolynomial<7> spline;
  for (int piece = 0; piece < durations_.size(); ++piece) {
    spline.emplace_back(durations_(piece),
        coefficients_.middleRows<kCoeffs>(kCoeffs * piece).transpose().rowwise().reverse());
  }
  return spline;
}

double MincoPV::addSnapEnergy(double weight, Eigen::MatrixX3d& grad_coefficients,
                              Eigen::VectorXd& grad_durations) const {
  if (weight <= 1e-6) {
    return 0.0;
  }
  // Four-point Gauss-Legendre integrates the degree-six squared snap exactly.
  constexpr std::array<double, 4> nodes{
      -0.8611363115940526, -0.3399810435848563, 0.3399810435848563, 0.8611363115940526};
  constexpr std::array<double, 4> weights{
      0.3478548451374539, 0.6521451548625461, 0.6521451548625461, 0.3478548451374539};
  double energy = 0.0;
  for (int piece = 0; piece < durations_.size(); ++piece) {
    const auto coeffs = coefficients_.middleRows<kCoeffs>(kCoeffs * piece);
    auto grad = grad_coefficients.middleRows<kCoeffs>(kCoeffs * piece);
    const double half_time = 0.5 * durations_(piece);
    for (int node = 0; node < 4; ++node) {
      const Basis snap_basis = basis(half_time * (1.0 + nodes[node]), 4);
      const Eigen::RowVector3d snap = snap_basis * coeffs;
      const double scale = half_time * weights[node] * weight;
      energy += scale * snap.squaredNorm();
      grad.noalias() += 2.0 * scale * snap_basis.transpose() * snap;
    }
    grad_durations(piece) += weight * (basis(durations_(piece), 4) * coeffs).squaredNorm();
  }
  return energy;
}

void MincoPV::propagate(const Eigen::MatrixX3d& grad_coefficients,
                        Eigen::Matrix3Xd& grad_points,
                        Eigen::Ref<Eigen::Matrix3Xd> grad_derivatives,
                        Eigen::VectorXd& grad_durations) const {
  if (derivative_order_ == 3) {
    const int knots = durations_.size()-1;
    grad_points.setZero();
    grad_derivatives.setZero();
    for(int piece=0;piece<=knots;++piece) {
      const long double t=durations_(piece);
      long double time_gradient=0.;
      for(int axis=0;axis<3;++axis) {
        long double endpoint[8];
        for(int side=0;side<2;++side)for(int order=0;order<4;++order) {
          const int knot=piece+side-1;
          endpoint[4*side+order]=knot<0 ? start_(axis,order) : knot>=knots ? finish_(axis,order) :
            order==0 ? prescribed_points_(axis,knot) : prescribed_derivatives_(axis,(order-1)*knots+knot);
        }
        for(int power=0;power<8;++power) {
          const long double weight=grad_coefficients(8*piece+power,axis);
          for(int side=0;side<2;++side)for(int order=0;order<4;++order) {
            const int knot=piece+side-1;
            if(knot<0 || knot>=knots)continue;
            const double value=static_cast<double>(weight*pvaj_unit_hermite_inverse[power][4*side+order]*std::pow(t,order-power));
            if(order==0)grad_points(axis,knot)+=value;
            else grad_derivatives(axis,(order-1)*knots+knot)+=value;
          }
          if(power>=4) {
            long double derivative=-power*pvaj_unit_hermite_inverse[power][4]*(endpoint[4]-endpoint[0])*std::pow(t,-power-1);
            for(int order=1;order<4;++order)
              derivative+=(order-power)*(pvaj_unit_hermite_inverse[power][order]*endpoint[order]+
                pvaj_unit_hermite_inverse[power][4+order]*endpoint[4+order])*std::pow(t,order-power-1);
            time_gradient+=weight*derivative;
          }
        }
      }
      grad_durations(piece)+=static_cast<double>(time_gradient);
    }
    return;
  }
  Eigen::MatrixX3d adjoint = grad_coefficients;
  system_.solveAdj(adjoint);
  const int last = durations_.size() - 1;
  for (int piece = 0; piece < last; ++piece) {
    const int base = kCoeffs * piece;
    const auto coeffs = coefficients_.middleRows<kCoeffs>(base);
    grad_points.col(piece) = adjoint.row(base + 7).transpose();
    for (int order = 1; order <= derivative_order_; ++order) {
      grad_derivatives.col((order - 1) * last + piece) = adjoint.row(base + 7 - order).transpose();
    }
    for (int equation = 0; equation < 8; ++equation) {
      const Eigen::RowVector3d derivative =
          basis(durations_(piece), knot_orders_[equation] + 1) * coeffs;
      grad_durations(piece) -= adjoint.row(base + 4 + equation).dot(derivative);
    }
  }
  const int base = kCoeffs * last;
  const auto coeffs = coefficients_.middleRows<kCoeffs>(base);
  for (int order = 0; order < 4; ++order) {
    const Eigen::RowVector3d derivative = basis(durations_(last), order + 1) * coeffs;
    grad_durations(last) -= adjoint.row(base + 4 + order).dot(derivative);
  }
}

}  // namespace togt_pv
