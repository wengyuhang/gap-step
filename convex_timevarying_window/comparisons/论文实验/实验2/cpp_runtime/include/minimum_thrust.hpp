#pragma once

#include "numpy_passage.hpp"

namespace minimum_thrust {

// Match the current execution model's 150 rad/s motor-speed floor.
constexpr double motor_constant = 8.54858e-6;
constexpr double motor_speed_floor = 150.;
constexpr double collective_floor = 4. * motor_constant * motor_speed_floor * motor_speed_floor;

inline Json check(const Curve& curve, double duration, double mass, double gravity,
                  double minimum_force = collective_floor, double tolerance = 1e-6) {
    const auto began = std::chrono::steady_clock::now();
    if (!(duration > 0.) || !(mass > 0.) || !(gravity > 0.) || !(minimum_force > 0.) ||
        !std::isfinite(duration) || !curve.allFinite())
        throw std::runtime_error("Invalid minimum-thrust check inputs");
    Eigen::Matrix<double, 6, 3> acceleration;
    for (int k = 0; k < 6; ++k)
        acceleration.row(k) = (k + 2.) * (k + 1.) * curve.row(k + 2) / (duration * duration);
    acceleration(0, 2) += gravity;
    Eigen::VectorXd squared = Eigen::VectorXd::Zero(11);
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            squared[i + j] += acceleration.row(i).dot(acceleration.row(j));
    Eigen::VectorXd derivative(10);
    for (int k = 0; k < 10; ++k) derivative[k] = (k + 1.) * squared[k + 1];
    auto stationary = numpy_roots(derivative);
    std::vector<double> candidates{0., 1.};
    for (double u : stationary) if (u > 0. && u < 1.) candidates.push_back(u);
    double minimum_squared = INFINITY, witness = 0.;
    for (double u : candidates) {
        Vector value = acceleration.row(5).transpose();
        for (int k = 4; k >= 0; --k) value = value * u + acceleration.row(k).transpose();
        const double squared_norm = value.squaredNorm();
        if (squared_norm < minimum_squared) { minimum_squared = squared_norm; witness = u; }
    }
    const double acceleration_floor = minimum_force / mass;
    const double residual = (acceleration_floor * acceleration_floor - minimum_squared) / (gravity * gravity);
    return Json{{"passed", residual <= tolerance}, {"minimum_total_thrust_n", mass * std::sqrt(minimum_squared)},
        {"minimum_force_per_mass", std::sqrt(minimum_squared)}, {"required_total_thrust_n", minimum_force},
        {"normalized_residual", residual}, {"witness_u", witness}, {"candidate_fractions", candidates},
        {"check_s", std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count()},
        {"evidence", "floating-point polynomial stationary roots and endpoint evaluation"}};
}

} // namespace minimum_thrust
