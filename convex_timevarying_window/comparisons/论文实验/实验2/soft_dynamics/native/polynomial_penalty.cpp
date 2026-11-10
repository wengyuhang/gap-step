// Reuse the exact frozen x500 TOGT integrator and its analytic gradients.
#include "togt_x500.cpp"

extern "C" int togt_polynomial_penalty_gradient(
    int n, const double* normalized_coefficients, const double* durations,
    double* cost, double* gradient_coefficients, double* gradient_durations,
    char* error, int capacity) {
  try {
    if (n < 1) throw std::runtime_error("invalid piece count");
    Eigen::VectorXd times(n);
    Eigen::MatrixX3d coefficients(8*n, 3);
    for (int piece = 0; piece < n; ++piece) {
      times(piece) = durations[piece];
      if (!(times(piece) > 0.) || !std::isfinite(times(piece)))
        throw std::runtime_error("invalid duration");
      for (int power = 0; power < 8; ++power)
        for (int axis = 0; axis < 3; ++axis)
          coefficients(8*piece+power, axis) =
              normalized_coefficients[24*piece+3*power+axis] /
              std::pow(times(piece), power);
    }
    drolib::TrajParams parameters;
    standard(parameters);
    drolib::QuadManifold quad(quad_params());
    Eigen::MatrixX3d gradient = Eigen::MatrixX3d::Zero(8*n, 3);
    Eigen::VectorXd partial_times = Eigen::VectorXd::Zero(n);
    *cost = integrate(times, coefficients, quad, parameters, gradient, partial_times);
    if (!std::isfinite(*cost)) throw std::runtime_error("nonfinite penalty");
    for (int piece = 0; piece < n; ++piece) {
      double time_gradient = partial_times(piece);
      for (int power = 0; power < 8; ++power) {
        time_gradient -= power/times(piece) *
            gradient.row(8*piece+power).dot(coefficients.row(8*piece+power));
        for (int axis = 0; axis < 3; ++axis) {
          const double value = gradient(8*piece+power, axis) /
                               std::pow(times(piece), power);
          if (!std::isfinite(value)) throw std::runtime_error("nonfinite gradient");
          gradient_coefficients[24*piece+3*power+axis] = value;
        }
      }
      if (!std::isfinite(time_gradient)) throw std::runtime_error("nonfinite time gradient");
      gradient_durations[piece] = time_gradient;
    }
    return 0;
  } catch (const std::exception& exception) {
    if (error && capacity) {
      std::strncpy(error, exception.what(), capacity-1);
      error[capacity-1] = 0;
    }
    return 1;
  }
}
