#pragma once

#include "geometry.hpp"
#include <cstdint>

// These ILP64 entry points are the same OpenBLAS/LAPACK library used by wyh NumPy.
extern "C" {
void dgeev_64_(const char*, const char*, const int64_t*, double*, const int64_t*,
    double*, double*, double*, const int64_t*, double*, const int64_t*,
    double*, const int64_t*, int64_t*);
double cblas_ddot64_(int64_t, const double*, int64_t, const double*, int64_t);
void cblas_dgemm64_(int, int, int, int64_t, int64_t, int64_t, double,
    const double*, int64_t, const double*, int64_t, double, double*, int64_t);
void cblas_dgemv64_(int, int, int64_t, int64_t, double, const double*, int64_t,
    const double*, int64_t, double, double*, int64_t);
}

inline double numpy_dot(const double* first, const double* second, int64_t count) {
    return cblas_ddot64_(count, first, 1, second, 1);
}

inline std::vector<double> numpy_roots(const Eigen::VectorXd& polynomial) {
    int64_t degree = polynomial.size() - 1;
    while (degree > 0 && polynomial[degree] == 0.) --degree;
    if (degree == 0) return {};
    if (degree == 1) return {-polynomial[0] / polynomial[1]};

    Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(degree, degree);
    for (int64_t index = 1; index < degree; ++index) companion(index, index - 1) = 1.;
    for (int64_t index = 0; index < degree; ++index)
        companion(index, degree - 1) -= polynomial[index] / polynomial[degree];
    companion = companion.reverse().eval();

    const char vectors = 'N';
    const int64_t leading = degree, unused_leading = 1;
    int64_t workspace_size = -1, status = 0;
    double optimum = 0., unused_left = 0., unused_right = 0.;
    std::vector<double> real(degree), imaginary(degree);
    dgeev_64_(&vectors, &vectors, &degree, companion.data(), &leading,
        real.data(), imaginary.data(), &unused_left, &unused_leading,
        &unused_right, &unused_leading, &optimum, &workspace_size, &status);
    if (status != 0) throw std::runtime_error("LAPACK polynomial workspace query failed");
    workspace_size = static_cast<int64_t>(optimum);
    std::vector<double> workspace(workspace_size);
    dgeev_64_(&vectors, &vectors, &degree, companion.data(), &leading,
        real.data(), imaginary.data(), &unused_left, &unused_leading,
        &unused_right, &unused_leading, workspace.data(), &workspace_size, &status);
    if (status != 0) throw std::runtime_error("LAPACK polynomial eigensolve failed");
    std::vector<double> result;
    for (int64_t index = 0; index < degree; ++index)
        if (std::abs(imaginary[index]) < 1e-7) result.push_back(real[index]);
    std::sort(result.begin(), result.end());
    return result;
}

inline Curve numpy_local_curve(const Curve& world, const Window& window) {
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotation = window.rotation;
    Eigen::Matrix<double, 8, 3, Eigen::RowMajor> product;
    // The original physical curve is F contiguous, the aperture rotation is C contiguous.
    cblas_dgemm64_(101, 112, 111, 8, 3, 3, 1., world.data(), 8,
        rotation.data(), 3, 0., product.data(), 3);
    Vector origin;
    cblas_dgemv64_(101, 112, 3, 3, 1., rotation.data(), 3,
        window.center.data(), 1, 0., origin.data(), 1);
    for (int axis = 0; axis < 3; ++axis) product(0, axis) -= origin[axis];
    return product;
}

inline Curve numpy_bernstein(const Curve& power) {
    Eigen::Matrix<double, 8, 8, Eigen::RowMajor> transform = Eigen::Matrix<double, 8, 8>::Zero();
    for (int row = 0; row < 8; ++row)
        for (int column = 0; column <= row; ++column)
            transform(row, column) = binomial(row, column) / binomial(7, column);
    Eigen::Matrix<double, 8, 3, Eigen::RowMajor> coefficients = power, controls;
    cblas_dgemm64_(101, 111, 111, 8, 3, 8, 1., transform.data(), 8,
        coefficients.data(), 3, 0., controls.data(), 3);
    return controls;
}

inline double numpy_frame_distance(const Window& window, const Vector& point) {
    if (window.circle) {
        const double norm = std::sqrt(numpy_dot(point.data(), point.data(), 2));
        const double distance = norm - (window.opening_radius + .085);
        return distance * distance + point.z() * point.z();
    }
    double minimum = INFINITY;
    for (size_t index = 0; index < window.frame.size(); ++index) {
        const Eigen::Vector2d edge = window.frame[(index + 1) % window.frame.size()] - window.frame[index];
        const Eigen::Vector2d delta = point.head<2>() - window.frame[index];
        const double length_squared = edge[0] * edge[0] + edge[1] * edge[1];
        const double fraction = std::clamp((delta[0] * edge[0] + delta[1] * edge[1]) / length_squared, 0., 1.);
        const Eigen::Vector2d residual = delta - fraction * edge;
        minimum = std::min(minimum, residual[0] * residual[0] + residual[1] * residual[1]);
    }
    return minimum + point.z() * point.z();
}

inline Json numpy_passage(const Curve& world, double duration, double start,
        const Window& window, double radius) {
    const Curve local = numpy_local_curve(world, window);
    auto position = [&](double fraction) -> Vector {
        return evaluate(local, fraction) - window.motion(start + fraction * duration);
    };
    for (double fraction : numpy_roots(local.col(2))) {
        if (fraction > 1e-6 && fraction < 1. - 1e-6 && window.contains(position(fraction).head<2>()))
            return Json{{"passed", false}, {"outcome", "repeat_aperture_crossing"},
                {"witness_u", nullptr}, {"boxes", 0}, {"minimum_sample_distance_m", 0.}};
    }
    std::vector<double> cuts{0., 1.};
    for (double level : {-radius, radius}) {
        Eigen::VectorXd polynomial = local.col(2);
        polynomial[0] -= level;
        for (double fraction : numpy_roots(polynomial))
            if (fraction > 0. && fraction < 1.) cuts.push_back(fraction);
    }
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());

    using Rows2 = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;
    Rows2 tangents(window.frame.size(), 2), normals(window.frame.size(), 2);
    std::vector<double> lengths(window.frame.size()), along_origin(window.frame.size()), across_origin(window.frame.size());
    for (size_t index = 0; index < window.frame.size(); ++index) {
        const Eigen::Vector2d edge = window.frame[(index + 1) % window.frame.size()] - window.frame[index];
        lengths[index] = std::sqrt(edge[0] * edge[0] + edge[1] * edge[1]);
        tangents.row(index) = edge / lengths[index];
        normals.row(index) = Eigen::Vector2d(tangents(index, 1), -tangents(index, 0));
        along_origin[index] = window.frame[index][0] * tangents(index, 0) + window.frame[index][1] * tangents(index, 1);
        across_origin[index] = window.frame[index][0] * normals(index, 0) + window.frame[index][1] * normals(index, 1);
    }
    const Rows2 absolute_tangents = tangents.cwiseAbs(), absolute_normals = normals.cwiseAbs();
    std::priority_queue<Box, std::vector<Box>, Compare> queue;
    size_t serial = 0;
    double best = INFINITY, witness = 0., safe = INFINITY;
    bool have_witness = false;
    auto enqueue = [&](double lower, double upper, const Curve& controls) {
        const auto motion = window.bounds(start + lower * duration, start + upper * duration);
        const Vector low = controls.colwise().minCoeff().transpose() - motion.second - Vector::Constant(1e-10);
        const Vector high = controls.colwise().maxCoeff().transpose() - motion.first + Vector::Constant(1e-10);
        double planar = INFINITY;
        if (window.circle) {
            Eigen::Vector2d near, far;
            for (int axis = 0; axis < 2; ++axis) {
                near[axis] = std::max({low[axis], -high[axis], 0.});
                far[axis] = std::max(std::abs(low[axis]), std::abs(high[axis]));
            }
            const double frame_radius = window.opening_radius + .085;
            const double near_norm = std::sqrt(numpy_dot(near.data(), near.data(), 2));
            const double far_norm = std::sqrt(numpy_dot(far.data(), far.data(), 2));
            const double gap = std::max({near_norm - frame_radius, frame_radius - far_norm, 0.});
            planar = gap * gap;
        } else {
            const Eigen::Vector2d center = (low.head<2>() + high.head<2>()) / 2.;
            const Eigen::Vector2d half = (high.head<2>() - low.head<2>()) / 2.;
            std::vector<double> along(lengths.size()), along_radius(lengths.size()), across(lengths.size()), across_radius(lengths.size());
            auto multiply = [&](const Rows2& matrix, const Eigen::Vector2d& vector, std::vector<double>& result) {
                cblas_dgemv64_(101, 111, matrix.rows(), 2, 1., matrix.data(), 2, vector.data(), 1, 0., result.data(), 1);
            };
            multiply(tangents, center, along);
            multiply(absolute_tangents, half, along_radius);
            multiply(normals, center, across);
            multiply(absolute_normals, half, across_radius);
            for (size_t index = 0; index < lengths.size(); ++index) {
                const double ac = along[index] - along_origin[index];
                const double nc = across[index] - across_origin[index];
                const double ag = std::max({-ac - along_radius[index], ac - along_radius[index] - lengths[index], 0.});
                const double ng = std::max(std::abs(nc) - across_radius[index], 0.);
                planar = std::min(planar, ag * ag + ng * ng);
            }
        }
        for (double fraction : {lower, (lower + upper) / 2., upper}) {
            const double distance = numpy_frame_distance(window, position(fraction));
            if (distance < best) { best = distance; witness = fraction; have_witness = true; }
        }
        const double normal = std::max({low.z(), -high.z(), 0.});
        queue.push(Box{planar + normal * normal, lower, upper, ++serial, controls});
    };
    for (size_t index = 1; index < cuts.size(); ++index) {
        if (std::abs(evaluate(local, (cuts[index - 1] + cuts[index]) / 2.).z()) <= radius + 1e-9)
            enqueue(cuts[index - 1], cuts[index], restrict_interval(numpy_bernstein(local), cuts[index - 1], cuts[index]));
    }
    int boxes = 0;
    std::string outcome = "floating_local_bound";
    while (!queue.empty() && boxes < 512) {
        const Box item = queue.top();
        queue.pop();
        if (item.bound >= radius * radius) { safe = std::min(safe, item.bound); continue; }
        if (best < (radius - 2e-4) * (radius - 2e-4)) { outcome = "violated"; break; }
        if ((item.b - item.a) * duration < 1e-6) { outcome = "unresolved"; break; }
        const auto halves = split(item.control, .5);
        const double middle = (item.a + item.b) / 2.;
        enqueue(item.a, middle, halves.first);
        enqueue(middle, item.b, halves.second);
        ++boxes;
    }
    if (!queue.empty() && outcome == "floating_local_bound") outcome = "budget_unresolved";
    return Json{{"passed", outcome == "floating_local_bound"}, {"outcome", outcome},
        {"witness_u", have_witness ? Json(witness) : Json(nullptr)}, {"boxes", boxes},
        {"minimum_sample_distance_m", std::isfinite(best) ? Json(std::sqrt(best)) : Json(nullptr)},
        {"distance_lower_bound_m", outcome == "floating_local_bound" && std::isfinite(safe) ? Json(std::sqrt(safe)) : Json(nullptr)}};
}
