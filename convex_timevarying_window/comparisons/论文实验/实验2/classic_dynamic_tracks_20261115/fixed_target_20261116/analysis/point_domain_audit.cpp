#include "trajectory.hpp"
#include "planar_points.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <algorithm>
namespace fs = std::filesystem;
Json load(const fs::path &file) {
    std::ifstream stream(file);
    if (!stream) throw std::runtime_error("Missing " + file.string());
    Json value;
    stream >> value;
    return value;
}
void require(bool value, const std::string &message) {
    if (!value) throw std::runtime_error(message);
}
Json statistics(const Json &points) {
    std::vector<double> distances, fractions;
    int nearBoundary = 0;
    double normalError = 0.;
    for (const auto &point : points) {
        distances.push_back(point.at("distance_to_center_m"));
        fractions.push_back(point.at("boundary_fraction"));
        if (fractions.back() >= .95) ++nearBoundary;
        normalError = std::max(normalError, std::abs(point.at("plane_error_m").get<double>()));
    }
    if (distances.empty()) return Json{{"count", 0}};
    std::sort(distances.begin(), distances.end());
    std::sort(fractions.begin(), fractions.end());
    auto median = [](const std::vector<double> &values) {
        const size_t middle = values.size()/2;
        return values.size()%2 ? values[middle] : .5*(values[middle-1]+values[middle]);
    };
    return {{"count", distances.size()}, {"mean_distance_m", std::accumulate(distances.begin(), distances.end(), 0.)/distances.size()},
        {"median_distance_m", median(distances)}, {"maximum_distance_m", distances.back()},
        {"mean_boundary_fraction", std::accumulate(fractions.begin(), fractions.end(), 0.)/fractions.size()},
        {"median_boundary_fraction", median(fractions)}, {"maximum_boundary_fraction", fractions.back()},
        {"points_at_least_95_percent_to_boundary", nearBoundary}, {"maximum_plane_error_m", normalError}};
}
Json inspect(const Json &manifest, const Json &raw, const Json &finite, const std::string &level, const std::string &candidate, bool selected) {
    const int count = manifest.at("m"), pieces = manifest.at("n");
    const double erosion = planar_points::erosion_radius(manifest);
    const auto trajectory = Trajectory::load(raw);
    Json points = Json::array();
    double at = 0.;
    for (int gate = 0; gate < count; ++gate) {
        at += trajectory.durations.at(gate);
        const auto window = load_window(manifest.at("windows").at(gate));
        const auto domain = planar_points::domain(window, erosion);
        const Vector local = window.rotation.transpose()*(trajectory.sample(at)-window.center)-window.motion(at);
        const Eigen::Vector2d point = local.head<2>();
        const double fraction = window.circle ? point.norm()/domain.safe_radius :
            std::max(std::abs(point.x())/domain.upper.x(), std::abs(point.y())/domain.upper.y());
        const double physicalArea = window.circle ? pi*window.opening_radius*window.opening_radius :
            4.*domain.scale.x()*domain.scale.y();
        const double allowedArea = window.circle ? pi*domain.safe_radius*domain.safe_radius :
            (domain.upper.x()-domain.lower.x())*(domain.upper.y()-domain.lower.y());
        const auto &slice = manifest.at("slices").at(gate);
        const int begin = slice.at("begin");
        const auto &variables = raw.at("variables");
        require(std::abs(variables.at(pieces+2*gate).get<double>()-point.x()) < 1e-8 &&
            std::abs(variables.at(pieces+2*gate+1).get<double>()-point.y()) < 1e-8, "Trajectory and physical crossing coordinates differ");
        for (int axis = 0; axis < 2; ++axis) {
            require(std::abs(finite.at("lower_x").at(begin+axis).get<double>()*domain.scale[axis]-domain.lower[axis]) < 1e-12, "Solver lower bound is not point-domain bound");
            require(std::abs(finite.at("upper_x").at(begin+axis).get<double>()*domain.scale[axis]-domain.upper[axis]) < 1e-12, "Solver upper bound is not point-domain bound");
        }
        require(fraction <= 1.+1e-7, "Physical point outside allowed domain");
        require(std::abs(local.z()) < 1e-8, "Point off moving gate plane");
        points.push_back({{"level",level},{"candidate",candidate},{"selected",selected},{"gate",gate+1},
            {"shape",window.circle?"circle":"square"},{"moving",window.amplitude!=0. && window.frequency!=0.},
            {"arrival_s",at},{"qx_m",point.x()},{"qy_m",point.y()},{"plane_error_m",local.z()},
            {"distance_to_center_m",point.norm()},{"boundary_fraction",fraction},
            {"lower_m",{domain.lower.x(),domain.lower.y()}},{"upper_m",{domain.upper.x(),domain.upper.y()}},
            {"physical_half_size_m",{domain.scale.x(),domain.scale.y()}},
            {"allowed_radius_m",window.circle?Json(domain.safe_radius):Json(nullptr)},
            {"allowed_area_m2",allowedArea},{"physical_area_m2",physicalArea},{"area_fraction",allowedArea/physicalArea},
            {"point_erosion_radius_m",erosion},{"centerline_threshold_m",manifest.at("finite_radius")},
            {"frame_tube_radius_m",planar_points::frame_tube_radius(manifest)}});
    }
    return points;
}
int main(int argc, char **argv) {
    try {
        if (argc != 3) throw std::runtime_error("Usage: SIX_COURSE_RESULTS OUTPUT_DIRECTORY");
        const fs::path input = fs::absolute(argv[1]), output = fs::absolute(argv[2]);
        if (fs::exists(output)) throw std::runtime_error("Output exists");
        fs::create_directories(output);
        const auto summary = load(input / "SUMMARY.json");
        Json selectedPoints = Json::array(), allAccepted = Json::array(), courses = Json::array();
        double maxCoordinateError = 0.;
        for (const auto &condition : summary.at("conditions")) {
            const std::string level = condition.at("level");
            const auto &method = condition.at("methods").at("ours");
            if (!method.at("found").get<bool>()) continue;
            const std::string chosen = method.at("selected");
            Json coursePoints = Json::array();
            for (const auto &candidate : method.at("candidates")) {
                if (!candidate.at("accepted").get<bool>()) continue;
                const std::string name = candidate.at("name");
                const fs::path folder = candidate.at("directory").get<std::string>();
                const auto manifest = load(input / level / "prepared" / name / "manifest.json");
                planar_points::validate(manifest);
                const auto points = inspect(manifest, load(folder / "result.json"), load(folder / "finite_input.json"), level, name, name==chosen);
                for (const auto &point : points) {
                    allAccepted.push_back(point);
                    if (name==chosen) {
                        selectedPoints.push_back(point);
                        coursePoints.push_back(point);
                    }
                }
            }
            Json square = Json::array(), circle = Json::array(), moving = Json::array(), stationary = Json::array();
            for (const auto &point : coursePoints) {
                (point.at("shape")=="square" ? square : circle).push_back(point);
                (point.at("moving").get<bool>() ? moving : stationary).push_back(point);
            }
            courses.push_back({{"level",level},{"selected_candidate",chosen},{"all",statistics(coursePoints)},
                {"square",statistics(square)},{"circle",statistics(circle)},{"moving",statistics(moving)},{"static",statistics(stationary)}});
        }
        Json squares = Json::array(), circles = Json::array(), moving = Json::array();
        for (const auto &point : selectedPoints) {
            (point.at("shape")=="square" ? squares : circles).push_back(point);
            if (point.at("moving").get<bool>()) moving.push_back(point);
        }
        Json report{{"selected",selectedPoints},{"accepted_candidate_points",allAccepted},
            {"selected_statistics",statistics(selectedPoints)},{"selected_square_statistics",statistics(squares)},
            {"selected_circle_statistics",statistics(circles)},{"selected_moving_statistics",statistics(moving)},
            {"all_accepted_statistics",statistics(allAccepted)},{"courses",courses},
            {"bounds_match_solver_input",true},{"physical_points_match_trajectory",true},
            {"frame_radius_counted_once",true}};
        std::ofstream(output / "AUDIT.json") << report.dump(2) << '\n';
        std::ofstream table(output / "POINTS.csv");
        table << "level,candidate,gate,shape,moving,arrival_s,qx_m,qy_m,distance_m,boundary_fraction,lower_x_m,upper_x_m,lower_y_m,upper_y_m,allowed_area_m2,physical_area_m2,area_fraction\n";
        table << std::setprecision(15);
        for (const auto &point : selectedPoints)
            table << point.at("level").get<std::string>() << ',' << point.at("candidate").get<std::string>() << ','
                << point.at("gate") << ',' << point.at("shape").get<std::string>() << ',' << point.at("moving").get<bool>() << ','
                << point.at("arrival_s") << ',' << point.at("qx_m") << ',' << point.at("qy_m") << ',' << point.at("distance_to_center_m") << ','
                << point.at("boundary_fraction") << ',' << point.at("lower_m").at(0) << ',' << point.at("upper_m").at(0) << ','
                << point.at("lower_m").at(1) << ',' << point.at("upper_m").at(1) << ',' << point.at("allowed_area_m2") << ','
                << point.at("physical_area_m2") << ',' << point.at("area_fraction") << '\n';
        std::cout << "Selected trajectory points " << selectedPoints.size() << ", all accepted candidate points " << allAccepted.size() << '\n';
        std::cout << report.at("selected_statistics").dump(2) << '\n';
    } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
    return 0;
}
