#include <casadi/casadi.hpp>
#include <libcmaes/cmaes.h>
#include <drolib/solver/minco_snap.hpp>
#include "geometry.hpp"
#include <chrono>
#include <filesystem>
#include <numeric>
#include <tuple>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

Json read_json(const fs::path& path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("Cannot read " + path.string());
    Json value;
    stream >> value;
    return value;
}

struct Quality {
    bool finite = false;
    bool feasible = false;
    double maximum = 1e100;
    double mean_square = 1e100;
    double duration = 1e100;
    int violated = 0;
    int physical_violated = 0;
    double physical_maximum = 0.;
    auto key() const {
        return feasible ? std::make_tuple(0, duration, 0., 0.)
                        : std::make_tuple(1, maximum, mean_square, duration);
    }
    Json json() const {
        return Json{{"finite", finite}, {"sampled_feasible", feasible}, {"maximum_violation", maximum},
                    {"mean_square_violation", mean_square}, {"trajectory_time_s", duration},
                    {"violated_rows", violated}, {"physical_violated_rows", physical_violated},
                    {"physical_maximum_violation", physical_maximum}};
    }
};

struct Candidate {
    std::vector<double> log_times;
    std::vector<double> times;
    std::vector<double> arrivals;
    std::vector<double> derivatives;
    std::vector<double> coefficients;
    Quality quality;
    Json json() const {
        return Json{{"log_times", log_times}, {"durations", times}, {"arrivals", arrivals},
                    {"VAJ", derivatives}, {"coefficients_flat", coefficients}, {"quality", quality.json()}};
    }
};

class Model {
    Json metadata_;
    casadi::Function constraints_;
    std::vector<Window> windows_;
    drolib::MincoSnap minco_;
    std::vector<double> lower_, upper_;
    int n_, m_, physical_rows_;
    double tolerance_;
public:
    explicit Model(const fs::path& input)
        : metadata_(read_json(input / "model.json")),
          constraints_(casadi::Function::load((input / "constraints.casadi").string())) {
        n_ = metadata_["n"];
        m_ = metadata_["m"];
        physical_rows_ = metadata_["physical_rows"];
        tolerance_ = metadata_["tolerance"];
        lower_ = metadata_["lower"].get<std::vector<double>>();
        upper_ = metadata_["upper"].get<std::vector<double>>();
        for (const auto& window : metadata_["windows"]) windows_.push_back(load_window(window));
        Eigen::Matrix<double,3,4> head = Eigen::Matrix<double,3,4>::Zero();
        Eigen::Matrix<double,3,4> tail = head;
        for (int axis = 0; axis < 3; ++axis) {
            head(axis,0) = metadata_["start"][axis];
            tail(axis,0) = metadata_["goal"][axis];
        }
        minco_.setConditions(head, tail, n_);
    }
    const Json& metadata() const { return metadata_; }
    Candidate evaluate(const std::vector<double>& log_times) {
        Candidate result;
        result.log_times = log_times;
        result.times.resize(n_);
        result.arrivals.resize(m_);
        Eigen::VectorXd times(n_);
        double bound_violation = 0.;
        double arrival = 0.;
        Eigen::Matrix3Xd points(3,m_);
        for (int piece = 0; piece < n_; ++piece) {
            const double duration = std::exp(log_times[piece]);
            if (!std::isfinite(duration) || duration < 1e-8) return result;
            result.times[piece] = times[piece] = duration;
            bound_violation += std::max(lower_[piece]-duration, 0.) + std::max(duration-upper_[piece], 0.);
            arrival += duration;
            if (piece < m_) {
                result.arrivals[piece] = arrival;
                points.col(piece) = windows_[piece].center + windows_[piece].rotation*windows_[piece].motion(arrival);
            }
        }
        result.quality.duration = arrival;
        if (bound_violation > 1e-10) {
            result.quality.maximum = 1e6 + bound_violation;
            return result;
        }
        minco_.setParameters(points, times);
        const auto& raw = minco_.getCoeffs();
        if (!raw.allFinite()) return result;
        result.coefficients.resize(24*n_);
        for (int piece = 0; piece < n_; ++piece)
            for (int axis = 0; axis < 3; ++axis)
                for (int power = 0; power < 8; ++power)
                    result.coefficients[24*piece+8*axis+power] = raw(8*piece+power,axis)*std::pow(times[piece],power);
        const auto residuals = constraints_(std::vector<casadi::DM>{casadi::DM(result.coefficients),casadi::DM(result.times)})[0].get_elements();
        result.quality.maximum = 0.;
        result.quality.mean_square = 0.;
        for (size_t row = 0; row < residuals.size(); ++row) {
            if (!std::isfinite(residuals[row])) return Candidate{};
            const double violation = std::max(-residuals[row],0.);
            result.quality.maximum = std::max(result.quality.maximum,violation);
            result.quality.mean_square += violation*violation;
            if (violation > tolerance_) {
                ++result.quality.violated;
                if (static_cast<int>(row) < physical_rows_) ++result.quality.physical_violated;
            }
            if (static_cast<int>(row) < physical_rows_)
                result.quality.physical_maximum = std::max(result.quality.physical_maximum,violation);
        }
        result.quality.mean_square /= residuals.size();
        result.quality.finite = true;
        result.quality.feasible = result.quality.maximum <= tolerance_;
        result.derivatives.resize(9*m_);
        for (int order = 1; order <= 3; ++order)
            for (int gate = 0; gate < m_; ++gate)
                for (int axis = 0; axis < 3; ++axis) {
                    double derivative = 0.;
                    for (int power = 7; power >= order; --power) {
                        double factor = 1.;
                        for (int k = 0; k < order; ++k) factor *= power-k;
                        derivative = derivative*times[gate]+factor*raw(8*gate+power,axis);
                    }
                    result.derivatives[3*((order-1)*m_+gate)+axis] = derivative;
                }
        return result;
    }
};

class RankedStrategy : public libcmaes::CMAStrategy<libcmaes::CovarianceUpdate> {
public:
    RankedStrategy(libcmaes::FitFunc& function,libcmaes::CMAParameters<>& parameters)
        : libcmaes::CMAStrategy<libcmaes::CovarianceUpdate>(function,parameters) {}
    void rank(const Eigen::MatrixXd& population,const std::vector<Quality>& qualities) {
        std::vector<int> order(population.cols());
        std::iota(order.begin(),order.end(),0);
        std::stable_sort(order.begin(),order.end(),[&](int a,int b){return qualities[a].key()<qualities[b].key();});
        for (size_t position = 0; position < order.size(); ++position) {
            const int index = order[position];
            _solutions.get_candidate(index).set_x(population.col(index));
            _solutions.get_candidate(index).set_fvalue(static_cast<double>(position));
        }
        update_fevals(population.cols());
    }
};

int unit_test() {
    std::vector<double> origin(8,1.5);
    libcmaes::CMAParameters<> parameters(origin,.3,24,12);
    parameters.set_quiet(true);
    libcmaes::FitFunc dummy = [](const double*,int){return 0.;};
    libcmaes::ESOptimizer<RankedStrategy,libcmaes::CMAParameters<>> optimizer(dummy,parameters);
    double best = 1e100;
    for (int generation = 0; generation < 180; ++generation) {
        const auto population = optimizer.ask();
        std::vector<Quality> qualities(population.cols());
        for (int col = 0; col < population.cols(); ++col) {
            double cost = 0.;
            for (int row = 0; row < population.rows(); ++row) {
                const double a = population(row,col);
                const double b = population((row+1)%population.rows(),col);
                cost += std::pow(1.7*a+.3*b,2)*(row+1);
            }
            best = std::min(best,cost);
            qualities[col].feasible = true;
            qualities[col].duration = cost;
        }
        optimizer.rank(population,qualities);
        optimizer.tell();
        optimizer.inc_iter();
    }
    std::cout << Json{{"passed",best<1e-8},{"best_rotated_quadratic",best}} << '\n';
    return best<1e-8 ? 0 : 1;
}

int main(int argc,char** argv) {
    try {
        if (argc==2 && std::string(argv[1])=="--unit") return unit_test();
        if (argc!=6) throw std::runtime_error("Usage: generate_initials MODEL_DIR SEED GENERATIONS POPULATION OUTPUT");
        const auto begin = Clock::now();
        Model model(argv[1]);
        const int seed = std::stoi(argv[2]);
        const int generations = std::stoi(argv[3]);
        const int population_size = std::stoi(argv[4]);
        const int n = model.metadata()["n"];
        const auto original = model.metadata()["original_initials"][seed]["original"].get<std::vector<double>>();
        std::vector<double> origin(original.begin(),original.begin()+n);
        Candidate initial = model.evaluate(origin);
        Candidate best = initial;
        std::vector<Candidate> archive{initial};
        double coefficient_error = 0.;
        const int vs = model.metadata()["velocity_start"];
        if (initial.derivatives.size()!=original.size()-vs) throw std::runtime_error("Initial derivative dimension mismatch");
        for (size_t k = 0; k < initial.derivatives.size(); ++k)
            coefficient_error = std::max(coefficient_error,std::abs(initial.derivatives[k]-original[vs+k]));
        if (coefficient_error>1e-6) throw std::runtime_error("Original physical initial curve differs");
        libcmaes::CMAParameters<> parameters(origin,.12,population_size,20261113+seed);
        parameters.set_quiet(true);
        libcmaes::FitFunc dummy = [](const double*,int){return 0.;};
        libcmaes::ESOptimizer<RankedStrategy,libcmaes::CMAParameters<>> optimizer(dummy,parameters);
        const fs::path output = argv[5];
        fs::create_directories(output.parent_path());
        std::ofstream progress(output.parent_path()/("generation_"+std::to_string(seed)+".jsonl"));
        int feasible_count = initial.quality.feasible;
        int evaluations = 1;
        for (int generation = 0; generation < generations; ++generation) {
            const auto population = optimizer.ask();
            std::vector<Quality> qualities;
            std::vector<Candidate> candidates;
            for (int col = 0; col < population.cols(); ++col) {
                std::vector<double> point(population.col(col).data(),population.col(col).data()+n);
                Candidate candidate = model.evaluate(point);
                feasible_count += candidate.quality.feasible;
                if (candidate.quality.key()<best.quality.key()) best = candidate;
                qualities.push_back(candidate.quality);
                candidates.push_back(std::move(candidate));
                ++evaluations;
            }
            optimizer.rank(population,qualities);
            optimizer.tell();
            optimizer.inc_iter();
            const auto winner = std::min_element(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b){return a.quality.key()<b.quality.key();});
            if (winner->quality.finite) archive.push_back(*winner);
            progress << Json{{"generation",generation},{"evaluations",evaluations},{"best",best.quality.json()},
                             {"wall_s",std::chrono::duration<double>(Clock::now()-begin).count()}} << '\n';
            progress.flush();
        }
        Json archived = Json::array();
        for (const auto& candidate : archive) archived.push_back(candidate.json());
        Json result{{"seed",seed},{"generations",generations},{"population",population_size},{"sigma_log_time",.12},
                    {"search_dimension",n},{"evaluations",evaluations},{"feasible_evaluations",feasible_count},
                    {"initial",initial.json()},{"best",best.json()},{"archive",archived},
                    {"initial_VAJ_error",coefficient_error},
                    {"compute_wall_s",std::chrono::duration<double>(Clock::now()-begin).count()},
                    {"rank_rule","sampled feasibility first; feasible total time; infeasible maximum then mean-square violation"},
                    {"libcmaes_commit","6a53cd562c85dc6df3b8df317be3ecb415240c2d"}};
        std::ofstream(output) << result.dump(2) << '\n';
        std::cout << Json{{"seed",seed},{"initial",initial.quality.json()},{"best",best.quality.json()},
                         {"compute_wall_s",result["compute_wall_s"]}} << '\n';
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
