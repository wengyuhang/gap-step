#include "initial_model.hpp"
#include <map>
#include <random>
#include <sstream>
#include <set>

std::string timing_branch(const Candidate& candidate,const Json& windows) {
    std::ostringstream key;
    for (size_t gate=0;gate<windows.size();++gate) {
        const double frequency=windows[gate]["frequency"];
        const double amplitude=windows[gate]["amplitude"];
        if (frequency<=0. || amplitude<=0.) {key<<"static,";continue;}
        const double phase=2*pi*frequency*candidate.arrivals[gate]+windows[gate]["phase"].get<double>();
        key<<static_cast<long>(std::floor((phase-pi/2)/pi))<<',';
    }
    return key.str();
}

double score(const Quality& quality,double reference,double weight) {
    return quality.duration/reference+weight*quality.maximum;
}

class ScalarStrategy : public libcmaes::CMAStrategy<libcmaes::CovarianceUpdate> {
public:
    ScalarStrategy(libcmaes::FitFunc& function,libcmaes::CMAParameters<>& parameters)
        :libcmaes::CMAStrategy<libcmaes::CovarianceUpdate>(function,parameters) {}
    void evaluate_scores(const Eigen::MatrixXd& population,const std::vector<double>& scores) {
        for (int col=0;col<population.cols();++col) {
            _solutions.get_candidate(col).set_x(population.col(col));
            _solutions.get_candidate(col).set_fvalue(scores[col]);
        }
        update_fevals(population.cols());
    }
};

int unit_test() {
    Candidate candidate;
    candidate.arrivals={.1};
    Json windows=Json::array({Json{{"frequency",1.},{"amplitude",1.},{"phase",0.}}});
    const auto first=timing_branch(candidate,windows);
    candidate.arrivals={.6};
    const auto second=timing_branch(candidate,windows);
    candidate.quality.duration=18.;candidate.quality.maximum=.02;
    const bool passed=first!=second && score(candidate.quality,20.,1.)<30./20.;
    std::cout<<Json{{"passed",passed},{"distinct_motion_half_cycles",first!=second}}<<'\n';
    return passed?0:1;
}

int main(int argc,char** argv) {
    try {
        if (argc==2 && std::string(argv[1])=="--unit") return unit_test();
        if (argc!=6) throw std::runtime_error("Usage: generate_multimodal MODEL_DIR SEED GENERATIONS POPULATION OUTPUT");
        const auto begin=Clock::now();
        Model model(argv[1]);
        const int seed=std::stoi(argv[2]);
        const int generations=std::stoi(argv[3]);
        const int population_size=std::stoi(argv[4]);
        const int n=model.metadata()["n"];
        const auto original=model.metadata()["original_initials"][seed]["original"].get<std::vector<double>>();
        std::vector<double> origin(original.begin(),original.begin()+n);
        std::vector<double> reference_times;
        for (const auto& initial:model.metadata()["original_initials"]) {
            double sum=0.;
            for (int k=0;k<n;++k)sum+=std::exp(initial["original"][k].get<double>());
            reference_times.push_back(sum);
        }
        std::sort(reference_times.begin(),reference_times.end());
        const double reference=.5*(reference_times[(reference_times.size()-1)/2]+reference_times[reference_times.size()/2]);
        const double weight=1.;
        if (seed>=8) {
            std::mt19937 generator(20261113+seed);
            std::uniform_real_distribution<double> factor(.65,.85);
            for (int piece=0;piece<n;++piece)origin[piece]+=std::log(factor(generator));
        }
        Candidate initial=model.evaluate(origin),best=initial;
        const int vs=model.metadata()["velocity_start"];
        double error=0.;
        for (size_t k=0;k<initial.derivatives.size();++k)error=std::max(error,std::abs(initial.derivatives[k]-original[vs+k]));
        if (seed<8 && (initial.derivatives.size()!=original.size()-vs || error>1e-6))throw std::runtime_error("Initial MINCO curve changed");
        std::map<std::string,Candidate> branches;
        auto retain=[&](const Candidate& candidate) {
            if (!candidate.quality.finite) return;
            const auto key=timing_branch(candidate,model.metadata()["windows"]);
            const auto found=branches.find(key);
            if (found==branches.end() || score(candidate.quality,reference,weight)<score(found->second.quality,reference,weight))branches[key]=candidate;
        };
        retain(initial);
        libcmaes::CMAParameters<> parameters(origin,.12,population_size,20261113+seed);
        parameters.set_quiet(true);
        libcmaes::FitFunc dummy=[](const double*,int){return 0.;};
        libcmaes::ESOptimizer<ScalarStrategy,libcmaes::CMAParameters<>> optimizer(dummy,parameters);
        const fs::path output=argv[5];fs::create_directories(output.parent_path());
        std::ofstream progress(output.parent_path()/("generation_"+std::to_string(seed)+".jsonl"));
        int evaluations=1;
        for (int generation=0;generation<generations;++generation) {
            const auto population=optimizer.ask();
            std::vector<double> scores;
            for (int col=0;col<population.cols();++col) {
                std::vector<double> x(population.col(col).data(),population.col(col).data()+n);
                Candidate candidate=model.evaluate(x);
                const double value=score(candidate.quality,reference,weight);
                scores.push_back(value);
                if (value<score(best.quality,reference,weight))best=candidate;
                retain(candidate);
                ++evaluations;
            }
            optimizer.evaluate_scores(population,scores);
            optimizer.tell();optimizer.inc_iter();
            progress<<Json{{"generation",generation},{"evaluations",evaluations},{"best",best.quality.json()},
                           {"branch_count",branches.size()},{"weight",weight},{"wall_s",std::chrono::duration<double>(Clock::now()-begin).count()}}<<'\n';
        }
        Json archive=Json::array();
        std::vector<std::pair<std::string,Candidate>> choices(branches.begin(),branches.end());
        std::set<std::string> included;
        for (int direction=0;direction<8;++direction) {
            const double preference=std::pow(4.,-1.+direction/7.);
            std::sort(choices.begin(),choices.end(),[&](const auto& a,const auto& b){return score(a.second.quality,reference,preference)<score(b.second.quality,reference,preference);});
            for (size_t index=0;index<std::min(size_t(8),choices.size());++index) {
                const auto& item=choices[index];
                if (!included.insert(item.first).second) continue;
                Json value=item.second.json();value["timing_branch"]=item.first;archive.push_back(value);
            }
        }
        Json result{{"seed",seed},{"generations",generations},{"population",population_size},{"evaluations",evaluations},
                    {"weight",weight},{"reference_time_s",reference},{"initial",initial.json()},{"best",best.json()},
                    {"archive",archive},{"initial_VAJ_error",seed<8?Json(error):Json(nullptr)},{"search_dimensions",n},{"initial_duration_family",seed<8?"original independent phase samples":"independently shortened segment samples"},
                    {"objective","T/reference_time + maximum_original_normalized_constraint_violation"},
                    {"compute_wall_s",std::chrono::duration<double>(Clock::now()-begin).count()}};
        std::ofstream(output)<<result.dump(2)<<'\n';
        std::cout<<Json{{"seed",seed},{"weight",weight},{"best",best.quality.json()},{"branch_count",branches.size()},{"wall_s",result["compute_wall_s"]}}<<'\n';
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n';return 1;
    }
    return 0;
}
