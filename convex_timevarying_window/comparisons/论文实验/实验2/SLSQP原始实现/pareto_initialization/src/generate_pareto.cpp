#include "initial_model.hpp"

bool dominates(const Candidate& a,const Candidate& b) {
    const auto& x=a.quality;
    const auto& y=b.quality;
    return x.duration<=y.duration && x.maximum<=y.maximum &&
           (x.duration<y.duration || x.maximum<y.maximum);
}

void retain_pareto(std::vector<Candidate>& front,const Candidate& candidate) {
    if (!candidate.quality.finite) return;
    for (const auto& old:front) {
        if (dominates(old,candidate)) return;
        if (std::abs(old.quality.duration-candidate.quality.duration)<1e-10 &&
            std::abs(old.quality.maximum-candidate.quality.maximum)<1e-12) return;
    }
    front.erase(std::remove_if(front.begin(),front.end(),[&](const Candidate& old){return dominates(candidate,old);}),front.end());
    front.push_back(candidate);
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
    Candidate fast,slow,dominated;
    fast.quality.finite=slow.quality.finite=dominated.quality.finite=true;
    fast.quality.duration=18.;fast.quality.maximum=.02;
    slow.quality.duration=30.;slow.quality.maximum=0.;
    dominated.quality.duration=22.;dominated.quality.maximum=.03;
    std::vector<Candidate> front;
    retain_pareto(front,fast);retain_pareto(front,slow);retain_pareto(front,dominated);
    const bool tradeoff=score(fast.quality,20.,2.)<score(slow.quality,20.,2.);
    const bool passed=front.size()==2 && tradeoff;
    std::cout<<Json{{"passed",passed},{"pareto_size",front.size()},{"short_low_violation_preferred",tradeoff}}<<'\n';
    return passed?0:1;
}

int main(int argc,char** argv) {
    try {
        if (argc==2 && std::string(argv[1])=="--unit") return unit_test();
        if (argc!=6) throw std::runtime_error("Usage: generate_pareto MODEL_DIR SEED GENERATIONS POPULATION OUTPUT");
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
        const std::array<double,4> weights={1.,2.,4.,8.};
        const double weight=weights[(seed/4)%4];
        Candidate initial=model.evaluate(origin),best=initial;
        const int vs=model.metadata()["velocity_start"];
        double error=0.;
        for (size_t k=0;k<initial.derivatives.size();++k)error=std::max(error,std::abs(initial.derivatives[k]-original[vs+k]));
        if (initial.derivatives.size()!=original.size()-vs || error>1e-6)throw std::runtime_error("Initial MINCO curve changed");
        std::vector<Candidate> front;
        retain_pareto(front,initial);
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
                retain_pareto(front,candidate);
                ++evaluations;
            }
            optimizer.evaluate_scores(population,scores);
            optimizer.tell();optimizer.inc_iter();
            progress<<Json{{"generation",generation},{"evaluations",evaluations},{"best",best.quality.json()},
                           {"pareto_size",front.size()},{"weight",weight},{"wall_s",std::chrono::duration<double>(Clock::now()-begin).count()}}<<'\n';
        }
        Json archive=Json::array();
        for (const auto& candidate:front)archive.push_back(candidate.json());
        Json result{{"seed",seed},{"generations",generations},{"population",population_size},{"evaluations",evaluations},
                    {"weight",weight},{"reference_time_s",reference},{"initial",initial.json()},{"best",best.json()},
                    {"archive",archive},{"initial_VAJ_error",error},{"search_dimensions",n},
                    {"objective","T/reference_time + weight * maximum_original_normalized_constraint_violation"},
                    {"compute_wall_s",std::chrono::duration<double>(Clock::now()-begin).count()}};
        std::ofstream(output)<<result.dump(2)<<'\n';
        std::cout<<Json{{"seed",seed},{"weight",weight},{"best",best.quality.json()},{"pareto_size",front.size()},{"wall_s",result["compute_wall_s"]}}<<'\n';
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n';return 1;
    }
    return 0;
}
