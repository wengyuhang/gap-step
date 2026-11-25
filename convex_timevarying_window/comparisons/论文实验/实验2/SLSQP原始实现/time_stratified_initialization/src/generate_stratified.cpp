#include "initial_model.hpp"

std::vector<double> log_durations(const std::vector<double>& allocation,double total,
                                  const std::vector<double>& lower) {
    if (allocation.size()+1!=lower.size()) throw std::runtime_error("Wrong allocation dimension");
    const double reserve=std::accumulate(lower.begin(),lower.end(),0.);
    if (!(total>reserve)) throw std::runtime_error("Total shorter than original segment lower bounds");
    const double maximum=std::max(0.,*std::max_element(allocation.begin(),allocation.end()));
    std::vector<double> weights(lower.size());
    double sum=0.;
    for (size_t k=0;k<weights.size();++k) {
        weights[k]=std::exp((k<allocation.size()?allocation[k]:0.)-maximum);
        sum+=weights[k];
    }
    std::vector<double> result(lower.size());
    for (size_t k=0;k<result.size();++k)
        result[k]=std::log(lower[k]+(total-reserve)*weights[k]/sum);
    return result;
}

class AllocationStrategy : public libcmaes::CMAStrategy<libcmaes::CovarianceUpdate> {
public:
    AllocationStrategy(libcmaes::FitFunc& function,libcmaes::CMAParameters<>& parameters)
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
    const std::vector<double> lower(8,.3);
    bool passed=true;
    for (const std::vector<double>& allocation : {std::vector<double>(7,0.),std::vector<double>{1000.,-1000.,1.,-1.,2.,-2.,0.}}) {
        auto logs=log_durations(allocation,16.,lower);
        double sum=0.;
        for (size_t i=0;i<logs.size();++i) {
            const double duration=std::exp(logs[i]);sum+=duration;
            passed=passed && duration>=lower[i]-1e-12 && std::isfinite(duration);
        }
        passed=passed && std::abs(sum-16.)<1e-10;
    }
    std::cout<<Json{{"passed",passed},{"total_time",16.},{"allocation_dimensions",7}}<<'\n';
    return passed?0:1;
}

int main(int argc,char** argv) {
    try {
        if (argc==2 && std::string(argv[1])=="--unit") return unit_test();
        if (argc!=6) throw std::runtime_error("Usage: generate_stratified MODEL_DIR SEED GENERATIONS POPULATION OUTPUT");
        const auto begin=Clock::now();
        Model model(argv[1]);
        const int seed=std::stoi(argv[2]),generations=std::stoi(argv[3]),population_size=std::stoi(argv[4]);
        const int n=model.metadata()["n"];
        auto lower=model.metadata()["lower"].get<std::vector<double>>();
        std::vector<double> references;
        for (const auto& item:model.metadata()["original_initials"]) {
            double sum=0.;for(int k=0;k<n;++k)sum+=std::exp(item["original"][k].get<double>());
            references.push_back(sum);
        }
        std::sort(references.begin(),references.end());
        const double reference=.5*(references[(references.size()-1)/2]+references[references.size()/2]);
        const std::array<double,4> fractions={.65,.70,.75,.80};
        const int time_band=seed/4;
        const double target=reference*fractions[time_band];
        const auto original=model.metadata()["original_initials"][seed]["original"].get<std::vector<double>>();
        std::vector<double> allocation(n-1);
        const double last=std::exp(original[n-1])-lower[n-1];
        for(int k=0;k<n-1;++k)allocation[k]=std::log((std::exp(original[k])-lower[k])/last);
        Candidate initial=model.evaluate(log_durations(allocation,target,lower));
        Candidate best=initial;
        std::vector<Candidate> archive{initial};
        libcmaes::CMAParameters<> parameters(allocation,.25,population_size,20261113+seed);
        parameters.set_quiet(true);
        libcmaes::FitFunc dummy=[](const double*,int){return 0.;};
        libcmaes::ESOptimizer<AllocationStrategy,libcmaes::CMAParameters<>> optimizer(dummy,parameters);
        const fs::path output=argv[5];fs::create_directories(output.parent_path());
        std::ofstream progress(output.parent_path()/("generation_"+std::to_string(seed)+".jsonl"));
        int evaluations=1;
        for(int generation=0;generation<generations;++generation) {
            const auto population=optimizer.ask();
            std::vector<double> scores;
            Candidate winner;
            for(int col=0;col<population.cols();++col) {
                std::vector<double> x(population.col(col).data(),population.col(col).data()+n-1);
                Candidate candidate=model.evaluate(log_durations(x,target,lower));
                scores.push_back(candidate.quality.maximum);
                if(candidate.quality.maximum<best.quality.maximum)best=candidate;
                if(candidate.quality.maximum<winner.quality.maximum)winner=candidate;
                ++evaluations;
            }
            optimizer.evaluate_scores(population,scores);
            optimizer.tell();optimizer.inc_iter();
            if(winner.quality.finite)archive.push_back(std::move(winner));
            progress<<Json{{"generation",generation},{"evaluations",evaluations},{"best",best.quality.json()},
                           {"target_time_s",target},{"wall_s",std::chrono::duration<double>(Clock::now()-begin).count()}}<<'\n';
        }
        Json saved=Json::array();
        for(const auto& candidate:archive)saved.push_back(candidate.json());
        Json result{{"seed",seed},{"time_band",time_band},{"target_time_s",target},{"target_fraction",fractions[time_band]},
                    {"reference_time_s",reference},{"search_dimensions",n-1},{"generations",generations},
                    {"population",population_size},{"evaluations",evaluations},{"initial",initial.json()},
                    {"best",best.json()},{"archive",saved},{"objective","minimum maximum normalized violation at fixed total initial duration"},
                    {"compute_wall_s",std::chrono::duration<double>(Clock::now()-begin).count()}};
        std::ofstream(output)<<result.dump(2)<<'\n';
        std::cout<<Json{{"seed",seed},{"band",time_band},{"best",best.quality.json()},{"wall_s",result["compute_wall_s"]}}<<'\n';
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
