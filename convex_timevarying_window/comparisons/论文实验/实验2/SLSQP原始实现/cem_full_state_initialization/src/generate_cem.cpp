#include "full_state_model.hpp"
#include <omp.h>
#include <random>

// Scales affect CEM coordinate conditioning only; they are not constraints.
double scale(int k,int n,int m) { return k<n?1.:(k<n+3*m?5.:(k<n+6*m?10.:20.)); }
std::vector<double> decode(const std::vector<double>& z,int n,int m) {
    auto x=z; for(size_t k=0;k<x.size();++k)x[k]*=scale(k,n,m);return x;
}
bool better(const Candidate& a,const Candidate& b,double eps) {
    if(a.quality.finite!=b.quality.finite)return a.quality.finite;
    if(!a.quality.finite)return false;
    bool af=a.quality.maximum<=eps,bf=b.quality.maximum<=eps;
    if(af!=bf)return af;
    if(af)return a.quality.duration<b.quality.duration;
    return std::tie(a.quality.maximum,a.quality.duration)<std::tie(b.quality.maximum,b.quality.duration);
}
double endpoint(const Candidate& c,int piece,int axis,int order,double u) {
    double v=0.;for(int k=7;k>=order;--k){double f=1.;for(int j=0;j<order;++j)f*=k-j;
        v=v*u+f*c.coefficients[24*piece+8*axis+k];}
    return v/std::pow(c.times[piece],order);
}
std::vector<double> original_state(const Json& meta,int i) {
    auto x=meta["original_initials"][i]["original"].get<std::vector<double>>();
    int n=meta["n"],vs=meta["velocity_start"];
    std::vector<double> z(x.begin(),x.begin()+n);z.insert(z.end(),x.begin()+vs,x.end());return z;
}
int main(int argc,char**argv) {try {
    if(argc==6 && std::string(argv[1])=="--export") {
        const auto result=read_json(argv[2]);const auto& selected=result.at("selected");
        if(selected.size()!=8)throw std::runtime_error("Expected eight candidates");
        fs::path destination=argv[5];if(fs::exists(destination))throw std::runtime_error("Refusing overwrite");
        for(const std::string method : {"ours","center"}) {
            std::string name=std::string(argv[4])+"_"+method;
            auto manifest=read_json(fs::path(argv[3])/"inputs"/name/"manifest.json");
            int n=manifest["n"],vs=manifest["velocity_start"];
            for(int i=0;i<8;++i) {
                auto x=manifest["initials"][i]["original"].get<std::vector<double>>();
                auto times=selected[i]["log_times"].get<std::vector<double>>();
                auto vaj=selected[i]["VAJ"].get<std::vector<double>>();
                if(times.size()!=size_t(n)||x.size()!=vs+vaj.size())throw std::runtime_error("Layout mismatch");
                std::copy(times.begin(),times.end(),x.begin());std::copy(vaj.begin(),vaj.end(),x.begin()+vs);
                manifest["initials"][i]["original"]=x;
            }
            manifest["initialization"]="C++ CEM full actual VAJ and independent segment times; center position initialization";
            auto target=destination/"inputs"/name;fs::create_directories(target);
            std::ofstream(target/"manifest.json")<<manifest.dump(2);
        }
        return 0;
    }
    if(argc!=8)throw std::runtime_error("Usage: generate_cem MODEL OUTPUT GENERATIONS POPULATION THREADS RNG_SEED CHECK_ONLY(0/1)");
    auto start=Clock::now(); int generations=std::stoi(argv[3]),pop=std::stoi(argv[4]),threads=std::stoi(argv[5]);
    if(pop<8 || threads<1 || generations<0)throw std::runtime_error("Invalid budget");
    std::vector<std::unique_ptr<Model>> models;for(int t=0;t<threads;++t)models.emplace_back(std::make_unique<Model>(argv[1]));
    const auto& meta=models[0]->metadata();int n=meta["n"],m=meta["m"],dim=n+9*m;
    // Verify every internal actual derivative is honored on both adjacent pieces.
    auto x=original_state(meta,0);auto base=models[0]->evaluate(x);
    if(!base.quality.finite)throw std::runtime_error("Nonfinite original state");
    double err=0.,changed=0.;
    for(int k=n;k<dim;++k) {
        auto pert=x;pert[k]+=.01*scale(k,n,m);auto c=models[0]->evaluate(pert);
        if(!c.quality.finite)throw std::runtime_error("Nonfinite perturbation");
        int j=k-n,axis=j%3,gate=(j/3)%m,order=j/(3*m)+1;
        err=std::max(err,std::abs(endpoint(c,gate,axis,order,1.)-pert[k]));
        err=std::max(err,std::abs(endpoint(c,gate+1,axis,order,0.)-pert[k]));
        changed=std::max(changed,std::abs(endpoint(c,gate,axis,order,1.)-endpoint(base,gate,axis,order,1.)));
    }
    if(err>1e-6 || changed<.01)throw std::runtime_error("PVAJ independence regression failed");
    Json check={{"passed",true},{"dimensions",dim},{"independent_derivatives_tested",dim-n},{"endpoint_max_error",err}};
    fs::path output=argv[2];if(fs::exists(output))throw std::runtime_error("Refusing to overwrite output");
    if(!output.parent_path().empty())fs::create_directories(output.parent_path());
    if(std::stoi(argv[7])){std::ofstream(output)<<check.dump(2);std::cout<<check<<'\n';return 0;}
    std::mt19937_64 rng(std::stoull(argv[6]));std::normal_distribution<double> normal(0.,1.);std::uniform_real_distribution<double> uni(0.,1.);
    std::vector<std::vector<double>> z(pop);std::vector<Candidate> values(pop);
    int origins=meta["original_initials"].size();
    const int origin_index=(std::stoull(argv[6])-20261113)%origins;
    for(int i=0;i<pop;++i){z[i]=original_state(meta,origin_index);for(int k=0;k<dim;++k){z[i][k]/=scale(k,n,m);if(i>0)z[i][k]+=(k<n?.12:.08)*normal(rng);}}
    auto evaluate=[&](const auto& states,auto& vals){
        #pragma omp parallel for num_threads(threads) schedule(static)
        for(int i=0;i<pop;++i)vals[i]=models[omp_get_thread_num()]->evaluate(decode(states[i],n,m));
    };
    evaluate(z,values);std::vector<double> violations;for(auto&v:values)if(v.quality.finite)violations.push_back(v.quality.maximum);
    if(violations.empty())throw std::runtime_error("No finite initial candidates");std::sort(violations.begin(),violations.end());double eps0=violations[violations.size()/2];
    std::vector<Candidate> archive=values;std::ofstream progress(output.string()+".progress.jsonl");
    const int elite_count=std::max(2,int(std::ceil(.2*pop)));
    const double alpha=.7, sigma_floor=.001;
    Eigen::VectorXd mean=Eigen::Map<const Eigen::VectorXd>(z[0].data(),dim);
    Eigen::MatrixXd covariance=Eigen::MatrixXd::Zero(dim,dim);
    for(int k=0;k<dim;++k)covariance(k,k)=std::pow(k<n?.12:.08,2);
    // Strict finite-node feasibility ordering; no permissive epsilon that rewards
    // shorter but increasingly infeasible states. Keep elites in each population.
    for(int g=0;g<generations;++g) {
        const double eps=meta["tolerance"];
        std::vector<int> order(pop);std::iota(order.begin(),order.end(),0);
        std::stable_sort(order.begin(),order.end(),[&](int a,int b){return better(values[a],values[b],eps);});
        std::vector<int> elite;for(int i:order)if(values[i].quality.finite&&int(elite.size())<elite_count)elite.push_back(i);
        if(elite.size()<2)throw std::runtime_error("Too few finite CEM elites");
        Eigen::VectorXd em=Eigen::VectorXd::Zero(dim);
        for(int i:elite)em+=Eigen::Map<const Eigen::VectorXd>(z[i].data(),dim)/elite.size();
        Eigen::MatrixXd ec=Eigen::MatrixXd::Zero(dim,dim);
        for(int i:elite){Eigen::VectorXd d=Eigen::Map<const Eigen::VectorXd>(z[i].data(),dim)-em;ec.noalias()+=d*d.transpose()/elite.size();}
        mean=(1.-alpha)*mean+alpha*em;
        covariance=(1.-alpha)*covariance+alpha*ec;
        covariance=.5*(covariance+covariance.transpose()).eval();
        Eigen::MatrixXd regularized=covariance;
        regularized.diagonal().array()+=sigma_floor*sigma_floor;
        Eigen::LLT<Eigen::MatrixXd> llt(regularized);
        if(llt.info()!=Eigen::Success)throw std::runtime_error("CEM covariance not positive definite");
        std::vector<std::vector<double>> next(pop);std::vector<Candidate> nv(pop);
        for(size_t i=0;i<elite.size();++i){next[i]=z[elite[i]];nv[i]=values[elite[i]];}
        for(int i=elite.size();i<pop;++i){Eigen::VectorXd noise(dim);for(int k=0;k<dim;++k)noise[k]=normal(rng);
            Eigen::VectorXd sample=mean+llt.matrixL()*noise;next[i]=std::vector<double>(sample.data(),sample.data()+dim);}
        #pragma omp parallel for num_threads(threads) schedule(static)
        for(int i=int(elite.size());i<pop;++i)nv[i]=models[omp_get_thread_num()]->evaluate(decode(next[i],n,m));
        z=std::move(next);values=std::move(nv);
        auto best=std::min_element(values.begin(),values.end(),[&](auto&a,auto&b){return better(a,b,eps);});
        archive.push_back(*best);
        progress<<Json{{"generation",g+1},{"epsilon",eps},{"elites",elite.size()},{"best",best->quality.json()},{"covariance_trace",covariance.trace()}}<<'\n';
    }
    for(auto& c:values)if(c.quality.finite)archive.push_back(c);
    // Preserve the whole finite sample archive. Select eight with sampled feasibility first,
    // then time; if infeasible, rank violations. Deduplicate in full normalized state.
    std::sort(archive.begin(),archive.end(),[](auto&a,auto&b){return a.quality.key()<b.quality.key();});
    Json selected=Json::array(),all=Json::array();std::vector<std::vector<double>> chosen;
    for(auto&c:archive){if(!c.quality.finite)continue;all.push_back(c.json());auto s=c.log_times;s.insert(s.end(),c.derivatives.begin(),c.derivatives.end());
        double nearest=1e100;for(auto&old:chosen){double d=0.;for(int k=0;k<dim;++k)d+=std::pow((s[k]-old[k])/scale(k,n,m),2);nearest=std::min(nearest,std::sqrt(d/dim));}
        if(selected.size()<8&&nearest>1e-4){selected.push_back(c.json());chosen.push_back(s);}}
    if(selected.size()!=8)throw std::runtime_error("Fewer than 8 distinct finite full states");
    Json result={{"algorithm","CEM independent full covariance with persistent elites"},{"dimensions",dim},{"generations",generations},{"population",pop},{"threads",threads},{"seed",std::stoull(argv[6])},{"evaluations",pop+(pop-elite_count)*generations},{"selected",selected},{"archive",all},{"validation",check},{"compute_wall_s",std::chrono::duration<double>(Clock::now()-start).count()},{"time_mapping","original frozen logT/exp"},{"state_scales",{5.,10.,20.}},{"initial_jitter",{.12,.08}},{"elite_fraction",.2},{"elite_count",elite_count},{"smoothing",alpha},{"sigma_floor",sigma_floor},{"origin_index",origin_index},{"epsilon_initial",meta["tolerance"]},{"epsilon_final",meta["tolerance"]},{"selection","sampled feasible/time then violation; normalized full-state duplicates removed"}};
    std::ofstream(output)<<result.dump(2);std::cout<<Json{{"selected",selected.size()},{"dimensions",dim},{"wall_s",result["compute_wall_s"]}}<<'\n';
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;} }
