#include <casadi/casadi.hpp>
#include "numpy_passage.hpp"
#include <filesystem>
#include <iomanip>
#include <map>
#include <numeric>
#include <sstream>

using casadi::DM;
using casadi::MX;
using casadi::Function;
using Clock=std::chrono::steady_clock;
using WitnessKey=std::pair<int,int>;
struct Witness {std::string kind;std::vector<double> values;};
struct FiniteProblem {
    Function solver;
    std::vector<std::string> keys;
    std::vector<double> lower,upper;
    int geometry_rows;
};

std::vector<double> numbers(const Json& value) {return value.get<std::vector<double>>();}
Json matrix(const Curve& curve) {
    Json result=Json::array();
    for(int i=0;i<8;++i)result.push_back({curve(i,0),curve(i,1),curve(i,2)});
    return result;
}
casadi::Dict solver_options(const Json& configuration) {
    casadi::Dict options;
    for(auto item=configuration.begin();item!=configuration.end();++item) {
        if(item.value().is_boolean()) options[item.key()]=item.value().get<bool>();
        else if(item.value().is_number_integer()) options[item.key()]=item.value().get<int>();
        else if(item.value().is_number()) options[item.key()]=item.value().get<double>();
        else options[item.key()]=item.value().get<std::string>();
    }
    return options;
}

class Planner {
    Json manifest;
    std::string directory;
    int n,m,nvar,workspace_rows;
    Function base,trajectory,baseline,residuals;
    std::vector<Function> knots;
    std::map<std::pair<int,std::string>,Function> frames;
    std::vector<Window> windows;
    std::vector<MX> expressions;
    Function physical_curve;
    std::ofstream progress;

    FiniteProblem build(const std::map<WitnessKey,Witness>& witnesses,bool warm) {
        const MX& variables=expressions[0];const MX& parameters=expressions[1];
        const MX times=MX::exp(expressions[2](casadi::Slice(0,n)));
        const MX& coefficients=expressions[4];
        const MX& objective=expressions[5];const MX& base_constraints=expressions[6];
        std::vector<MX> rows{base_constraints(casadi::Slice(0,workspace_rows))};
        FiniteProblem problem;
        const auto& lower=manifest.at("lower_g");const auto& upper=manifest.at("upper_g");
        auto append_bounds=[&](int begin,int end) {
            for(int k=begin;k<end;++k) {
                problem.lower.push_back(lower[k].get<double>());
                problem.upper.push_back(upper[k].is_null()?std::numeric_limits<double>::infinity():upper[k].get<double>());
                problem.keys.push_back("base_"+std::to_string(k));
            }
        };
        append_bounds(0,workspace_rows);
        const MX starts=MX::vertcat({MX(0.),MX::cumsum(times)(casadi::Slice(0,n-1))});
        for(const auto& entry:witnesses) {
            const int gate=entry.first.first,piece=entry.first.second;
            const auto& witness=entry.second;
            const int slots=manifest["config"]["witness_slots"];
            const auto& function=frames.at({gate,witness.kind});
            const MX state=MX::vertcat({coefficients(casadi::Slice(24*piece,24*(piece+1))),times(piece),starts(piece)});
            for(size_t begin=0;begin<witness.values.size();begin+=slots) {
                std::vector<double> unique;
                for(size_t k=begin;k<std::min(witness.values.size(),begin+slots);++k) {
                    double offset=witness.values[k];
                    if(std::find(unique.begin(),unique.end(),offset)==unique.end())unique.push_back(offset);
                }
                MX offsets=DM(unique).T();
                MX values=function.map(unique.size())(std::vector<MX>{
                    MX::repmat(state,1,unique.size()),offsets})[0];
                const bool at_knot=(witness.kind=="before" && piece==gate) ||
                    (witness.kind=="after" && piece==gate+1);
                for(size_t column=0;column<unique.size();++column) {
                    const double offset=unique[column];
                    if(at_knot && offset==0.)values(casadi::Slice(),column)=expressions[7+gate];
                    for(int edge=0;edge<values.size1();++edge) {
                        problem.lower.push_back(0.);problem.upper.push_back(std::numeric_limits<double>::infinity());
                        std::ostringstream key;key<<"frame_"<<gate<<'_'<<piece<<'_'<<witness.kind<<'_'<<std::setprecision(17)<<offset<<'_'<<edge;
                        problem.keys.push_back(key.str());
                    }
                }
                rows.push_back(MX::vec(values));
            }
        }
        problem.geometry_rows=static_cast<int>(problem.lower.size())+m;
        rows.push_back(base_constraints(casadi::Slice(workspace_rows,base_constraints.numel())));
        append_bounds(workspace_rows,base_constraints.numel());
        auto options=solver_options(manifest.at("solver_options"));
        if(warm)options["ipopt.warm_start_init_point"]="yes";
        // The original high-swing runner removes the three tiny warm-push overrides.
        casadi::MXDict formulation{{"x",variables},{"p",parameters},
            {"f",objective},{"g",MX::vertcat(rows)}};
        problem.solver=casadi::nlpsol("factorized_finite_sip","ipopt",formulation,options);
        return problem;
    }

public:
#ifdef SIP_EXPORT_PROBE
    void export_problem(const Json& rows,const std::string& output) {
        std::map<WitnessKey,Witness> witnesses;
        for(const auto& row:rows)witnesses[{row["gate"],row["piece"]}]={row["kind"],numbers(row["values"])};
        auto problem=build(witnesses,false);
        problem.solver.get_function("nlp_jac_g").save(output+"/solver_jacobian.casadi");
        problem.solver.get_function("nlp_grad_f").save(output+"/solver_gradient.casadi");
        auto inputs=base.mx_in();
        const auto gradient=problem.solver.get_function("nlp_grad_f")(inputs);
        const auto jacobian=problem.solver.get_function("nlp_jac_g")(inputs);
        Function checks("assembled_finite_checks",inputs,
            {gradient[0],jacobian[0],gradient[1],jacobian[1]});
        checks.save(output+"/checks.casadi");
    }
#endif
    Planner(const std::string& input,const std::string& output):directory(input) {
        std::ifstream(directory+"/manifest.json")>>manifest;
        n=manifest["n"];m=manifest["m"];nvar=manifest["nvar"];workspace_rows=manifest["workspace_rows"];
        std::ifstream symbolic(directory+"/symbolic.mx");
        if(!symbolic)throw std::runtime_error("Re-export this SIP graph with the schema-2 export_runtime.py before solving");
        const std::string serialized((std::istreambuf_iterator<char>(symbolic)),std::istreambuf_iterator<char>());
        expressions=casadi::StringDeserializer(serialized).unpack_mx_vector();
        if(expressions.size()!=static_cast<size_t>(7+m))throw std::runtime_error("Incomplete SIP expression bundle");
        physical_curve=Function::load(directory+"/physical_curve.casadi");
        base=Function::load(directory+"/base.casadi");
        trajectory=Function::load(directory+"/trajectory.casadi");
        baseline=Function::load(directory+"/baseline.casadi");residuals=Function::load(directory+"/residuals.casadi");
        for(int gate=0;gate<m;++gate) {
            windows.push_back(load_window(manifest["windows"][gate]));
            knots.push_back(Function::load(directory+"/knot_"+std::to_string(gate)+".casadi"));
            for(const auto& kind:{"before","after","normalized"})
                frames[{gate,kind}]=Function::load(directory+"/frame_"+std::to_string(gate)+"_"+kind+".casadi");
        }
        std::filesystem::create_directories(output);progress.open(output+"/progress.jsonl");
    }

    Json solve(int seed_index,const Json& center=Json()) {
        const auto began=Clock::now();
        Json seed=manifest.at("initials").at(center.is_null()?seed_index:0);
        if(!center.is_null()) {
            const auto previous=numbers(center.at("variables"));
            auto original=numbers(seed["original"]);
            const int velocity_start=manifest["velocity_start"];
            std::copy(previous.begin(),previous.begin()+n,original.begin());
            std::copy(previous.begin()+n,previous.end(),original.begin()+velocity_start);
            auto parameters=numbers(seed["parameters"]);double total=0.;
            for(int k=0;k<n;++k){total+=std::exp(original[k]);parameters[1+k]=total;}parameters[0]=total;
            const auto scale=numbers(manifest["state_scale"]);std::vector<double> point(original.begin(),original.end());
            for(int k=0;k<n;++k)point[k]=0.;
            for(size_t k=0;k<scale.size();++k)point[velocity_start+k]/=scale[k];
            const auto minco=baseline(std::vector<DM>{DM(point),DM(parameters)})[0].get_elements();
            for(size_t k=0;k<scale.size();++k)point[velocity_start+k]-=minco[k]/scale[k];
            const auto residual=residuals(std::vector<DM>{DM(point),DM(parameters)})[0].get_elements();
            const auto slack_scale=numbers(manifest["slack_scale"]);
            for(size_t k=0;k<residual.size();++k)point.push_back(std::max(residual[k],0.)/slack_scale[k]);
            seed["finite"]=point;seed["parameters"]=parameters;seed["original"]=original;
        }
        std::vector<double> current=numbers(seed["finite"]),parameters=numbers(seed["parameters"]);
        std::vector<double> physical=numbers(seed["original"]);
        std::vector<double> lam_x(current.size(),0.);std::map<std::string,double> lam_g;
        std::map<WitnessKey,Witness> witnesses;
        for(int gate=0;gate<m;++gate){witnesses[{gate,gate}]={"before",{0.}};witnesses[{gate,gate+1}]={"after",{}};}
        Json calls=Json::array(),records=Json::array();bool found=false;
        Json selected;
        for(int round=0;round<24;++round) {
            const auto build_begin=Clock::now();auto problem=build(witnesses,round>0);
            const double build_s=std::chrono::duration<double>(Clock::now()-build_begin).count();
            Json canonicalized=Json::array();
            if(round>0)for(int gate=0;gate<static_cast<int>(manifest["slices"].size());++gate) {
                const auto& selection=manifest["slices"][gate];
                if(!selection["circle"].get<bool>())continue;
                const int begin=selection["begin"],end=selection["end"];double squared=numpy_dot(current.data()+begin,current.data()+begin,end-begin);
                if(squared>1.){for(int k=begin;k<end;++k)current[k]/=squared;canonicalized.push_back(gate);}
            }
            std::vector<double> lower_x(current.size(),-std::numeric_limits<double>::infinity());
            std::fill(lower_x.begin()+nvar,lower_x.end(),0.);
            casadi::DMDict input{{"x0",DM(current)},{"p",DM(parameters)},
                {"lbx",DM(lower_x)},{"ubx",DM::inf(current.size())},
                {"lbg",DM(problem.lower)},{"ubg",DM(problem.upper)}};
            if(round>0) {
                std::vector<double> previous;for(const auto& key:problem.keys)previous.push_back(lam_g[key]);
                input["lam_x0"]=DM(lam_x);input["lam_g0"]=DM(previous);
            }
            const auto solve_begin=Clock::now();const auto solution=problem.solver(input);
            const double solve_s=std::chrono::duration<double>(Clock::now()-solve_begin).count();
            const auto statistics=problem.solver.stats();
            current=solution.at("x").nonzeros();lam_x=solution.at("lam_x").nonzeros();
            const auto multipliers=solution.at("lam_g").nonzeros();lam_g.clear();
            for(size_t k=0;k<problem.keys.size();++k)lam_g[problem.keys[k]]=multipliers[k];
            const auto values=trajectory(std::vector<DM>{DM(current),DM(parameters)});
            const auto original=values[0].get_elements(),times=values[1].get_elements(),coefficient_values=values[2].get_elements();
            std::vector<Curve> coefficients(n);Json curves=Json::array();
            for(int piece=0;piece<n;++piece) {
                for(int axis=0;axis<3;++axis)for(int power=0;power<8;++power)
                    coefficients[piece](power,axis)=coefficient_values[24*piece+8*axis+power];
                curves.push_back(matrix(coefficients[piece]));
            }
            const double objective=static_cast<double>(values[4]),penalty=static_cast<double>(values[5]);
            const auto g=solution.at("g").nonzeros();double minimum=std::numeric_limits<double>::infinity(),violation=0.;
            for(size_t k=0;k<g.size();++k) {
                if(static_cast<int>(k)<problem.geometry_rows)
                    minimum=std::min({minimum,g[k]-problem.lower[k],problem.upper[k]-g[k]});
                violation=std::max({violation,problem.lower[k]-g[k],g[k]-problem.upper[k]});
            }
            const bool success=statistics.at("success").to_bool();
            Json call{{"stage","factorized_finite_ipopt"},{"round",round},{"success",success},
                {"status",statistics.at("return_status").to_string()},{"iterations",statistics.at("iter_count").to_int()},
                {"solver_s",solve_s},{"problem_build_s",build_s},{"warm_start_used",round>0},
                {"native_objective",objective},{"dynamic_soft_integral",penalty},{"constraint_violation_inf",violation},
                {"minimum_constraint",minimum},{"solver_variables",current.size()},{"constraint_rows",g.size()},
                {"canonicalized_circle_gates",canonicalized},{"original_variables",original}};
            calls.push_back(call);progress<<call.dump()<<std::endl;
            std::cout<<"ROUND "<<seed_index<<' '<<round<<' '<<call["status"]<<' '<<objective<<' '<<solve_s<<std::endl;
            if(minimum < -1e-5 || !std::all_of(current.begin(),current.end(),[](double value){return std::isfinite(value);}))break;
            // The original outer loop converts the physical result through its scaled coordinates.
            for(size_t k=0;k<physical.size();++k) {
                double scale=1.;
                if(k<static_cast<size_t>(m))scale=1./std::max(1.,manifest["legacy_outer_omega"].get<double>()*std::exp(physical[k]));
                else if(k>=static_cast<size_t>(manifest["velocity_start"].get<int>())) {
                    const size_t state=k-manifest["velocity_start"].get<int>();
                    scale=manifest["state_scale"][state];
                }
                const double reduced=(original[k]-physical[k])/scale;
                physical[k]=physical[k]+scale*reduced;
            }
            std::vector<double> physical_times;for(int k=0;k<n;++k)physical_times.push_back(std::exp(physical[k]));
            const auto physical_values=physical_curve(std::vector<DM>{DM(physical)})[0].get_elements();
            for(int piece=0;piece<n;++piece)for(int axis=0;axis<3;++axis)for(int power=0;power<8;++power)
                coefficients[piece](power,axis)=physical_values[24*piece+8*axis+power];
            Json checks=Json::array();bool passed=true,added=false;
            for(int gate=0;gate<m;++gate) {
                double start=0.;
                for(int piece=0;piece<n;++piece) {
                    Json check=numpy_passage(coefficients[piece],physical_times[piece],start,windows[gate],manifest["verification_radius"]);
                    check["gate"]=gate;check["piece"]=piece;checks.push_back(check);passed=passed&&check["passed"].get<bool>();
                    start+=physical_times[piece];
                    if(check["passed"].get<bool>() || check["witness_u"].is_null())continue;
                    const WitnessKey key{gate,piece};if(!witnesses.count(key))witnesses[key]={"normalized",{}};
                    auto& witness=witnesses[key];const double u=check["witness_u"];
                    const double offset=witness.kind=="before"?(1.-u)*physical_times[piece]:witness.kind=="after"?u*physical_times[piece]:u;
                    if(std::all_of(witness.values.begin(),witness.values.end(),[&](double value){return std::abs(value-offset)>1e-5;})) {
                        witness.values.push_back(offset);added=true;
                    }
                }
            }
            records.push_back(Json{{"round",round},{"optimizer_converged",success},{"global_passed",passed},{"checks",checks}});
            if(passed && success) {
                found=true;Json points=Json::array();const auto point_values=values[3].get_elements();
                for(int gate=0;gate<m;++gate)points.push_back({point_values[3*gate],point_values[3*gate+1],point_values[3*gate+2]});
                std::vector<double> crossings;double total=0.;for(int k=0;k<n;++k){total+=physical_times[k];if(k<m)crossings.push_back(total);}
                curves=Json::array();for(const auto& coefficient:coefficients)curves.push_back(matrix(coefficient));
                selected={{"variables",physical},{"durations",physical_times},{"coefficients",curves},{"nominal_time_s",total},
                    {"objective",objective},{"dynamic_soft_integral",penalty},{"crossing_times_s",crossings},{"crossing_points",points},{"made_at",0.}};
                break;
            }
            if(passed && !success)continue;
            if(!added)break;
        }
        Json result=found?selected:Json::object();
        result.update(Json{{"found",found},{"solve_s",std::chrono::duration<double>(Clock::now()-began).count()},
            {"optimizer_calls",calls},{"records",records},{"config",manifest["config"]},{"level",manifest["level"]},
            {"method",manifest["method"]},{"initial_sample_index",seed_index},{"initial_variables",seed["original"]},
            {"runtime","C++17 CasADi/IPOPT and native continuous frame exchange"},{"all_seven_joint",true},
            {"replanning",false},{"dynamic_limit_fraction",.95},{"finite_safety_radius_m",manifest["finite_radius"]},
            {"verification_safety_radius_m",manifest["verification_radius"]}});
        return result;
    }
};

int main(int argc,char** argv) {
    try {
        if(argc!=4)throw std::runtime_error("Usage: sip_planner GRAPH_DIRECTORY OUTPUT_DIRECTORY SEED_INDEX");
        Planner planner(argv[1],argv[2]);Json center;
        const std::string seed=argv[3];
        if(seed.rfind("release=",0)==0)std::ifstream(seed.substr(8))>>center;
        const Json result=planner.solve(center.is_null()?std::stoi(seed):8,center);
        std::ofstream(std::string(argv[2])+"/result.json")<<result.dump(2)<<'\n';
    } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
