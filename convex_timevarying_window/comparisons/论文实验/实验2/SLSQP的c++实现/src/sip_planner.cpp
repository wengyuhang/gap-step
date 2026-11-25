#include "hard_problem.hpp"
#include "numpy_passage.hpp"
#include <filesystem>
#include <chrono>
#include <numeric>

using casadi::DM;
using Clock=std::chrono::steady_clock;
namespace fs=std::filesystem;
int main(int argc,char** argv) {
    try {
        if(argc!=6 || std::string(argv[4])!="--slsqp-native")
            throw std::runtime_error("Usage: sip_planner INPUT OUTPUT SEED --slsqp-native SCENE");
        fs::create_directories(argv[2]);const fs::path out=argv[2];const int seed=std::stoi(argv[3]);
        const auto process_begin=Clock::now();hard_problem::Model model(argv[1],argv[5]);
        const int n=model.metadata["n"],m=model.metadata["m"],nv=model.metadata["nvar"];
        auto x=model.initial(seed);
        const auto initial=x;
        const DM parameters(model.metadata["initials"][seed]["parameters"].get<std::vector<double>>());
        hard_problem::Witnesses witnesses;
        for(int gate=0;gate<m;++gate){witnesses[{gate,gate}]={"before",{0.}};witnesses[{gate,gate+1}]={"after",{}};}
        std::ofstream progress(out/"progress.jsonl");
        Json calls=Json::array(),records=Json::array(),selected;bool found=false;
        std::string reason="exchange_round_limit";
        for(int round=0;round<24;++round) {
            if(round)for(const auto& slice:model.metadata["slices"])if(slice["circle"].get<bool>()) {
                const int begin=slice["begin"],end=slice["end"];
                const double squared=numpy_dot(x.data()+begin,x.data()+begin,end-begin);
                if(squared>1.)for(int k=begin;k<end;++k)x[k]/=squared;
            }
            const auto build_begin=Clock::now();const auto finite=model.build(witnesses);
            const double build_s=std::chrono::duration<double>(Clock::now()-build_begin).count();
            const auto solved=native_slsqp::minimize(finite.values,finite.derivatives,parameters,x,finite.equalities,5000,1e-8,
                [&](int iteration,double objective) {
                    if(iteration%100==0)progress<<Json{{"stage","iteration"},{"round",round},{"iteration",iteration},{"scaled_objective",std::isfinite(objective)?Json(objective):Json(nullptr)}}.dump()<<std::endl;
                });
            x=solved.x;const bool success=solved.mode==0;
            if(solved.mode==-100) {
                const Json call{{"round",round},{"solver","SLSQP"},{"status",native_slsqp::status(solved.mode)},
                    {"success",false},{"mode",solved.mode},{"iterations",solved.iterations},{"solver_s",solved.seconds},
                    {"problem_build_s",build_s},{"solver_variables",nv},{"constraint_rows",finite.constraints},
                    {"dynamic_rows",finite.dynamic_rows}};
                calls.push_back(call);progress<<Json{{"stage","finite_slsqp"},{"record",call}}.dump()<<std::endl;
                reason="nonfinite_evaluation";break;
            }
            const auto geometry=finite.geometry(std::vector<DM>{DM(x),parameters})[0].get_elements();
            const auto dynamic=finite.dynamics(std::vector<DM>{DM(x),parameters})[0].get_elements();
            const double minimum=*std::min_element(geometry.begin(),geometry.end());
            const double dynamic_max=*std::max_element(dynamic.begin(),dynamic.end());
            const auto curve=model.graph->trajectory(std::vector<DM>{DM(x),parameters});
            const auto times=curve[1].get_elements(),coefficients=curve[2].get_elements();
            const double total=std::accumulate(times.begin(),times.end(),0.);
            Json call{{"round",round},{"solver","SLSQP"},{"status",native_slsqp::status(solved.mode)},{"mode",solved.mode},
                {"success",success},{"iterations",solved.iterations},{"solver_s",solved.seconds},{"problem_build_s",build_s},
                {"solver_variables",nv},{"constraint_rows",finite.constraints},{"equalities",finite.equalities},
                {"dynamic_rows",finite.dynamic_rows},{"minimum_constraint",minimum},{"constraint_violation_inf",solved.violation},
                {"sampled_dynamic_max_residual",dynamic_max},{"native_objective",total},{"warm_start_previous_primal",round>0},
                {"dual_warm_start_supported",false},{"function_evaluations",solved.evaluations},{"derivative_evaluations",solved.derivatives}};
            calls.push_back(call);progress<<Json{{"stage","finite_slsqp"},{"record",call}}.dump()<<std::endl;
            if(solved.mode==-100 || !std::isfinite(total) || solved.violation>1e-5 || minimum < -1e-5 || dynamic_max>1e-5) {
                reason=solved.mode==-100?"nonfinite_evaluation":"finite_constraints_not_satisfied";break;
            }
            Json checks=Json::array();bool passed=true,added=false;
            for(int gate=0;gate<m;++gate) {
                double start=0.;
                for(int piece=0;piece<n;++piece) {
                    Curve c;for(int axis=0;axis<3;++axis)for(int power=0;power<8;++power)c(power,axis)=coefficients[24*piece+8*axis+power];
                    auto check=numpy_passage(c,times[piece],start,model.windows[gate],model.metadata["verification_radius"]);
                    check["gate"]=gate;check["piece"]=piece;checks.push_back(check);passed=passed && check["passed"].get<bool>();
                    start+=times[piece];
                    if(check["passed"].get<bool>() || check["witness_u"].is_null())continue;
                    const hard_problem::Key key{gate,piece};if(!witnesses.count(key))witnesses[key]={"normalized",{}};
                    auto& witness=witnesses[key];const double u=check["witness_u"];
                    const double value=witness.kind=="before"?(1.-u)*times[piece]:witness.kind=="after"?u*times[piece]:u;
                    if(std::all_of(witness.values.begin(),witness.values.end(),[&](double old){return std::abs(value-old)>1e-5;})) {
                        witness.values.push_back(value);added=true;
                    }
                }
            }
            const Json record{{"round",round},{"optimizer_converged",success},{"global_passed",passed},{"checks",checks},{"added",added}};
            records.push_back(record);progress<<Json{{"stage","continuous_geometry"},{"record",record}}.dump()<<std::endl;
            if(passed && success) {
                Json packed=Json::array(),points=Json::array();
                for(int piece=0;piece<n;++piece){Json c=Json::array();for(int power=0;power<8;++power)c.push_back({coefficients[24*piece+power],coefficients[24*piece+8+power],coefficients[24*piece+16+power]});packed.push_back(c);}
                const auto positions=curve[3].get_elements();for(int gate=0;gate<m;++gate)points.push_back({positions[3*gate],positions[3*gate+1],positions[3*gate+2]});
                double arrival=0.;Json arrivals=Json::array();for(int gate=0;gate<m;++gate){arrival+=times[gate];arrivals.push_back(arrival);}
                selected={{"durations",times},{"coefficients",packed},{"variables",curve[0].get_elements()},
                    {"nominal_time_s",total},{"objective",total},{"crossing_points",points},{"crossing_times_s",arrivals}};
                found=true;reason="verified_solution";break;
            }
            if(passed && !success)continue;
            if(!added){reason="no_new_safety_witness";break;}
        }
        const double wall=std::chrono::duration<double>(Clock::now()-process_begin).count();
        Json result{{"found",found},{"solve_s",wall},{"build_s",0.},{"optimizer_calls",calls},{"records",records},
            {"termination_reason",reason},{"solver","native SLSQP (SciPy1.14.1 Kraft core)"},{"trajectory_variables",nv},
            {"minco_derivative_order",3},{"baseline_offset_used",false},{"dynamics_auxiliary_variables",0},
            {"dynamic_nodes_per_piece",17},{"dynamic_scalar_constraints",n*17*6},{"dynamic_hard_constraints",true},
            {"dynamics_verification","17 uniform normalized-time nodes per piece"},{"time_weight",1.},{"dynamics_weight",0.},
            {"initial_sample_index",seed},{"initial_variables",model.metadata["initials"][seed]["original"]},
            {"time_mapping","TOGT positive independent durations"},{"config",model.metadata["config"]},
            {"soft_dynamic_penalty",false},{"warm_start","previous finite problem primal; SLSQP internal QP state reset"}};
        if(found)result.update(selected);
        std::ofstream(out/"result.json")<<result.dump(2)<<'\n';
        std::cout<<"COMPLETE "<<seed<<' '<<found<<' '<<reason<<std::endl;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
