#include <casadi/casadi.hpp>
#include "numpy_passage.hpp"
#include "dynamics_exchange.hpp"
#include "minimum_thrust.hpp"
#include "stable_dynamics.hpp"
#include "native_minco_pvaj.hpp"
#include "restored_kernels.hpp"
#include <filesystem>
#include <iomanip>
#include <map>
#include <numeric>
#include <sstream>
#include <memory>

using casadi::DM;
using casadi::MX;
using casadi::Function;
using Clock=std::chrono::steady_clock;
using WitnessKey=std::pair<int,int>;
using DynamicWitnesses=std::map<std::pair<int,int>,std::vector<double>>;
struct Witness {std::string kind;std::vector<double> values;};
struct FiniteProblem {
    Function solver,projector;
    std::vector<std::string> keys;
    std::vector<double> lower,upper;
    int geometry_rows;
    size_t dynamic_row_start=0;
    std::vector<std::pair<int,double>> thrust_nodes;
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
    bool native_minco_enabled=false,native_soft_enabled=false,direct_points_enabled=false;
    bool restored_soft=false;
    double native_dynamics_weight=1.;
    std::unique_ptr<native_minco::Graph> native_graph;
    std::string directory,destination;
    int n,m,nvar,workspace_rows,hard_nodes=0;
    bool direct_soft=false,unit_epigraph=false,sampled_hard=false,reset_togt_each_round=false,dynamic_exchange=false;
    Function base,trajectory,baseline,residuals,global_dynamics;
    bool minimum_thrust_enabled=false,stable_dynamics_enabled=false,direct_pvaj=false,norm_dynamics=false,lifted_thrust=false,phase_scaled_time=false,feasible_sqp=false;
    std::vector<double> time_centers,time_scales;
    Function exchange_node;
    Function exchange_to_original;
    Json dynamics_metadata;
    std::unique_ptr<dynamics_exchange::Model> dynamics_model;
    std::vector<Function> knots;
    std::map<std::pair<int,std::string>,Function> frames;
    std::vector<Window> windows;
    std::vector<MX> expressions;
    Function physical_curve;
    std::ofstream progress;

    int dynamic_components() const { return minimum_thrust_enabled ? 7 : 6; }

    Json check_dynamics(const Curve& curve, double duration) const {
        Json check=dynamics_exchange::check_segment(*dynamics_model,curve,duration,
            dynamics_metadata["verification_residual_tolerance"]);
        if(minimum_thrust_enabled) {
            const auto thrust=minimum_thrust::check(curve,duration,dynamics_model->mass,dynamics_model->gravity);
            check["minimum_thrust"]=thrust;
            check["peak_residuals"].push_back(thrust["normalized_residual"]);
            check["peak_fractions"].push_back(thrust["witness_u"]);
            check["local_peak_fractions"].push_back(thrust["candidate_fractions"]);
            check["check_s"]=check["check_s"].get<double>()+thrust["check_s"].get<double>();
            if(!thrust["passed"].get<bool>()) {
                check["passed"]=false;check["outcome"]="violated";
            }
        }
        return check;
    }

    FiniteProblem build(const std::map<WitnessKey,Witness>& witnesses,bool warm,const DynamicWitnesses& dynamic_witnesses={}) {
        const MX& variables=expressions[0];const MX& parameters=expressions[1];
        const MX times=MX::exp(expressions[2](casadi::Slice(0,n)));
        const MX& coefficients=expressions[4];
        const MX& objective=expressions[5];const MX& base_constraints=expressions[6];
        std::vector<MX> rows{base_constraints(casadi::Slice(0,workspace_rows))};
        std::vector<MX> optimization_variables{variables};
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
                const bool at_knot=(witness.kind=="before" && piece==gate) ||
                    (witness.kind=="after" && piece==gate+1);
                // The input point map covers the opening eroded by finite_radius.
                // Assigned-knot safety is therefore already enforced by the map.
                if(norm_dynamics && manifest["free_points"].get<bool>() && at_knot)
                    unique.erase(std::remove(unique.begin(),unique.end(),0.),unique.end());
                if(unique.empty())continue;
                MX offsets=DM(unique).T();
                MX values=function.map(unique.size())(std::vector<MX>{
                    MX::repmat(state,1,unique.size()),offsets})[0];
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
        problem.geometry_rows=static_cast<int>(problem.lower.size())+m+manifest.value("point_domain_rows",0);
        if(norm_dynamics) {
            rows.push_back(base_constraints(casadi::Slice(workspace_rows,workspace_rows+m)));
            append_bounds(workspace_rows,workspace_rows+m);
            const int after_times=workspace_rows+m+n;
            rows.push_back(base_constraints(casadi::Slice(after_times,base_constraints.numel())));
            append_bounds(after_times,base_constraints.numel());
        } else {
            rows.push_back(base_constraints(casadi::Slice(workspace_rows,base_constraints.numel())));
            append_bounds(workspace_rows,base_constraints.numel());
        }
        problem.dynamic_row_start=problem.lower.size();
        if(dynamic_exchange) {
            for(int piece=0;piece<n;++piece) {
                std::vector<double> fractions;
                for(int component=0;component<dynamic_components();++component)for(double u:dynamic_witnesses.at({piece,component}))
                    if(std::find(fractions.begin(),fractions.end(),u)==fractions.end())fractions.push_back(u);
                std::sort(fractions.begin(),fractions.end());
                const MX state=MX::vertcat({coefficients(casadi::Slice(24*piece,24*(piece+1))),times(piece)});
                std::vector<MX> node_inputs{MX::repmat(state,1,fractions.size()),DM(fractions).T()};
                if(lifted_thrust) {
                    const MX force=MX::sym("positive_thrust_"+std::to_string(piece),fractions.size());
                    optimization_variables.push_back(force);
                    node_inputs.push_back(force.T());
                    for(double u:fractions)problem.thrust_nodes.push_back({piece,u});
                }
                const MX values=exchange_node.map(fractions.size())(node_inputs)[0];
                for(int component=0;component<dynamic_components();++component) {
                    const auto& nodes=norm_dynamics || (stable_dynamics_enabled && component==6) ?
                        fractions : dynamic_witnesses.at({piece,component});
                    for(double u:nodes) {
                        const auto column=std::find(fractions.begin(),fractions.end(),u)-fractions.begin();
                        rows.push_back(values(component,column));
                        const bool link=lifted_thrust && component==6;
                        problem.lower.push_back(link ? 0. : -INFINITY);
                        problem.upper.push_back(link ? 0. : -dynamics_metadata["finite_residual_slack"].get<double>());
                        std::ostringstream key;key<<"dynamic_"<<piece<<'_'<<component<<'_'<<std::setprecision(17)<<u;
                        problem.keys.push_back(key.str());
                    }
                }
            }
        } else if(sampled_hard) {
            const MX values=global_dynamics(std::vector<MX>{coefficients,times})[0];
            rows.push_back(values);
            for(int k=0;k<values.numel();++k) {
                problem.lower.push_back(-std::numeric_limits<double>::infinity());
                problem.upper.push_back(0.);
                problem.keys.push_back("global_dynamic_"+std::to_string(k));
            }
        }
        auto options=solver_options(manifest.at("solver_options"));
        if(direct_soft) {
            options["ipopt.nlp_scaling_method"]="gradient-based";
        }
        if(direct_soft && std::getenv("SIP_IPOPT_TRACE")) {
            options["ipopt.print_level"]=5;
            options["ipopt.max_iter"]=1000;
        }
        if(stable_dynamics_enabled) {
            options["ipopt.hessian_approximation"]=std::getenv("SIP_LIMITED_MEMORY") ? "limited-memory" : "exact";
        }
        if(dynamic_exchange && std::getenv("SIP_IPOPT_TRACE"))options["ipopt.print_level"]=5;
        if(warm)options["ipopt.warm_start_init_point"]="yes";
        // The original high-swing runner removes the three tiny warm-push overrides.
        casadi::MXDict formulation{{"x",MX::vertcat(optimization_variables)},{"p",parameters},
            {"f",objective},{"g",MX::vertcat(rows)}};
        if(std::getenv("SIP_DUMP_NLP"))
            Function("frozen_finite_nlp",{formulation.at("x"),parameters},{objective,formulation.at("g")})
                .save(destination+"/finite_nlp.casadi");
        if(feasible_sqp) {
            auto projection_options=options;
            projection_options["ipopt.hessian_approximation"]="exact";
            const MX reference=MX::sym("feasible_initial_reference",variables.numel());
            auto projection=formulation;
            projection["p"]=MX::vertcat({parameters,reference});
            projection["f"]=.5*MX::sumsqr(variables-reference);
            problem.projector=casadi::nlpsol("finite_feasible_initialization","ipopt",projection,projection_options);
            const casadi::Dict qp_options{{"printLevel","none"},{"error_on_fail",false}};
            const casadi::Dict sqp_options{{"qpsol","qpoases"},{"qpsol_options",qp_options},
                {"hessian_approximation","exact"},{"convexify_strategy","eigen-clip"},
                {"convexify_margin",1e-5},{"max_iter_eig",1000.},{"max_iter",5000},{"feas_tol",1e-8},{"optim_tol",1e-8},
                {"tol_pr",1e-7},{"tol_du",1e-5},{"tr_rad0",.1},{"max_inner_iter",50.},
                {"print_header",true},{"print_iteration",true},{"print_status",true},
                {"print_time",false},{"error_on_fail",false}};
            if(std::getenv("SIP_PROJECTED_IPOPT"))
                problem.solver=casadi::nlpsol("finite_projected_ipopt","ipopt",formulation,options);
            else problem.solver=casadi::nlpsol("finite_feasible_sqp","feasiblesqpmethod",formulation,sqp_options);
        } else problem.solver=casadi::nlpsol("factorized_finite_sip","ipopt",formulation,options);
        return problem;
    }

public:
#ifdef SIP_EXPORT_PROBE
    void export_problem(const Json& rows,const std::string& output,const Json& dynamics=Json::array()) {
        std::map<WitnessKey,Witness> witnesses;
        for(const auto& row:rows)witnesses[{row["gate"],row["piece"]}]={row["kind"],numbers(row["values"])};
        DynamicWitnesses dynamic_witnesses;
        if(dynamic_exchange)for(int piece=0;piece<n;++piece)for(int component=0;component<dynamic_components();++component)
            dynamic_witnesses[{piece,component}]={0.,.5,1.};
        for(const auto& row:dynamics)for(const auto& u:row["values"])
            dynamic_witnesses[{row["piece"],row["component"]}].push_back(u);
        auto problem=build(witnesses,false,dynamic_witnesses);
        std::ofstream(output+"/constraint_metadata.json")<<Json{{"keys",problem.keys},
            {"lower",problem.lower},{"upper",problem.upper},{"dynamic_row_start",problem.dynamic_row_start}}.dump(2)<<'\n';
        problem.solver.get_function("nlp_jac_g").save(output+"/solver_jacobian.casadi");
        problem.solver.get_function("nlp_grad_f").save(output+"/solver_gradient.casadi");
        std::vector<MX> inputs{expressions[0],expressions[1]};
        const auto gradient=problem.solver.get_function("nlp_grad_f")(inputs);
        const auto jacobian=problem.solver.get_function("nlp_jac_g")(inputs);
        Function checks("assembled_finite_checks",inputs,
            {gradient[0],jacobian[0],gradient[1],jacobian[1]});
        checks.save(output+"/checks.casadi");
    }
#endif
    Planner(const std::string& input,const std::string& output,bool direct=false,bool unit=false,bool hard=false,bool reset=false,bool exchange=false,bool minimum_thrust=false,bool stable=false,bool direct_states=false,bool norm_form=false,bool lift=false,bool scaled_time=false,bool feasible_method=false,const std::string& native_scene="",bool native_soft=false,double native_weight=1.,bool direct_points=false,int derivative_order=3,bool restored=false):directory(input),destination(output),direct_soft(direct),unit_epigraph(unit),sampled_hard(hard),reset_togt_each_round(reset),dynamic_exchange(exchange) {
        restored_soft=restored;
        minimum_thrust_enabled=minimum_thrust;stable_dynamics_enabled=stable;direct_pvaj=direct_states;norm_dynamics=norm_form;lifted_thrust=lift;phase_scaled_time=scaled_time;feasible_sqp=feasible_method;
        if(minimum_thrust_enabled && !dynamic_exchange)throw std::runtime_error("Minimum thrust requires dynamics exchange");
        if(reset_togt_each_round && !sampled_hard)throw std::runtime_error("TOGT reset requires sampled-hard dynamics");
        std::ifstream(directory+"/manifest.json")>>manifest;
        n=manifest["n"];m=manifest["m"];nvar=manifest["nvar"];workspace_rows=manifest["workspace_rows"];
        if(!native_scene.empty()) {
            native_minco_enabled=true;native_soft_enabled=native_soft;native_dynamics_weight=native_weight;direct_pvaj=true;
            Json scene;std::ifstream(native_scene)>>scene;
            if(restored_soft)manifest["restore_historical_domain"]=true;
            else {native_minco::remove_domain_bounds(manifest);workspace_rows=0;}
            direct_points_enabled=direct_points;
            if(direct_points_enabled)native_minco::use_direct_point_coordinates(manifest);
            native_minco::use_derivative_order(manifest,derivative_order);nvar=manifest["nvar"];
            for(int gate=0;gate<m;++gate) {
                windows.push_back(load_window(manifest["windows"][gate]));
                for(const auto& kind:{"before","after","normalized"})
                    frames[{gate,kind}]=restored_soft ? restored_kernels::frame_node(windows.back(),kind,manifest["finite_radius"]) :
                        Function::load(directory+"/frame_"+std::to_string(gate)+"_"+kind+".casadi");
            }
            std::ifstream(directory+"/exchange_dynamics.json")>>dynamics_metadata;
            exchange_node=restored_soft ? restored_kernels::dynamic_node(dynamics_metadata) :
                Function::load(directory+"/exchange_dynamic_node.casadi");
            dynamics_model=std::make_unique<dynamics_exchange::Model>(dynamics_metadata);
            if(native_soft_enabled)manifest["dynamic_residual_scale"]=dynamics_metadata["residual_scale"];
            native_graph=std::make_unique<native_minco::Graph>(manifest,scene,windows,
                native_soft_enabled ? exchange_node : Function(),native_dynamics_weight);
            expressions=native_graph->expressions;trajectory=native_graph->trajectory;
            if(native_soft_enabled)residuals=native_graph->residuals;
            else {
                const MX state=MX::sym("minimum_thrust_segment",25),u=MX::sym("minimum_thrust_fraction");
                const MX curve=MX::reshape(state(casadi::Slice(0,24)),8,3);
                std::vector<MX> acceleration;
                for(int axis=0;axis<3;++axis) {
                    MX value=42.*curve(7,axis);
                    for(int power=6;power>=2;--power)value=value*u+double(power*(power-1))*curve(power,axis);
                    acceleration.push_back(value/(state(24)*state(24))+(axis==2?dynamics_model->gravity:0.));
                }
                const double floor=minimum_thrust::collective_floor/dynamics_model->mass;
                const MX residual=(floor*floor-MX::sumsqr(MX::vertcat(acceleration)))/
                    (dynamics_model->gravity*dynamics_model->gravity);
                exchange_node=Function("native_exchange_with_minimum_thrust",{state,u},
                    {MX::vertcat({exchange_node(std::vector<MX>{state,u})[0],residual})});
                dynamics_metadata["minimum_collective_thrust_n"]=minimum_thrust::collective_floor;
            }
            const size_t geometry_size=expressions[6].numel();
            auto low=manifest["lower_g"].get<std::vector<double>>();low.resize(geometry_size);manifest["lower_g"]=low;
            auto high=manifest["upper_g"].get<std::vector<Json>>();high.resize(geometry_size);manifest["upper_g"]=high;
            progress.open(output+"/progress.jsonl");
            return;
        }

        std::ifstream symbolic(directory+"/symbolic.mx");
        if(!symbolic)throw std::runtime_error("Re-export this SIP graph with the schema-2 export_runtime.py before solving");
        const std::string serialized((std::istreambuf_iterator<char>(symbolic)),std::istreambuf_iterator<char>());
        expressions=casadi::StringDeserializer(serialized).unpack_mx_vector();
        if(expressions.size()!=static_cast<size_t>(7+m))throw std::runtime_error("Incomplete SIP expression bundle");
        physical_curve=Function::load(directory+"/physical_curve.casadi");
        base=Function::load(directory+"/base.casadi");
        trajectory=Function::load(directory+"/trajectory.casadi");
        baseline=Function::load(directory+"/baseline.casadi");residuals=Function::load(directory+"/residuals.casadi");
        if(dynamic_exchange) {
            exchange_node=Function::load(directory+"/exchange_dynamic_node.casadi");
            std::ifstream(directory+"/exchange_dynamics.json")>>dynamics_metadata;
            dynamics_model=std::make_unique<dynamics_exchange::Model>(dynamics_metadata);
            if(minimum_thrust_enabled) {
                const MX state=MX::sym("minimum_thrust_segment",25),u=MX::sym("minimum_thrust_fraction");
                const MX curve=MX::reshape(state(casadi::Slice(0,24)),8,3);
                std::vector<MX> acceleration;
                for(int axis=0;axis<3;++axis) {
                    MX value=42.*curve(7,axis);
                    for(int power=6;power>=2;--power)value=value*u+double(power*(power-1))*curve(power,axis);
                    acceleration.push_back(value/(state(24)*state(24))+(axis==2?dynamics_model->gravity:0.));
                }
                const MX a=MX::vertcat(acceleration);
                const double floor=minimum_thrust::collective_floor/dynamics_model->mass;
                const MX residual=(floor*floor-MX::sumsqr(a))/(dynamics_model->gravity*dynamics_model->gravity);
                exchange_node=Function("exchange_with_minimum_thrust",{state,u},
                    {MX::vertcat({exchange_node(std::vector<MX>{state,u})[0],residual})});
                dynamics_metadata["minimum_collective_thrust_n"]=minimum_thrust::collective_floor;
                if(stable_dynamics_enabled)exchange_node=stable_dynamics::segment_node(*dynamics_model,norm_dynamics,lifted_thrust);
            }
        } else if(sampled_hard) {
            global_dynamics=Function::load(directory+"/global_uniform_dynamics.casadi");
            const int residual_count=global_dynamics.size1_out(0);
            if(residual_count%6)throw std::runtime_error("Global dynamics graph must have six residuals per time node");
            hard_nodes=residual_count/6;
        }
        if(unit_epigraph) {
            const MX parameters=expressions[1];
            const MX total=MX::sum1(MX::exp(expressions[2](casadi::Slice(0,n))));
            expressions[5]=(total+(expressions[5]*parameters(0)-total)/1000.)/parameters(0);
        }
        if(direct_soft || sampled_hard) {
            const MX extended=expressions[0];
            const MX parameters=expressions[1];
            const MX z=MX::sym(sampled_hard?"hard_trajectory":"direct_trajectory",nvar);
            const auto scales=numbers(manifest.at("slack_scale"));
            const MX slacks=sampled_hard?MX::zeros(scales.size(),1):
                MX::fmax(residuals(std::vector<MX>{z,parameters})[0]/DM(scales),0.);
            const MX replacement=MX::vertcat({z,slacks});
            std::vector<MX> original(expressions.begin()+2,expressions.end());
            auto reduced=MX::substitute(original, std::vector<MX>{extended},std::vector<MX>{replacement});
            expressions[0]=z;
            for(size_t i=0;i<reduced.size();++i)expressions[i+2]=reduced[i];
            const MX total=MX::sum1(MX::exp(expressions[2](casadi::Slice(0,n))));
            expressions[5]=sampled_hard?total/parameters(0):
                total+(expressions[5]*parameters(0)-total)/1000.;
            const int soft_rows=n*33*6;
            expressions[6]=expressions[6](casadi::Slice(0,expressions[6].numel()-soft_rows));
            if(direct_soft)Function("direct_objective_and_gradient",{z,parameters},{expressions[5],MX::gradient(expressions[5],z)}).save(output+"/direct_objective_gradient.casadi");
            auto lower=manifest["lower_g"].get<std::vector<double>>();
            lower.resize(lower.size()-soft_rows);manifest["lower_g"]=lower;
            auto upper=manifest["upper_g"].get<std::vector<Json>>();
            upper.resize(upper.size()-soft_rows);manifest["upper_g"]=upper;
        }
        if(direct_pvaj) {
            const int start=manifest["velocity_start"];
            const MX old_states=expressions[2](casadi::Slice(start,nvar));
            const MX new_states=DM(numbers(manifest["state_scale"]))*expressions[0](casadi::Slice(start,nvar));
            std::vector<MX> original(expressions.begin()+2,expressions.end());
            const auto before=Function("baseline_state_coordinates",{expressions[0],expressions[1]},{expressions[2]});
            bool updated=false;
            const auto changed=MX::graph_substitute(original,{old_states},{new_states},updated);
            if(!updated)throw std::runtime_error("Full PVAJ branch was not found in the exported graph");
            for(size_t k=0;k<changed.size();++k)expressions[k+2]=changed[k];
            const auto after=Function("direct_state_coordinates",{expressions[0],expressions[1]},{expressions[2]});
            std::ofstream(output+"/pvaj_coordinates.json")<<Json{{"baseline_physical_nodes",before.n_nodes()},
                {"direct_physical_nodes",after.n_nodes()},{"state_scale",manifest["state_scale"]}}.dump(2)<<'\n';
        }
        if(dynamic_exchange && dynamics_metadata.value("togt_time_coordinates",false)) {
            const MX old_variables=expressions[0],parameters=expressions[1];
            const MX z=MX::sym("independent_segment_coordinates",nvar);
            const MX origin=parameters(casadi::Slice(1,1+n));
            const MX scale=parameters(casadi::Slice(1+n,1+2*n));
            MX k=z(casadi::Slice(0,n));
            if(phase_scaled_time) {
                const auto initial=numbers(manifest["initials"][0]["original"]);
                const auto p=numbers(manifest["initials"][0]["parameters"]);
                for(int piece=0;piece<n;++piece) {
                    const double t=std::exp(initial[piece]);
                    const double center=t>1.?std::sqrt(2.*t-1.)-1.:1.-std::sqrt(2./t-1.);
                    const double derivative=center>0.?center+1.:std::pow(t,2)*(1.-center);
                    time_centers.push_back(center);time_scales.push_back(p[1+n+piece]/derivative);
                }
                k=DM(time_centers)+DM(time_scales)*k;
                std::ofstream(output+"/time_coordinates.json")<<Json{{"centers",time_centers},
                    {"scales",time_scales},{"initial_zero",true}}.dump(2)<<'\n';
            }
            const MX positive=MX::if_else(k>0.,.5*k*k+k+1.,1./(.5*k*k-k+1.));
            const MX old_time=(MX::cumsum(positive)-origin)/scale;
            const MX replacement=MX::vertcat({old_time,z(casadi::Slice(n,nvar))});
            std::vector<MX> original(expressions.begin()+2,expressions.end());
            const auto changed=MX::substitute(original,std::vector<MX>{old_variables},std::vector<MX>{replacement});
            expressions[0]=z;for(size_t k=0;k<changed.size();++k)expressions[k+2]=changed[k];
            MX reporting=replacement;
            if(direct_pvaj) {
                const int start=manifest["velocity_start"];
                const MX offset=z(casadi::Slice(start,nvar))-
                    baseline(std::vector<MX>{replacement,parameters})[0]/DM(numbers(manifest["state_scale"]));
                reporting=MX::vertcat({replacement(casadi::Slice(0,start)),offset});
            }
            exchange_to_original=Function("exchange_to_original_coordinates",{z,parameters},{reporting});
            Function("exchange_coordinate_probe",{z,parameters},{expressions[2],expressions[4],
                MX::gradient(expressions[5],z)}).save(output+"/coordinate_probe.casadi");
        }
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
        if(native_minco_enabled && !restored_soft && !center.is_null())
            throw std::runtime_error("Native MINCO expects an explicit physical seed in manifest.initials");
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
            if(!restored_soft) {
                const auto minco=baseline(std::vector<DM>{DM(point),DM(parameters)})[0].get_elements();
                for(size_t k=0;k<scale.size();++k)point[velocity_start+k]-=minco[k]/scale[k];
            }
            if(restored_soft)point.resize(nvar+manifest["slack_scale"].size(),0.);
            else {
                const auto residual=residuals(std::vector<DM>{DM(point),DM(parameters)})[0].get_elements();
                const auto slack_scale=numbers(manifest["slack_scale"]);
                if(!direct_soft && !sampled_hard)for(size_t k=0;k<residual.size();++k)point.push_back(std::max(residual[k],0.)/slack_scale[k]);
            }
            seed["finite"]=point;seed["parameters"]=parameters;seed["original"]=original;
        }
        std::vector<double> current=numbers(seed["finite"]),parameters=numbers(seed["parameters"]);
        if(direct_soft || sampled_hard)current.resize(nvar);
        if(native_minco_enabled) {
            const auto initial=numbers(seed["original"]);
            std::copy(initial.begin(),initial.end(),current.begin());
            for(int k=0;k<n;++k) {
                const double t=std::exp(initial[k]);
                current[k]=t>1.?std::sqrt(2.*t-1.)-1.:1.-std::sqrt(2./t-1.);
            }
        }
        if(!exchange_to_original.is_null()) {
            auto reporting=numbers(seed["finite"]);reporting.resize(nvar+numbers(manifest["slack_scale"]).size(),0.);
            const auto seed_times=trajectory(std::vector<DM>{DM(reporting),DM(parameters)})[1].get_elements();
            for(int k=0;k<n;++k) {
                current[k]=seed_times[k]>1.?std::sqrt(2.*seed_times[k]-1.)-1.:1.-std::sqrt(2./seed_times[k]-1.);
                if(phase_scaled_time)current[k]=(current[k]-time_centers[k])/time_scales[k];
            }
        }
        if(direct_pvaj) {
            const auto physical_seed=numbers(seed["original"]),scale=numbers(manifest["state_scale"]);
            const int start=manifest["velocity_start"];
            for(int k=start;k<nvar;++k)current[k]=physical_seed[k]/scale[k-start];
        }
        if(direct_points_enabled && manifest["free_points"].get<bool>()) {
            auto axis=current;
            for(const auto& slice:manifest["slices"])if(slice["circle"].get<bool>() && current[slice["begin"].get<int>()]==0.)axis[slice["begin"].get<int>()+1]=0.;
            const auto gx=native_graph->point_objective_gradient(std::vector<DM>{DM(axis),DM(parameters)})[0].get_elements();
            for(const auto& slice:manifest["slices"])if(slice["circle"].get<bool>() && current[slice["begin"].get<int>()]==0.)axis[slice["begin"].get<int>()+1]=.5*pi;
            const auto gy=native_graph->point_objective_gradient(std::vector<DM>{DM(axis),DM(parameters)})[0].get_elements();
            auto physical=numbers(seed["original"]);
            for(const auto& slice:manifest["slices"])if(slice["circle"].get<bool>()) {
                const int begin=slice["begin"];
                if(current[begin]==0.) {
                    double angle=(gx[begin]==0. && gy[begin]==0.) ? 0. : std::atan2(-gy[begin],-gx[begin]);
                    if(angle<0.)angle+=2.*pi;
                    current[begin+1]=angle;physical[begin+1]=angle;
                }
            }
            seed["original"]=physical;
        }
        if(native_soft_enabled) {
            const auto values=residuals(std::vector<DM>{DM(current),DM(parameters)})[0].get_elements();
            const auto scaling=numbers(manifest["slack_scale"]);
            for(size_t k=0;k<values.size();++k)current[nvar+k]=std::max(values[k],0.)/scaling[k];
        }
        if(restored_soft && !center.is_null()) {
            const auto initial=trajectory(std::vector<DM>{DM(current),DM(parameters)});
            const auto coefficients=initial[2].get_elements(),times=initial[1].get_elements();
            double coefficient_error=0.,time_error=0.;
            for(int piece=0;piece<n;++piece) {
                time_error=std::max(time_error,std::abs(times[piece]-center["durations"][piece].get<double>()));
                for(int axis=0;axis<3;++axis)for(int power=0;power<8;++power) {
                    const double expected=center["coefficients"][piece][power][axis];
                    coefficient_error=std::max(coefficient_error,std::abs(coefficients[24*piece+8*axis+power]-expected)/(1.+std::abs(expected)));
                }
            }
            const double initial_objective=static_cast<double>(initial[4]);
            const double objective_error=std::abs(initial_objective-center["objective"].get<double>());
            const bool passed=coefficient_error<1e-8 && time_error<1e-10 && objective_error<1e-6;
            std::ofstream(destination+"/center_embedding.json")<<Json{{"passed",passed},{"coefficient_relative_error",coefficient_error},
                {"duration_max_error_s",time_error},{"objective_error",objective_error},{"center_time_s",center["nominal_time_s"]},
                {"free_initial_objective",initial_objective}}.dump(2)<<'\n';
            if(!passed)throw std::runtime_error("Center embedding does not preserve initial trajectory");
        }
        const std::vector<double> togt_initial=current;
        std::vector<double> physical=numbers(seed["original"]);
        std::vector<double> lam_x(current.size(),0.);std::map<std::string,double> lam_g;
        std::map<WitnessKey,Witness> witnesses;
        DynamicWitnesses dynamic_witnesses;
        if(dynamic_exchange)for(int piece=0;piece<n;++piece)for(int component=0;component<dynamic_components();++component)
            dynamic_witnesses[{piece,component}]={0.,.5,1.};
        if(dynamic_exchange && dynamics_metadata.value("initialization",std::string("three_points"))=="baseline_peaks") {
            auto reporting=exchange_to_original.is_null()?current:
                exchange_to_original(std::vector<DM>{DM(current),DM(parameters)})[0].get_elements();
            if(!native_minco_enabled)reporting.resize(nvar+numbers(manifest["slack_scale"]).size(),0.);
            const auto initial_curve=trajectory(std::vector<DM>{DM(reporting),DM(parameters)});
            const auto coeff=initial_curve[2].get_elements(),times=initial_curve[1].get_elements();
            for(int piece=0;piece<n;++piece) {
                Curve curve;for(int axis=0;axis<3;++axis)for(int power=0;power<8;++power)
                    curve(power,axis)=coeff[24*piece+8*axis+power];
                const auto check=check_dynamics(curve,times[piece]);
                for(int component=0;component<dynamic_components();++component)for(const auto& value:check["local_peak_fractions"][component]) {
                    const double u=value;auto& points=dynamic_witnesses[{piece,component}];
                    if(std::all_of(points.begin(),points.end(),[&](double old){return std::abs(old-u)>1e-7;}))points.push_back(u);
                }
            }
        }
        for(int gate=0;gate<m;++gate){witnesses[{gate,gate}]={"before",{0.}};witnesses[{gate,gate+1}]={"after",{}};}
        Json calls=Json::array(),records=Json::array(),projections=Json::array();bool found=false;
        std::string termination_reason="exchange_round_limit";
        Json selected;
        for(int round=0;round<24;++round) {
            if(round>0 && reset_togt_each_round) {
                current=togt_initial;
                lam_x.assign(current.size(),0.);
                lam_g.clear();
            }
            const bool warm_start=round>0 && !reset_togt_each_round;
            const auto build_begin=Clock::now();auto problem=build(witnesses,warm_start,dynamic_witnesses);
            const double build_s=std::chrono::duration<double>(Clock::now()-build_begin).count();
            Json canonicalized=Json::array();
            if(round>0 && (!native_minco_enabled || restored_soft))for(int gate=0;gate<static_cast<int>(manifest["slices"].size());++gate) {
                const auto& selection=manifest["slices"][gate];
                if(!selection["circle"].get<bool>())continue;
                const int begin=selection["begin"],end=selection["end"];double squared=numpy_dot(current.data()+begin,current.data()+begin,end-begin);
                if(squared>1.){for(int k=begin;k<end;++k)current[k]/=squared;canonicalized.push_back(gate);}
            }
            std::vector<double> lower_x(current.size(),-std::numeric_limits<double>::infinity());
            std::vector<double> upper_x(current.size(),std::numeric_limits<double>::infinity());
            if(!direct_soft && !sampled_hard)std::fill(lower_x.begin()+nvar,lower_x.end(),0.);
            if(dynamic_exchange && !native_minco_enabled) {
                if(!exchange_to_original.is_null())for(int k=0;k<n;++k) {
                    auto inverse=[](double value){return value>1.?std::sqrt(2.*value-1.)-1.:1.-std::sqrt(2./value-1.);};
                    lower_x[k]=inverse(manifest["lower_g"][workspace_rows+m+k].get<double>());
                    upper_x[k]=inverse(manifest["upper_g"][workspace_rows+m+k].get<double>());
                    if(phase_scaled_time) {
                        lower_x[k]=(lower_x[k]-time_centers[k])/time_scales[k];
                        upper_x[k]=(upper_x[k]-time_centers[k])/time_scales[k];
                    }
                }
            }
            if(direct_points_enabled && manifest["free_points"].get<bool>()) {
                for(int gate=0;gate<m;++gate)if(windows[gate].circle) {
                    const int begin=manifest["slices"][gate]["begin"];
                    lower_x[begin]=0.;upper_x[begin]=windows[gate].opening_radius+.085-manifest["finite_radius"].get<double>();
                    lower_x[begin+1]=0.;upper_x[begin+1]=2.*pi;
                }
            }
            auto starting=current,starting_lam_x=lam_x;
            if(lifted_thrust) {
                const auto values=Function("thrust_initial_curve",{expressions[0],expressions[1]},
                    {expressions[4],MX::exp(expressions[2](casadi::Slice(0,n)))})
                    (std::vector<DM>{DM(current),DM(parameters)});
                const auto coeff=values[0].get_elements(),times=values[1].get_elements();
                const double low=minimum_thrust::collective_floor/(dynamics_model->mass*dynamics_model->gravity);
                const double high=8.*dynamics_model->midpoint/(dynamics_model->mass*dynamics_model->gravity);
                for(const auto& [piece,u]:problem.thrust_nodes) {
                    Vector a=Vector::Zero();
                    for(int power=7;power>=2;--power)for(int axis=0;axis<3;++axis)
                        a[axis]=a[axis]*u+power*(power-1.)*coeff[24*piece+8*axis+power];
                    a/=times[piece]*times[piece];a[2]+=dynamics_model->gravity;
                    starting.push_back(std::clamp(a.norm()/dynamics_model->gravity,low+1e-8,high-1e-8));
                    lower_x.push_back(low);upper_x.push_back(high);starting_lam_x.push_back(0.);
                }
            }
            casadi::DMDict input{{"x0",DM(starting)},{"p",DM(parameters)},
                {"lbx",DM(lower_x)},{"ubx",DM(upper_x)},
                {"lbg",DM(problem.lower)},{"ubg",DM(problem.upper)}};
            if(warm_start) {
                std::vector<double> previous;for(const auto& key:problem.keys)previous.push_back(lam_g[key]);
                input["lam_x0"]=DM(starting_lam_x);input["lam_g0"]=DM(previous);
            }
            if(feasible_sqp) {
                auto projection_input=input;projection_input.erase("lam_x0");projection_input.erase("lam_g0");
                auto projection_parameters=parameters;
                projection_parameters.insert(projection_parameters.end(),starting.begin(),starting.end());
                projection_input["p"]=DM(projection_parameters);
                const auto projection_begin=Clock::now();
                const auto feasible=problem.projector(projection_input);
                const auto stats=problem.projector.stats();
                Json record{{"round",round},{"success",stats.at("success").to_bool()},
                    {"status",stats.at("return_status").to_string()},{"iterations",stats.at("iter_count").to_int()},
                    {"compute_s",std::chrono::duration<double>(Clock::now()-projection_begin).count()}};
                projections.push_back(record);progress<<Json{{"stage","feasible_initialization"},{"record",record}}.dump()<<std::endl;
                if(!stats.at("success").to_bool())break;
                input["x0"]=feasible.at("x");input["lam_x0"]=feasible.at("lam_x");input["lam_g0"]=feasible.at("lam_g");
            }
            if(std::getenv("SIP_DUMP_NLP"))
                std::ofstream(destination+"/finite_input.json")<<Json{{"round",round},
                    {"x0",input.at("x0").get_elements()},{"parameters",parameters},
                    {"lower_x",lower_x},{"upper_x",upper_x},{"lower_g",problem.lower},{"upper_g",problem.upper},
                    {"keys",problem.keys}}.dump(2)<<'\n';
            const auto solve_begin=Clock::now();const auto solution=problem.solver(input);
            const double solve_s=std::chrono::duration<double>(Clock::now()-solve_begin).count();
            const auto statistics=problem.solver.stats();
            current=solution.at("x").nonzeros();lam_x=solution.at("lam_x").nonzeros();
            if(lifted_thrust){current.resize(nvar);lam_x.resize(nvar);}
            const auto multipliers=solution.at("lam_g").nonzeros();lam_g.clear();
            for(size_t k=0;k<problem.keys.size();++k)lam_g[problem.keys[k]]=multipliers[k];
            std::vector<double> report_variables=current;
            if(!exchange_to_original.is_null())report_variables=
                exchange_to_original(std::vector<DM>{DM(current),DM(parameters)})[0].get_elements();
            if(direct_soft || sampled_hard) {
                if(direct_soft) {
                    const auto r=residuals(std::vector<DM>{DM(current),DM(parameters)})[0].get_elements();
                    const auto scale=numbers(manifest["slack_scale"]);
                    for(size_t k=0;k<r.size();++k)report_variables.push_back(std::max(r[k],0.)/scale[k]);
                } else if(!native_minco_enabled)report_variables.resize(current.size()+numbers(manifest["slack_scale"]).size(),0.);
            }
            const auto values=trajectory(std::vector<DM>{DM(report_variables),DM(parameters)});
            const auto original=values[0].get_elements(),times=values[1].get_elements(),coefficient_values=values[2].get_elements();
            std::vector<Curve> coefficients(n);Json curves=Json::array();
            for(int piece=0;piece<n;++piece) {
                for(int axis=0;axis<3;++axis)for(int power=0;power<8;++power)
                    coefficients[piece](power,axis)=coefficient_values[24*piece+8*axis+power];
                curves.push_back(matrix(coefficients[piece]));
            }
            const double penalty=native_soft_enabled?static_cast<double>(values[5]):sampled_hard?0.:static_cast<double>(values[5])/((direct_soft || unit_epigraph)?1000.:1.);
            const double total_time=static_cast<double>(DM::sum1(values[1]));
            const double actual_objective=sampled_hard?total_time:(direct_soft || unit_epigraph)?total_time+penalty:static_cast<double>(values[4]);
            const double epigraph_gap=native_soft_enabled?static_cast<double>(solution.at("f"))*parameters[0]-actual_objective:sampled_hard?static_cast<double>(solution.at("f"))*parameters[0]-actual_objective:
                (direct_soft || unit_epigraph)?static_cast<double>(solution.at("f"))*parameters[0]-actual_objective:0.;
            const auto g=solution.at("g").nonzeros();double minimum=std::numeric_limits<double>::infinity(),violation=0.;
            for(size_t k=0;k<g.size();++k) {
                if(static_cast<int>(k)<problem.geometry_rows)
                    minimum=std::min({minimum,g[k]-problem.lower[k],problem.upper[k]-g[k]});
                violation=std::max({violation,problem.lower[k]-g[k],g[k]-problem.upper[k]});
            }
            const bool success=statistics.at("success").to_bool();
            double maximum_dynamic_residual=-std::numeric_limits<double>::infinity();
            if(sampled_hard)for(size_t k=problem.dynamic_row_start;k<g.size();++k)
                maximum_dynamic_residual=std::max(maximum_dynamic_residual,g[k]);
            Json call{{"stage","factorized_finite_ipopt"},{"round",round},{"success",success},
                {"status",statistics.at("return_status").to_string()},{"iterations",statistics.at("iter_count").to_int()},
                {"solver_s",solve_s},{"problem_build_s",build_s},{"warm_start_used",warm_start},
                {"initial_reset_to_togt",round>0 && reset_togt_each_round},
                {"native_objective",actual_objective},{"dynamic_soft_integral",penalty},{"dynamic_penalty_contribution",actual_objective-total_time},{"epigraph_gap",epigraph_gap},{"constraint_violation_inf",violation},
                {"minimum_constraint",minimum},{"sampled_dynamic_max_residual",sampled_hard?Json(maximum_dynamic_residual):Json(nullptr)},
                {"solver_variables",solution.at("x").numel()},{"constraint_rows",g.size()},
                {"lifted_thrust_variables",problem.thrust_nodes.size()},
                {"dynamic_constraint_rows",g.size()-problem.dynamic_row_start},
                {"canonicalized_circle_gates",canonicalized},{"original_variables",original}};
            if(statistics.count("iterations")) {
                const auto history=statistics.at("iterations").to_dict();
                for(const auto& key:{"inf_pr","inf_du","mu","obj"})if(history.count(key)) {
                    const auto values=history.at(key).to_double_vector();
                    if(!values.empty())call[std::string("final_")+key]=values.back();
                }
            }
            calls.push_back(call);progress<<call.dump()<<std::endl;
            std::cout<<"ROUND "<<seed_index<<' '<<round<<' '<<call["status"]<<' '<<actual_objective<<' '<<solve_s<<std::endl;
            if((!success && !restored_soft) || minimum < -1e-5 || (sampled_hard && violation>1e-5) ||
                !std::all_of(current.begin(),current.end(),[](double value){return std::isfinite(value);})) {
                termination_reason=minimum < -1e-5 ? "finite_node_geometry_violation" : "finite_solve_not_ready";
                break;
            }
            std::vector<double> physical_times=times;
            if(native_minco_enabled) {
                physical=original;
            } else {
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
                physical_times.clear();for(int k=0;k<n;++k)physical_times.push_back(std::exp(physical[k]));
                const auto physical_values=physical_curve(std::vector<DM>{DM(physical)})[0].get_elements();
                for(int piece=0;piece<n;++piece)for(int axis=0;axis<3;++axis)for(int power=0;power<8;++power)
                    coefficients[piece](power,axis)=physical_values[24*piece+8*axis+power];
            }
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
            Json dynamic_checks=Json::array();bool dynamic_passed=true;int dynamic_added=0;
            if(dynamic_exchange)for(int piece=0;piece<n;++piece) {
                auto check=check_dynamics(coefficients[piece],physical_times[piece]);
                check["piece"]=piece;dynamic_passed=dynamic_passed&&check["passed"].get<bool>();
                for(int component=0;component<dynamic_components();++component) {
                    const double value=check["peak_residuals"][component];
                    if(value<=dynamics_metadata["verification_residual_tolerance"].get<double>())continue;
                    const double u=check["peak_fractions"][component];
                    auto& points=dynamic_witnesses[{piece,component}];
                    if(std::all_of(points.begin(),points.end(),[&](double old){return std::abs(old-u)>1e-7;})) {
                        points.push_back(u);++dynamic_added;added=true;
                    }
                }
                dynamic_checks.push_back(check);
            }
            Json record{{"round",round},{"optimizer_converged",success},{"global_passed",passed},
                {"dynamics_passed",dynamic_passed},{"dynamic_added_rows",dynamic_added},
                {"checks",checks},{"dynamic_checks",dynamic_checks}};
            records.push_back(record);
            if(dynamic_exchange) {
                progress<<Json{{"stage","continuous_checks"},{"record",record}}.dump()<<std::endl;
                std::cout<<"DYNAMIC_CHECK "<<round<<' '<<dynamic_passed<<' '<<dynamic_added<<std::endl;
            }
            if(passed && dynamic_passed && success) {
                termination_reason="verified_solution";found=true;Json points=Json::array();const auto point_values=values[3].get_elements();
                for(int gate=0;gate<m;++gate)points.push_back({point_values[3*gate],point_values[3*gate+1],point_values[3*gate+2]});
                std::vector<double> crossings;double total=0.;for(int k=0;k<n;++k){total+=physical_times[k];if(k<m)crossings.push_back(total);}
                curves=Json::array();for(const auto& coefficient:coefficients)curves.push_back(matrix(coefficient));
                selected={{"variables",physical},{"durations",physical_times},{"coefficients",curves},{"nominal_time_s",total},
                    {"objective",actual_objective},{"dynamic_soft_integral",penalty},{"crossing_times_s",crossings},{"crossing_points",points},{"made_at",0.}};
                break;
            }

            if(restored_soft && passed && !success)continue;
            if(!added){termination_reason="no_new_witness";break;}
        }
        Json result=found?selected:Json::object();
        result.update(Json{{"found",found},{"termination_reason",termination_reason},{"solve_s",std::chrono::duration<double>(Clock::now()-began).count()},
            {"optimizer_calls",calls},{"records",records},{"feasible_initializations",projections},{"config",manifest["config"]},{"level",manifest["level"]},
            {"method",manifest["method"]},{"direct_soft_objective",direct_soft},{"unit_weight_epigraph",unit_epigraph},
            {"sampled_hard_dynamics",sampled_hard},{"global_uniform_sample_count",hard_nodes},
            {"reset_to_togt_after_exchange",reset_togt_each_round},
            {"semi_infinite_dynamics",dynamic_exchange},
            {"minimum_collective_thrust_n",minimum_thrust_enabled?Json(minimum_thrust::collective_floor):Json(nullptr)},
            {"stable_dynamics_extension",stable_dynamics_enabled},
            {"direct_pvaj_coordinates",direct_pvaj && manifest.value("minco_derivative_order",3)==3},
            {"direct_state_coordinates",direct_pvaj},
            {"native_minco_pvaj",native_minco_enabled && manifest.value("minco_derivative_order",3)==3},
            {"restored_historical_soft_workflow",restored_soft},
            {"iteration_limit_action",restored_soft?"allow finite feasible candidate verification and exchange":"terminate initialization before verification and exchange"},
            {"native_minco_pv",native_minco_enabled && manifest.value("minco_derivative_order",3)==1},
            {"native_soft_dynamics",native_soft_enabled},
            {"point_coordinates",direct_points_enabled?"direct_polar_polygon":"TOGT maps"},
            {"point_slices",manifest["slices"]},{"trajectory_variable_count",nvar},
            {"initial_angle_rule",direct_points_enabled?"negative physical objective gradient at zero radius":"legacy"},
            {"workspace_constraints",!native_minco_enabled || restored_soft},
            {"segment_duration_bounds",native_minco_enabled && !restored_soft?Json(nullptr):Json("legacy bounds")},
            {"warm_start_raw_variables_preserved",native_minco_enabled && !restored_soft},
            {"dynamic_auxiliary_variables",native_soft_enabled ? n*33*6 : 0},
            {"minco_derivative_order",native_minco_enabled?Json(manifest.value("minco_derivative_order",3)):Json(nullptr)},
            {"baseline_offset_used",!native_minco_enabled && !direct_pvaj},
            {"legacy_trajectory_conversion_used",!native_minco_enabled},
            {"phase_scaled_time_coordinates",phase_scaled_time},
            {"feasible_sqp",feasible_sqp},
            {"norm_form_dynamics",norm_dynamics},
            {"bundled_dynamic_constraints",norm_dynamics},
            {"lifted_positive_thrust",lifted_thrust},
            {"redundant_knot_safety_eliminated",norm_dynamics && manifest["free_points"].get<bool>()},
            {"minimum_thrust_at_all_dynamic_nodes",stable_dynamics_enabled},
            {"exact_hessian",stable_dynamics_enabled && !std::getenv("SIP_LIMITED_MEMORY")},
            {"stable_exchange_coordinates",!exchange_to_original.is_null()},
            {"dynamics_verification",dynamic_exchange?"numerical interval enclosure on every polynomial segment":native_soft_enabled?"soft penalty, 33 nodes per segment":"fixed temporal samples"},
            {"time_weight",1.},{"dynamics_weight",native_soft_enabled?native_dynamics_weight:sampled_hard?0.:(direct_soft || unit_epigraph)?1.:1000.},{"initial_sample_index",seed_index},{"initial_variables",seed["original"]},
            {"runtime","C++17 CasADi/IPOPT and native continuous frame exchange"},{"all_seven_joint",true},
            {"replanning",false},{"dynamic_limit_fraction",.95},{"finite_safety_radius_m",manifest["finite_radius"]},
            {"verification_safety_radius_m",manifest["verification_radius"]}});
        return result;
    }
};

int main(int argc,char** argv) {
    try {
        if(argc!=4 && argc!=5 && argc!=6 && argc!=7)throw std::runtime_error("Usage: sip_planner GRAPH_DIRECTORY OUTPUT_DIRECTORY SEED_INDEX [--direct-soft|--epigraph-weight-one|--sampled-hard|--sampled-hard-reset-togt|--exchange-hard-reset-togt|--exchange-hard-min-thrust|--exchange-hard-stable|--exchange-hard-pvaj|--exchange-hard-norm|--exchange-hard-lifted|--exchange-hard-scaled|--exchange-hard-feasible]");
        const bool restored_mode=(argc==6 || argc==7) && (std::string(argv[4])=="--restored-pv" || std::string(argv[4])=="--restored-pvaj");
        const bool pv_mode=(argc==6 || argc==7) && (std::string(argv[4])=="--soft-mincopv-pv" || std::string(argv[4])=="--restored-pv");
        const bool direct_points=(argc==6 || argc==7) && (std::string(argv[4])=="--soft-mincopv-direct" || (pv_mode && !restored_mode));
        const bool native_soft=(argc==6 || argc==7) && (std::string(argv[4])=="--soft-mincopv" || direct_points || restored_mode);
        const bool native_hard=argc==6 && std::string(argv[4])=="--exchange-hard-mincopv";
        const bool native=native_soft || native_hard;
        if((argc==6 || argc==7) && !native)throw std::runtime_error("Unknown six-argument mode");
        const double native_weight=argc==7 ? std::stod(argv[6]) : 1.;
        const bool diagnostic=argc==5 && std::string(argv[4])=="--direct-check";
        const bool direct=argc==5 && (std::string(argv[4])=="--direct-soft" || diagnostic);
        const bool unit=argc==5 && std::string(argv[4])=="--epigraph-weight-one";
        const bool feasible_method=argc==5 && std::string(argv[4])=="--exchange-hard-feasible";
        const bool scaled_time=argc==5 && (std::string(argv[4])=="--exchange-hard-scaled" || feasible_method);
        const bool lift=argc==5 && std::string(argv[4])=="--exchange-hard-lifted";
        const bool norm_form=argc==5 && (std::string(argv[4])=="--exchange-hard-norm" || lift || scaled_time);
        const bool direct_states=argc==5 && (std::string(argv[4])=="--exchange-hard-pvaj" || norm_form);
        const bool stable=argc==5 && (std::string(argv[4])=="--exchange-hard-stable" || direct_states);
        const bool minimum_thrust=native_hard || (argc==5 && (std::string(argv[4])=="--exchange-hard-min-thrust" || stable));
        const bool exchange=native_hard || (argc==5 && (std::string(argv[4])=="--exchange-hard-reset-togt" || minimum_thrust));
        const bool reset=native_hard || (argc==5 && (std::string(argv[4])=="--sampled-hard-reset-togt" || (exchange && !norm_form)));
        const bool hard=native_hard || (argc==5 && (std::string(argv[4])=="--sampled-hard" || reset || exchange));
        if(argc==5 && !direct && !unit && !hard)throw std::runtime_error("Unknown option");
        std::filesystem::create_directories(argv[2]);
        Planner planner(argv[1],argv[2],direct,unit,hard,reset,exchange,minimum_thrust,stable,direct_states,norm_form,lift,scaled_time,feasible_method,native?argv[5]:"",native_soft,native_weight,direct_points,pv_mode?1:3,restored_mode);Json center;
        if(diagnostic)return 0;
        const std::string seed=argv[3];
        if(seed.rfind("release=",0)==0)std::ifstream(seed.substr(8))>>center;
        const Json result=planner.solve(center.is_null()?std::stoi(seed):8,center);
        std::ofstream(std::string(argv[2])+"/result.json")<<result.dump(2)<<'\n';
    } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
