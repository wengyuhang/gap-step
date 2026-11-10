#include <casadi/casadi.hpp>
#include "trajectory.hpp"
#include <chrono>
#include <filesystem>
#include <optional>
using casadi::DM;
using casadi::Function;
extern "C" int wang_optimize(int,double*,double(*)(const double*,double*,int),double,int,double,double,double*);
extern "C" const char* wang_status(int);
thread_local Function* active_function=nullptr;
thread_local std::vector<double>* active_parameters=nullptr;
thread_local bool evaluation_error=false;

double native_callback(const double* x,double* gradient,int count) {
    try {
        auto outputs=(*active_function)(std::vector<DM>{DM(std::vector<double>(x,x+count)),DM(*active_parameters)});
        const auto derivative=outputs[1].get_elements();std::copy(derivative.begin(),derivative.end(),gradient);
        return static_cast<double>(outputs[0]);
    } catch(const std::exception&) {
        evaluation_error=true;std::fill(gradient,gradient+count,NAN);return NAN;
    }
}

Eigen::Matrix<double,3,4> load_state(const Json& input) {
    Eigen::Matrix<double,3,4> result;for(int i=0;i<3;++i)for(int j=0;j<4;++j)result(i,j)=input[i][j];return result;
}
Vector load_vector(const Json& input) {return Vector(input[0],input[1],input[2]);}

std::vector<double> level_roots(const Trajectory& trajectory,const Window& window,const Vector& normal,double level) {
    std::vector<double> candidates;double start=trajectory.start;
    for(size_t piece=0;piece<trajectory.durations.size();++piece) {
        Eigen::VectorXd polynomial=trajectory.coefficients[piece]*normal;polynomial[0]-=window.center.dot(normal)+level;
        auto values=roots(polynomial);
        if(std::abs(polynomial[0])<1e-7)values.push_back(0.);
        if(std::abs(evaluate(trajectory.coefficients[piece],1.).dot(normal)-window.center.dot(normal)-level)<1e-7)values.push_back(1.);
        for(double u:values)if(u>=-1e-8 && u<=1.+1e-8)candidates.push_back(start+std::clamp(u,0.,1.)*trajectory.durations[piece]);
        start+=trajectory.durations[piece];
    }
    std::sort(candidates.begin(),candidates.end());std::vector<double> result;
    for(double at:candidates)if(result.empty() || std::abs(at-result.back())>1e-6)result.push_back(at);
    return result;
}
Json crossing_event(const Trajectory& trajectory,const Window& window,const Vector& normal,double depth) {
    std::optional<double> center;
    for(double at:level_roots(trajectory,window,normal,0.)) {
        if(at<=trajectory.start+1e-7)continue;
        const Vector local=window.rotation.transpose()*(trajectory.sample(at)-window.center)-window.motion(at);
        if(!window.contains(local.head<2>()))continue;
        if(trajectory.sample(at,1).dot(normal)<=0.)return Json{{"passed",false},{"failure","reverse_current_aperture_crossing"},{"time_s",at}};
        center=at;break;
    }
    if(!center)return Json{{"passed",false},{"failure","no_forward_current_aperture_crossing"}};
    for(double at:level_roots(trajectory,window,normal,depth))
        if(at>*center && trajectory.sample(at,1).dot(normal)>0.)return Json{{"passed",true},{"center_crossing_s",*center},{"body_exit_s",at}};
    return Json{{"passed",false},{"failure","no_complete_body_exit"},{"center_crossing_s",*center}};
}

int main(int argc,char** argv) {
    try {
        if(argc!=4 && argc!=5)throw std::runtime_error("Usage: wang_planner GRAPH_DIRECTORY OUTPUT_DIRECTORY verify|solve [COMPILED_DIRECTORY]");
        const std::string directory=argv[1],output=argv[2],mode=argv[3];
        Json manifest;std::ifstream(directory+"/manifest.json")>>manifest;
        std::filesystem::create_directories(output);std::vector<Function> functions,details;std::vector<Window> windows;
        for(int gate=0;gate<7;++gate) {
                        if(argc==5)functions.push_back(casadi::external("paper_fg",std::string(argv[4])+"/fg_"+std::to_string(gate)+".so"));
            else functions.push_back(Function::load(directory+"/fg_"+std::to_string(gate)+".casadi"));
            details.push_back(Function::load(directory+"/details_"+std::to_string(gate)+".casadi"));
            windows.push_back(load_window(manifest["windows"][gate]));
        }
        if(mode=="verify") {
            double maximum=0.;
            for(int gate=0;gate<7;++gate) {
                const auto& check=manifest["checks"][gate];
                const auto values=functions[gate](std::vector<DM>{DM(check["seed"].get<std::vector<double>>()),DM(check["parameters"].get<std::vector<double>>())});
                maximum=std::max(maximum,std::abs(static_cast<double>(values[0])-check["objective"].get<double>())/(1.+std::abs(check["objective"].get<double>())));
                const auto derivative=values[1].get_elements();for(size_t k=0;k<derivative.size();++k)
                    maximum=std::max(maximum,std::abs(derivative[k]-check["gradient"][k].get<double>())/(1.+std::abs(check["gradient"][k].get<double>())));
            }
            if(maximum>1e-9)throw std::runtime_error("Wang native function parity failed");
            std::ofstream(output+"/validation.json")<<Json{{"passed",true},{"relative_error",maximum}}.dump(2)<<'\n';return 0;
        }
        if(mode!="solve")throw std::runtime_error("Unknown Wang mode");
        const auto began=std::chrono::steady_clock::now();
        const auto& cfg=manifest["config"];const auto& reference=manifest["reference"];
        const int dimension=manifest["dimension"],state_end=manifest["state_end"],pieces=cfg["pieces"];
        const double derivative_scale=cfg["derivative_scale"],position_scale=cfg["position_scale"];
        const Trajectory route=Trajectory::load(reference);Trajectory complete;
        Eigen::Matrix<double,3,4> head=Eigen::Matrix<double,3,4>::Zero();head.col(0)=load_vector(manifest["start"]);
        Json requests=Json::array();std::string failure;double now=0.;
        for(int gate=0;gate<7;++gate) {
            const auto request_begin=std::chrono::steady_clock::now();
            const auto tail=load_state(reference["tail_states"][gate]);
            std::vector<Trajectory> candidates;Json records=Json::array();std::vector<int> eligible;
            std::vector<double> parameters{now};for(const auto& state:{head,tail})for(int d=0;d<4;++d)for(int axis=0;axis<3;++axis)parameters.push_back(state(axis,d));
            int seed_index=0;
            for(double scale:manifest["settings"]["temporal_scales"].get<std::vector<double>>()) {
                const double begin=reference["node_times"][gate],middle=reference["node_times"][gate+1],end=reference["tail_times"][gate];
                const double spans[2]={middle-begin,end-middle},durations[2]={spans[0]*scale,spans[1]*scale};
                const Vector center=windows[gate].center+windows[gate].rotation*windows[gate].motion(now+durations[0]);
                std::vector<double> seed(dimension,0.);seed[0]=std::log(durations[0]);seed[1]=std::log(durations[1]);
                for(int d=1;d<4;++d) {
                    const Vector velocity=route.sample(middle,d)*std::pow(spans[0]/durations[0],d)/derivative_scale;
                    for(int axis=0;axis<3;++axis)seed[2+3*(d-1)+axis]=velocity[axis];
                }
                for(int leg=0;leg<2;++leg) {
                    const double a=leg==0?begin:middle,b=leg==0?middle:end;
                    const Vector start=leg==0?Vector(head.col(0)):center,finish=leg==0?center:Vector(tail.col(0));
                    for(int k=1;k<pieces;++k) {
                        const double fraction=static_cast<double>(k)/pieces;
                        const double blend=35*std::pow(fraction,4)-84*std::pow(fraction,5)+70*std::pow(fraction,6)-20*std::pow(fraction,7);
                        const Vector point=(route.sample(a+fraction*(b-a))+(1.-blend)*(start-route.sample(a))+blend*(finish-route.sample(b)))/position_scale;
                        for(int axis=0;axis<3;++axis)seed[state_end+3*(pieces-1)*leg+3*(k-1)+axis]=point[axis];
                    }
                }
                auto x=seed;double summary[5]={0.};active_function=&functions[gate];active_parameters=&parameters;evaluation_error=false;
                const int status=wang_optimize(dimension,x.data(),native_callback,cfg["solve_budget_ms"],cfg["max_iterations"],cfg["gradient_tolerance"],cfg["relative_tolerance"],summary);
                const auto outputs=details[gate](std::vector<DM>{DM(x),DM(parameters)});const auto segment_times=outputs[1].get_elements();
                const auto coefficient_values=outputs[0].get_elements(),terms=outputs[2].get_elements(),residuals=outputs[3].get_elements();
                Trajectory curve;curve.start=now;
                for(int leg=0;leg<2;++leg)for(int k=0;k<pieces;++k)curve.durations.push_back(segment_times[leg]/pieces);
                for(int piece=0;piece<2*pieces;++piece) {
                    Curve c;for(int axis=0;axis<3;++axis)for(int power=0;power<8;++power)c(power,axis)=coefficient_values[axis*16*pieces+8*piece+power];
                    curve.coefficients.push_back(c);
                }
                bool finite=std::isfinite(summary[0])&&!evaluation_error;
                for(double value:terms)finite=finite&&std::isfinite(value);
                for(double value:residuals)finite=finite&&std::isfinite(value);
                for(double value:x)finite=finite&&std::isfinite(value);
                bool valid=finite&&(status==0 || status==1);
                for(const auto& c:curve.coefficients)valid=valid&&c.allFinite();
                for(double value:curve.durations)valid=valid&&std::isfinite(value)&&value>0.;
                if(valid)eligible.push_back(seed_index);
                Json coefficients=Json::array();for(const auto& c:curve.coefficients)coefficients.push_back(curve_json(c));
                records.push_back(Json{{"stats",Json{{"seed",seed_index},{"scale",scale},{"eligible",valid},{"converged",status==0 || status==1},
                    {"code",status},{"status",wang_status(status)},{"finite",finite},{"objective_scaled",summary[0]},
                    {"evaluations",static_cast<int>(summary[1])},{"iterations",static_cast<int>(summary[2])},{"optimizer_ms",summary[3]},
                    {"variables",x},{"initial_variables",seed}}},{"durations",segment_times},{"coefficients",coefficients}});
                candidates.push_back(curve);++seed_index;
            }
            std::optional<int> selected;
            for(int index:eligible)if(!selected || records[index]["stats"]["objective_scaled"].get<double>()<records[*selected]["stats"]["objective_scaled"].get<double>())selected=index;
            Json request{{"gate",gate+1},{"race_time_s",now},{"head_state",state_json(head)},{"tail_state",state_json(tail)},
                {"terminal_choice",reference["tail_choices"][gate]},{"selected_seed",selected?Json(*selected):Json(nullptr)},{"candidates",records}};
            if(!selected)failure="no_converged_local_candidate";
            if(selected) {
                const auto& trajectory=candidates[*selected];
                auto event=crossing_event(trajectory,windows[gate],load_vector(reference["normals"][gate]),reference["body_exit_depths"][gate]);request["event"]=event;
                if(!event["passed"].get<bool>())failure=event["failure"];
                else {
                    const double end=gate==6?trajectory.end():event["body_exit_s"].get<double>();double begin=trajectory.start;
                    for(size_t piece=0;piece<trajectory.durations.size();++piece) {
                        const double stop=std::min(end,begin+trajectory.durations[piece]);
                        if(stop>begin+1e-10) {
                            const double fraction=std::clamp((stop-begin)/trajectory.durations[piece],0.,1.);Curve c=trajectory.coefficients[piece];
                            for(int power=0;power<8;++power)c.row(power)*=std::pow(fraction,power);
                            complete.coefficients.push_back(c);complete.durations.push_back(stop-begin);
                        }
                        begin+=trajectory.durations[piece];if(begin>=end-1e-10)break;
                    }
                    now=end;for(int d=0;d<4;++d)head.col(d)=trajectory.sample(now,d);
                }
            }
            request["cycle_ms"]=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-request_begin).count();requests.push_back(request);
            std::cout<<"WANG_GATE "<<gate+1<<' '<<now<<' '<<failure<<std::endl;if(!failure.empty())break;
        }
        Json coefficients=Json::array();for(const auto& c:complete.coefficients)coefficients.push_back(curve_json(c));
        Json result{{"found",failure.empty()},{"failure",failure.empty()?Json(nullptr):Json(failure)},
            {"case",Json{{"layout","togt_whole"},{"level",manifest["level"]}}},{"method","wang2023"},
            {"solve_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()},
            {"nominal_time_s",complete.end()},{"coefficients",coefficients},{"durations",complete.durations},{"requests",requests},
            {"variables",Json::array()},{"reference_seed",reference},{"wang_config",cfg},{"config",cfg},
            {"made_at",0.},{"scenario_variant","higher_swing_length_2p20"},{"runtime","C++ unchanged pinned LBFGS-Lite and paper model graphs"}};
        result["config"]["radius"]=.3859447197013662;
        result["code_generated_runtime"]=argc==5;
        if(argc==5){result["config"]["compile_functions"]=true;result["wang_config"]["compile_functions"]=true;}std::ofstream(output+"/result.json")<<result.dump(2)<<'\n';
    } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
