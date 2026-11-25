#include "native_minco_pvaj.hpp"
#include "restored_kernels.hpp"
#include <iostream>
#include <Eigen/SVD>
using casadi::DM;using casadi::MX;using casadi::Function;
Json read(const std::string& p){Json j;std::ifstream(p)>>j;return j;}
int main(int argc,char**argv){try{
 if(argc!=5 && argc!=6)throw std::runtime_error("INPUT SCENE CANDIDATE OUTPUT");
 auto m=read(std::string(argv[1])+"/manifest.json"),s=read(argv[2]),r=read(std::string(argv[3])+"/result.json"),in=read(std::string(argv[3])+"/finite_input.json"),model=read(std::string(argv[1])+"/exchange_dynamics.json");
 native_minco::remove_domain_bounds(m);native_minco::use_derivative_order(m,3);
 m["dynamic_residual_scale"]=model["residual_scale"];m["native_soft_formulation"]="direct";m["slack_scale"]=Json::array();m["duration_normalized_states"]=r.value("duration_normalized_states",false);
 for(const char* key:{"lift_minco_coefficients","lift_world_points","lift_arrival_times","bernstein_coefficient_coordinates","hermite_dynamics_evaluation","feature_coordinate_equilibration","native_hessian"})m.erase(key);
 model["continuous_attitude_chart"]=r.value("continuous_attitude_chart",false);
 std::vector<Window>w;for(const auto&row:m["windows"])w.push_back(load_window(row));
 const auto node=restored_kernels::dynamic_node(model);native_minco::Graph g(m,s,w,node,1000.);
 auto x=r["finite_incumbent"]["solver_variables"].get<std::vector<double>>();
 const auto p=in["parameters"].get<std::vector<double>>();
 const bool inclusion=argc==6 && std::string(argv[5])=="--embed-center-for-evaluation";
 if(inclusion){if(!m["free_points"].get<bool>())throw std::runtime_error("Inclusion audit requires free graph");const int n=m["n"],vs=m["velocity_start"];if(x.size()!=m["nvar"].get<int>()-(vs-n))throw std::runtime_error("Center result dimension mismatch");x.insert(x.begin()+n,vs-n,0.);}
 if(!inclusion){if(x.size()<m["nvar"].get<int>())throw std::runtime_error("Candidate has too few primary variables");x.resize(m["nvar"].get<int>());}

 Function f("audit",{g.expressions[0],g.expressions[1]},{g.expressions[5],MX::gradient(g.expressions[5],g.expressions[0]),g.expressions[6],MX::jacobian(g.expressions[6],g.expressions[0])});
 auto v=f(std::vector<DM>{DM(x),DM(p)});auto grad=v[1].get_elements();double err=0.;int worst=-1;Json bad=Json::array();
 Json scales=Json::array();std::vector<double> best(x.size(),INFINITY);
 for(double step:{1e-5,1e-6,1e-7,1e-8,1e-9,1e-10,1e-11,1e-12}){double maximum=0.;for(int col=0;col<x.size();++col){auto a=x,b=x;const double h=step*(1.+std::abs(x[col]));a[col]+=h;b[col]-=h;const double fd=(double(f(std::vector<DM>{DM(a),DM(p)})[0])-double(f(std::vector<DM>{DM(b),DM(p)})[0]))/(2.*h);const double e=std::abs(fd-grad[col])/std::max({1.,std::abs(fd),std::abs(grad[col])});maximum=std::max(maximum,e);best[col]=std::min(best[col],e);}scales.push_back({{"step",step},{"maximum_error",maximum}});}
 for(int col=0;col<x.size();++col){if(best[col]>err){err=best[col];worst=col;}if(best[col]>1e-3)bad.push_back({{"column",col},{"ad",grad[col]},{"best_error",best[col]}});}

 const int n=m["n"],vs=m["velocity_start"];Json groups=Json::array();
 for(auto range:std::vector<std::pair<int,int>>{{0,n},{n,vs},{vs,int(x.size())}}){double mx=0.,l2=0.;for(int j=range.first;j<range.second;++j){mx=std::max(mx,std::abs(grad[j]));l2+=grad[j]*grad[j];}groups.push_back({{"begin",range.first},{"end",range.second},{"gradient_inf",mx},{"gradient_l2",std::sqrt(l2)}});}
 auto evaluated=g.trajectory(std::vector<DM>{DM(x),DM(p)});
 const double original_f=r["objective"],evaluated_f=double(evaluated[4]);
 double coeff_error=0.;const auto coeff=evaluated[2].get_elements();
 for(int piece=0;piece<m["n"];++piece)for(int axis=0;axis<3;++axis)for(int power=0;power<8;++power){const double old=r["coefficients"][piece][power][axis];coeff_error=std::max(coeff_error,std::abs(coeff[piece*24+axis*8+power]-old)/(1.+std::abs(old)));}
 Json chart_nodes=Json::array();double min_chart_distance=INFINITY,min_force=INFINITY;int fallback_nodes=0;
 const int pieces=m["n"];for(int piece=0;piece<pieces;++piece){const double duration=double(evaluated[1](piece));for(int node=0;node<=16;++node){const double u=double(node)/16;Vector force;for(int axis=0;axis<3;++axis){double value=0.;for(int power=7;power>=2;--power)value=value*u+power*(power-1)*coeff[piece*24+axis*8+power];force[axis]=value/(duration*duration);}force[2]+=model["gravity"].get<double>();const double mag=force.norm(),denominator=1.+force[2]/mag;min_force=std::min(min_force,mag);min_chart_distance=std::min(min_chart_distance,std::abs(denominator-.001));if(denominator<=.001)++fallback_nodes;if(std::abs(denominator-.001)<1e-6)chart_nodes.push_back({{"piece",piece},{"u",u},{"denominator",denominator},{"distance_to_switch",std::abs(denominator-.001)},{"force_acceleration_norm",mag}});}}
 Json report{{"chart_switch_distance",min_chart_distance},{"fallback_node_count",fallback_nodes},{"near_chart_switch_nodes",chart_nodes},{"minimum_force_acceleration_norm",min_force},{"cross_method_result_only_evaluated_not_used_as_initialization",inclusion},{"original_objective",original_f},{"evaluated_objective",evaluated_f},{"objective_relative_difference",std::abs(evaluated_f-original_f)/(1.+std::abs(original_f))},{"coefficient_relative_difference",coeff_error},{"objective_normalized",double(v[0])},{"gradient_groups",groups},{"maximum_relative_fd_error",err},{"worst_column",worst},{"bad_columns",bad},{"base_constraint_count",v[2].numel()},{"all_column_derivatives_pass",err<1e-3},{"scale_errors",scales}};
 std::ofstream(argv[4])<<report.dump(2)<<'\n';std::cout<<report.dump(2)<<'\n';return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
