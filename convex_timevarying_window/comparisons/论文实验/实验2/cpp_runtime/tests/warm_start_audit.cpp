#include "native_minco_pvaj.hpp"
#include "restored_kernels.hpp"
#include <iostream>
using casadi::DM;
Json read(const std::string& p){Json j;std::ifstream(p)>>j;return j;}
int main(int argc,char**argv){try{
 if(argc!=6)throw std::runtime_error("INPUT SCENE FINITE_INPUT ACTUAL_CALLBACK OUTPUT");
 auto m=read(std::string(argv[1])+"/manifest.json"),scene=read(argv[2]),model=read(std::string(argv[1])+"/exchange_dynamics.json"),input=read(argv[3]),actual=read(argv[4]);
 native_minco::remove_domain_bounds(m);native_minco::use_derivative_order(m,3);
 m["native_soft_formulation"]="direct";m["slack_scale"]=Json::array();m["dynamic_residual_scale"]=model["residual_scale"];
 for(const char* key:{"lift_minco_coefficients","lift_world_points","lift_arrival_times","bernstein_coefficient_coordinates","hermite_dynamics_evaluation","feature_coordinate_equilibration","native_hessian"})m.erase(key);
 model["continuous_attitude_chart"]=true;std::vector<Window> windows;for(const auto&w:m["windows"])windows.push_back(load_window(w));
 native_minco::Graph graph(m,scene,windows,restored_kernels::dynamic_node(model),1000.);
 auto before=input["x0"].get<std::vector<double>>(),after=actual["x"].get<std::vector<double>>();
 int nv=m["nvar"],n=m["n"],vs=m["velocity_start"];
 if(before.size()!=after.size() || before.size()<nv)throw std::runtime_error("Mismatched iterate dimensions");
 Json differences=Json::array();for(auto range:std::vector<std::pair<int,int>>{{0,n},{n,vs},{vs,nv},{nv,nv+n*17*6},{nv+n*17*6,int(before.size())}}){double maximum=0.;for(int k=range.first;k<range.second;++k)maximum=std::max(maximum,std::abs(after[k]-before[k]));differences.push_back({{"begin",range.first},{"end",range.second},{"maximum_coordinate_change",maximum}});}
 before.resize(nv);after.resize(nv);const DM p(input["parameters"].get<std::vector<double>>());
 const auto b=graph.trajectory(std::vector<DM>{DM(before),p}),a=graph.trajectory(std::vector<DM>{DM(after),p});
 Json out{{"note","Read-only reconstruction in ORIGINAL unlifted MincoPV; no optimizer calls. Actual callback is the solver's first iterate for the selected finite problem."},{"round",input["round"]},{"coordinate_changes",differences},{"before_original_objective",double(b[4])},{"after_original_objective",double(a[4])},{"before_soft_integral",double(b[5])},{"after_soft_integral",double(a[5])},{"actual_auxiliary_objective",actual["f"][0].get<double>()*double(p(0))}};
 std::ofstream(argv[5])<<out.dump(2)<<'\n';std::cout<<out.dump(2)<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
