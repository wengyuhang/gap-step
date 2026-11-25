#include "native_minco_pvaj.hpp"
#include "restored_kernels.hpp"
#include <iostream>
#include <Eigen/Eigenvalues>
using casadi::DM;using casadi::MX;using casadi::Function;
Json read(const std::string& file){Json j;std::ifstream(file)>>j;return j;}
int main(int argc,char**argv){try{
 if(argc!=5)throw std::runtime_error("INPUT SCENE RESULT_OR_RAW OUTPUT");
 auto m=read(std::string(argv[1])+"/manifest.json"),scene=read(argv[2]),dyn=read(std::string(argv[1])+"/exchange_dynamics.json");
 native_minco::remove_domain_bounds(m);native_minco::use_derivative_order(m,3);
 m["native_soft_formulation"]="direct";m["slack_scale"]=Json::array();m.erase("native_hessian");m.erase("lift_world_points");m.erase("lift_arrival_times");m.erase("lift_minco_coefficients");
 m["dynamic_residual_scale"]=dyn["residual_scale"];dyn["continuous_attitude_chart"]=true;
 int n=m["n"],nv=m["nvar"],vs=m["velocity_start"];
 auto x=m["initials"][0]["original"].get<std::vector<double>>();int seed=0;
 if(std::string(argv[3])!="raw"){auto r=read(argv[3]);x=r["optimizer_calls"].back()["original_variables"].get<std::vector<double>>();seed=r["initial_sample_index"];}
 DM p(m["initials"][seed]["parameters"].get<std::vector<double>>());
 for(int k=0;k<n;++k){double t=std::exp(x[k]);x[k]=t>1.?std::sqrt(2*t-1)-1:1-std::sqrt(2/t-1);}
 if(m["free_points"].get<bool>())for(int g=0;g<m["m"];++g)for(int a=0;a<m["point_scale_m"][g].size();++a)x[m["slices"][g]["begin"].get<int>()+a]/=m["point_scale_m"][g][a].get<double>();
 for(int k=vs;k<nv;++k)x[k]/=m["state_scale"][k-vs].get<double>();
 std::vector<Window>w;for(const auto& row:m["windows"])w.push_back(load_window(row));
 native_minco::Graph graph(m,scene,w,restored_kernels::dynamic_node(dyn,m.value("hermite_dynamics_evaluation",false)),1000.);
 MX z=graph.expressions[0],par=graph.expressions[1],f=graph.expressions[5],grad=MX::gradient(f,z);
 Function fg("objective_gradient",{z,par},{f,grad});Function hf("objective_hessian",{z,par},{MX::densify(MX::jacobian(grad,z))});
 auto value=fg(std::vector<DM>{DM(x),p});auto gradient=value[1].get_elements();auto hv=hf(std::vector<DM>{DM(x),p})[0].get_elements();
 Eigen::Map<const Eigen::MatrixXd> h(hv.data(),nv,nv);Eigen::VectorXd ulps(nv);double gradient_norm=0.,spread=0.;
 for(double v:gradient)gradient_norm=std::max(gradient_norm,std::abs(v));
 for(int k=0;k<nv;++k){ulps[k]=std::abs(std::nextafter(x[k],INFINITY)-x[k]);for(double sign:{-1.,1.}){auto y=x;y[k]=std::nextafter(x[k],sign*INFINITY);auto gy=fg(std::vector<DM>{DM(y),p})[1].get_elements();for(int j=0;j<nv;++j)spread=std::max(spread,std::abs(gy[j]-gradient[j]));}}
 double scale=std::max(1.,h.cwiseAbs().maxCoeff());Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig((.5*(h+h.transpose()))/scale);
 int positive=0,negative=0,near_zero=0;for(double value:eig.eigenvalues()){if(value>1e-10)++positive;else if(value< -1e-10)++negative;else ++near_zero;}
 Json out{{"note","Objective derivatives only; no optimizer calls and no safety/optimality claim. ULP linear sensitivity is an estimate, not an error certificate."},{"objective_normalized",double(value[0])},{"objective_gradient_inf",gradient_norm},{"one_ulp_gradient_spread",spread},{"linearized_ulp_sensitivity",(h.cwiseAbs()*ulps).maxCoeff()},{"hessian_largest_abs_entry",scale},{"hessian_min_eigenvalue",eig.eigenvalues().minCoeff()*scale},{"hessian_max_eigenvalue",eig.eigenvalues().maxCoeff()*scale}};
 out["hessian_eigenvalue_count"]={{"positive",positive},{"negative",negative},{"near_zero",near_zero},{"threshold_relative_to_max_entry",1e-10}};
 out["time_variables"]=n;out["point_variables"]=vs-n;out["state_variables"]=nv-vs;
 out["max_point_state_gradient_abs"]=0.;for(int k=n;k<nv;++k)out["max_point_state_gradient_abs"]=std::max(out["max_point_state_gradient_abs"].get<double>(),std::abs(gradient[k]));
 out["point_state_hessian_block_max_abs"]=h.bottomRightCorner(nv-n,nv-n).cwiseAbs().maxCoeff();
 std::ofstream(argv[4])<<out.dump(2)<<'\n';std::cout<<out.dump(2)<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
