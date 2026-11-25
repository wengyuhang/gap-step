#include "native_minco_pvaj.hpp"
#include "restored_kernels.hpp"
#include <iostream>
using casadi::DM;using casadi::MX;using casadi::Function;
Json read(const std::string&p){Json j;std::ifstream(p)>>j;return j;}
int main(int argc,char**argv){try{
 if(argc!=4)throw std::runtime_error("INPUT SCENE OUTPUT");
 auto m=read(std::string(argv[1])+"/manifest.json"),scene=read(argv[2]),model=read(std::string(argv[1])+"/exchange_dynamics.json");
 native_minco::remove_domain_bounds(m);native_minco::use_derivative_order(m,3);m["native_soft_formulation"]="direct";m["slack_scale"]=Json::array();m["dynamic_residual_scale"]=model["residual_scale"];
 std::vector<Window>w;for(const auto&r:m["windows"])w.push_back(load_window(r));const auto node=restored_kernels::dynamic_node(model);
 native_minco::Graph plain(m,scene,w,node,1000.);m["duration_normalized_states"]=true;native_minco::Graph normalized(m,scene,w,node,1000.);
 const int n=m["n"],vs=m["velocity_start"],gates=m["m"];const auto seed=m["initials"][0]["original"].get<std::vector<double>>(),parameters=m["initials"][0]["parameters"].get<std::vector<double>>(),units=m["state_scale"].get<std::vector<double>>();auto x=seed;
 std::vector<double> t0(n),t(n);for(int k=0;k<n;++k){t0[k]=std::exp(seed[k]);x[k]=t0[k]>1.?std::sqrt(2.*t0[k]-1.)-1.:1.-std::sqrt(2./t0[k]-1.);}
 for(int j=vs;j<x.size();++j)x[j]/=units[j-vs];
 for(int k=0;k<n;++k){const double shorter=.65*t0[k];x[k]=shorter>1.?std::sqrt(2.*shorter-1.)-1.:1.-std::sqrt(2./shorter-1.);}
 x[0]+=.03;x[n/2]-=.02;
 for(int k=0;k<n;++k)t[k]=x[k]>0.?.5*x[k]*x[k]+x[k]+1.:1./(.5*x[k]*x[k]-x[k]+1.);
 auto y=x;for(int order=1;order<=3;++order)for(int gate=0;gate<gates;++gate)for(int axis=0;axis<3;++axis)y[vs+(order-1)*3*gates+3*gate+axis]/=std::pow(std::sqrt(t0[gate]*t0[gate+1]/(t[gate]*t[gate+1])),order);
 const auto a=plain.trajectory(std::vector<DM>{DM(x),DM(parameters)}),b=normalized.trajectory(std::vector<DM>{DM(y),DM(parameters)});
 double coeff=0.,physical=0.;for(int output:{0,2}){const auto u=a[output].get_elements(),v=b[output].get_elements();for(int j=0;j<u.size();++j){const double error=std::abs(u[j]-v[j])/(1.+std::abs(u[j]));if(output==0)physical=std::max(physical,error);else coeff=std::max(coeff,error);}}
 const double fa=double(a[4]),fb=double(b[4]),fe=std::abs(fa-fb)/(1.+std::abs(fa));
 Function f("normalized_units_gradient",{normalized.expressions[0],normalized.expressions[1]},{normalized.expressions[5],MX::gradient(normalized.expressions[5],normalized.expressions[0])});
 const auto grad=f(std::vector<DM>{DM(y),DM(parameters)})[1].get_elements();double ge=0.;for(int j=0;j<y.size();++j){double best=INFINITY;for(double step:{1e-5,1e-6,1e-7}){auto plus=y,minus=y;const double h=step*(1.+std::abs(y[j]));plus[j]+=h;minus[j]-=h;const double fd=(double(f(std::vector<DM>{DM(plus),DM(parameters)})[0])-double(f(std::vector<DM>{DM(minus),DM(parameters)})[0]))/(2.*h);best=std::min(best,std::abs(fd-grad[j])/std::max({1.,std::abs(fd),std::abs(grad[j])}));}ge=std::max(ge,best);}
 const double integral=double(a[5]);const bool pass=integral>1e-7 && coeff<1e-10 && physical<1e-10 && fe<1e-9 && ge<1e-3;Json report{{"active_dynamic_soft_integral",integral},{"passed",pass},{"coefficient_relative_error",coeff},{"physical_variable_relative_error",physical},{"objective_relative_error",fe},{"all_variable_gradient_relative_error",ge},{"optimization_variables",x.size()},{"new_constraints",0}};
 std::ofstream(argv[3])<<report.dump(2)<<'\n';std::cout<<report.dump(2)<<'\n';return pass?0:1;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
