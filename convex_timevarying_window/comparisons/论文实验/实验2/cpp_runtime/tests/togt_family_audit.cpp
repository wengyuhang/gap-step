#include "native_minco_pvaj.hpp"
#include "restored_kernels.hpp"
#include <iostream>
using casadi::DM;using casadi::MX;using casadi::Function;
Json read(const std::string&p){Json j;std::ifstream(p)>>j;return j;}
int main(int argc,char**argv){try{
 if(argc!=5)throw std::runtime_error("INPUT SCENE TOGT_RESULT OUTPUT");auto m=read(std::string(argv[1])+"/manifest.json"),scene=read(argv[2]),ref=read(argv[3]),dyn=read(std::string(argv[1])+"/exchange_dynamics.json");
 native_minco::remove_domain_bounds(m);native_minco::use_derivative_order(m,3);m["native_soft_formulation"]="residual_epigraph";m["lift_arrival_times"]=false;dyn["continuous_attitude_chart"]=true;m["dynamic_residual_scale"]=dyn["residual_scale"];
 int n=m["n"],count=m["m"],nv=m["nvar"],vs=m["velocity_start"];std::vector<double>scales;for(int k=0;k<n*17*6;++k)scales.push_back(.01/dyn["residual_scale"][k%6].get<double>());m["slack_scale"]=scales;
 std::vector<Window>w;for(auto&r:m["windows"])w.push_back(load_window(r));native_minco::Graph graph(m,scene,w,restored_kernels::dynamic_node(dyn),1000.);
 auto T=ref["durations"].get<std::vector<double>>();std::vector<double>x(nv+scales.size(),0.),p(1+2*n,0.);double total=0.,violation=0.;
 for(int piece=0;piece<n;++piece){x[piece]=T[piece]>1?std::sqrt(2*T[piece]-1)-1:1-std::sqrt(2/T[piece]-1);total+=T[piece];p[1+piece]=total;p[1+n+piece]=1.;
  if(piece==count)continue;Curve c;for(int k=0;k<8;++k)for(int axis=0;axis<3;++axis)c(k,axis)=ref["coefficients"][piece][k][axis];
  auto at_end=[&](int order){Vector v=Vector::Zero();for(int k=order;k<8;++k){double factor=1;for(int j=0;j<order;++j)factor*=k-j;v+=factor*c.row(k).transpose();}return (v/std::pow(T[piece],order)).eval();};
  Vector q=w[piece].rotation.transpose()*(at_end(0)-w[piece].center)-w[piece].motion(total);
  if(m["free_points"].get<bool>())for(int axis=0;axis<2;++axis){int k=m["slices"][piece]["begin"].get<int>()+axis;x[k]=q[axis]/m["point_scale_m"][piece][axis].get<double>();violation=std::max({violation,m["point_lower_m"][piece][axis].get<double>()-q[axis],q[axis]-m["point_upper_m"][piece][axis].get<double>()});}
  else if(q.head<2>().norm()>1e-7)throw std::runtime_error("Noncentral reference in center family");
  for(int order=1;order<=3;++order){Vector v=at_end(order);for(int axis=0;axis<3;++axis){int k=vs+3*((order-1)*count+piece)+axis;x[k]=v[axis]/m["state_scale"][k-vs].get<double>();}}
 }
 p[0]=total;auto r=graph.residuals(std::vector<DM>{DM(x),DM(p)})[0].get_elements();for(int k=0;k<r.size();++k)x[nv+k]=std::max(0.,r[k])/scales[k];
 auto val=graph.trajectory(std::vector<DM>{DM(x),DM(p)});auto coeff=val[2].get_elements();double coeff_error=0.;
 for(int piece=0;piece<n;++piece)for(int k=0;k<8;++k)for(int axis=0;axis<3;++axis){double expected=ref["coefficients"][piece][k][axis];coeff_error=std::max(coeff_error,std::abs(coeff[24*piece+8*axis+k]-expected)/(1+std::abs(expected)));}
 Function constraints("base",{graph.expressions[0],graph.expressions[1]},{graph.expressions[6]});auto values=constraints(std::vector<DM>{DM(x),DM(p)})[0].get_elements();for(double g:values)violation=std::max(violation,-g);
 Json report{{"optimizer_calls",0},{"used_as_optimizer_initialization",false},{"coefficient_relative_error",coeff_error},{"finite_base_violation",violation},{"nominal_time_s",total},{"objective_in_current_model",double(val[4])},{"dynamic_soft_integral",double(val[5])},{"trajectory_family_contains_reference",coeff_error<1e-7 && violation<1e-5}};
 std::ofstream(argv[4])<<report.dump(2)<<'\n';std::cout<<report.dump(2)<<'\n';return report["trajectory_family_contains_reference"].get<bool>()?0:1;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
