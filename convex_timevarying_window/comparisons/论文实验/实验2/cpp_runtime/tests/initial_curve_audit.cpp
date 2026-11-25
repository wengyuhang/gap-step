#include "native_minco_pvaj.hpp"
#include "restored_kernels.hpp"
#include <iostream>
using casadi::DM;using casadi::Function;
Json read(const std::string&p){Json j;std::ifstream(p)>>j;return j;}
int main(int argc,char**argv){try{
 if(argc!=4)throw std::runtime_error("INPUT SCENE OUTPUT");
 auto m=read(std::string(argv[1])+"/manifest.json"),scene=read(argv[2]),dyn=read(std::string(argv[1])+"/exchange_dynamics.json");
 native_minco::remove_domain_bounds(m);native_minco::use_derivative_order(m,3);dyn["continuous_attitude_chart"]=true;
 m["native_soft_formulation"]="direct";m["dynamic_residual_scale"]=dyn["residual_scale"];m["slack_scale"]=Json::array();
 int n=m["n"],nv=m["nvar"],vs=m["velocity_start"],ng=m["m"];
 std::vector<Window>w;for(auto&j:m["windows"])w.push_back(load_window(j));
 native_minco::Graph graph(m,scene,w,restored_kernels::dynamic_node(dyn),1000.);
 const auto z=graph.expressions[0],p=graph.expressions[1];Function fg("initial_fg",{z,p},{graph.expressions[5],casadi::MX::gradient(graph.expressions[5],z),graph.expressions[6]});
 Json rows=Json::array();
 for(auto&seed:m["initials"]){auto x=seed["original"].get<std::vector<double>>();const auto param=DM(seed["parameters"].get<std::vector<double>>());
  for(int k=0;k<n;++k){double t=std::exp(x[k]);x[k]=t>1?std::sqrt(2*t-1)-1:1-std::sqrt(2/t-1);}
  if(m["free_points"].get<bool>())for(int g=0;g<ng;++g)for(int axis=0;axis<2;++axis)x[m["slices"][g]["begin"].get<int>()+axis]/=m["point_scale_m"][g][axis].get<double>();
  for(int k=vs;k<nv;++k)x[k]/=m["state_scale"][k-vs].get<double>();
  auto v=graph.trajectory(std::vector<DM>{DM(x),param});auto times=v[1].get_elements(),coeff=v[2].get_elements();auto rv=graph.residuals(std::vector<DM>{DM(x),param})[0].get_elements();
  const auto actual_states=v[0].get_elements();double endpoint_derivative_error=0.,endpoint_derivative_relative_error=0.;
  for(int piece=0;piece<n;++piece)for(int order=2;order<=3;++order)for(int axis=0;axis<3;++axis){
   double value=0.;for(int power=7;power>=order;--power){double factor=1.;for(int k=0;k<order;++k)factor*=power-k;value+=factor*coeff[24*piece+8*axis+power];}value/=std::pow(times[piece],order);
   const double expected=piece<ng?actual_states[vs+3*((order-1)*ng+piece)+axis]:0.;
   endpoint_derivative_error=std::max(endpoint_derivative_error,std::abs(value-expected));endpoint_derivative_relative_error=std::max(endpoint_derivative_relative_error,std::abs(value-expected)/(1.+std::abs(expected)));
  }
  std::vector<double> maxima(6,-INFINITY);std::vector<int> bad(6,0);double minforce=INFINITY,minchart=INFINITY,maxsnap=0.;Json worst;
  for(int piece=0;piece<n;++piece)for(int node=0;node<=16;++node){double u=node/16.;Vector a=Vector::Zero(),snap=Vector::Zero();for(int power=7;power>=2;--power)for(int axis=0;axis<3;++axis)a[axis]=a[axis]*u+power*(power-1.)*coeff[24*piece+8*axis+power];a/=times[piece]*times[piece];a[2]+=dyn["gravity"].get<double>();
   for(int power=7;power>=4;--power)for(int axis=0;axis<3;++axis)snap[axis]=snap[axis]*u+power*(power-1.)*(power-2.)*(power-3.)*coeff[24*piece+8*axis+power];snap/=std::pow(times[piece],4);maxsnap=std::max(maxsnap,snap.norm());
   minforce=std::min(minforce,a.norm());minchart=std::min(minchart,1.+a[2]/a.norm());
   for(int c=0;c<6;++c){double value=rv[(piece*17+node)*6+c];if(value>maxima[c])maxima[c]=value;if(value>0)++bad[c];}
  }
  auto vals=fg(std::vector<DM>{DM(x),param});auto g=vals[2].get_elements(),grad=vals[1].get_elements();double violation=0,gnorm=0;for(double value:g)violation=std::max(violation,-value);for(double value:grad)gnorm=std::max(gnorm,std::abs(value));
  rows.push_back({{"seed",seed["index"]},{"endpoint_derivative_error",endpoint_derivative_error},{"endpoint_derivative_relative_error",endpoint_derivative_relative_error},{"duration_s",std::accumulate(times.begin(),times.end(),0.)},{"objective",double(v[4])},{"soft_integral",double(v[5])},{"base_violation",violation},{"normalized_gradient_inf",gnorm},{"minimum_specific_thrust",minforce},{"minimum_chart_denominator",minchart},{"maximum_sample_snap",maxsnap},{"max_normalized_dynamic_residual",maxima},{"violating_nodes",bad}});
 }
 std::ofstream(argv[3])<<Json{{"optimizer_calls",0},{"nodes_per_piece",17},{"initials",rows}}.dump(2)<<'\n';
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
