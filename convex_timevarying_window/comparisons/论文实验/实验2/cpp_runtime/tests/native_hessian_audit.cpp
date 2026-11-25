#include "native_minco_pvaj.hpp"
#include "restored_kernels.hpp"
#include <iostream>
using casadi::DM;using casadi::MX;using casadi::Function;
Json read(const std::string&p){Json j;std::ifstream(p)>>j;return j;}
int main(int argc,char**argv){try{
 if(argc!=5)throw std::runtime_error("INPUT SCENE SEED OUTPUT");
 auto m=read(std::string(argv[1])+"/manifest.json"),scene=read(argv[2]),dyn=read(std::string(argv[1])+"/exchange_dynamics.json");
 native_minco::remove_domain_bounds(m);native_minco::use_derivative_order(m,3);dyn["continuous_attitude_chart"]=true;
 int n=m["n"],nv=m["nvar"],vs=m["velocity_start"],seed=std::stoi(argv[3]);
 m["dynamic_residual_scale"]=dyn["residual_scale"];m["native_soft_formulation"]="residual_epigraph";
 std::vector<double>scale;for(int i=0;i<n*17*6;++i)scale.push_back(.01/dyn["residual_scale"][i%6].get<double>());m["slack_scale"]=scale;
 std::vector<Window>w;for(auto&j:m["windows"])w.push_back(load_window(j));
 native_minco::Graph graph(m,scene,w,restored_kernels::dynamic_node(dyn),1000.);
 auto x=m["initials"][seed]["original"].get<std::vector<double>>();x.resize(nv+scale.size(),0.);
 DM p(m["initials"][seed]["parameters"].get<std::vector<double>>());
 for(int i=0;i<n;++i){double t=std::exp(x[i]);x[i]=t>1.?std::sqrt(2*t-1)-1:1-std::sqrt(2/t-1);}
 if(m["free_points"].get<bool>())for(int gate=0;gate<m["m"];++gate)for(int axis=0;axis<2;++axis)x[m["slices"][gate]["begin"].get<int>()+axis]/=m["point_scale_m"][gate][axis].get<double>();
 for(int i=vs;i<nv;++i)x[i]/=m["state_scale"][i-vs].get<double>();
 auto residual=graph.residuals(std::vector<DM>{DM(x),p})[0].get_elements();for(int i=0;i<residual.size();++i)x[nv+i]=std::max(residual[i],0.)/scale[i];
 auto z=graph.expressions[0],par=graph.expressions[1],g=graph.expressions[6];
 std::vector<double>lambda(g.numel());for(int i=0;i<lambda.size();++i)lambda[i]=.01*std::sin(double(i+1));
 MX lag=graph.expressions[5]+MX::dot(DM(lambda),g),grad=MX::gradient(lag,z),direction=MX::sym("d",z.numel());
 Function probe("hessian_probe",{z,par,direction},{grad,MX::jtimes(grad,z,direction)});
 double worst=0.;Json tests=Json::array();
 for(int sample=0;sample<2;++sample){auto current=x;if(sample)for(int i=0;i<nv;++i)current[i]+=.002*std::sin(i+1.);
  for(int trial=0;trial<4;++trial){std::vector<double>d(x.size(),0.);for(int i=0;i<nv;++i)d[i]=std::sin((i+1.)*(trial+1.))/std::sqrt(double(nv));
   auto h=probe(std::vector<DM>{DM(current),p,DM(d)})[1].get_elements();double best=INFINITY;
   for(double step:{1e-4,1e-5,1e-6,1e-7}){auto a=current,b=current;for(int i=0;i<nv;++i){a[i]+=step*d[i];b[i]-=step*d[i];}
    auto ga=probe(std::vector<DM>{DM(a),p,DM(d)})[0].get_elements(),gb=probe(std::vector<DM>{DM(b),p,DM(d)})[0].get_elements();double numerator=0.,denominator=1.;
    for(int i=0;i<h.size();++i){double fd=(ga[i]-gb[i])/(2*step);numerator=std::max(numerator,std::abs(fd-h[i]));denominator=std::max({denominator,std::abs(fd),std::abs(h[i])});}best=std::min(best,numerator/denominator);
   }worst=std::max(worst,best);tests.push_back({{"probe",sample},{"direction",trial},{"relative_error",best}});
  }
 }
 Json out{{"passed",worst<1e-4},{"maximum_relative_error",worst},{"physical_variables",nv},{"total_variables",x.size()},{"checks",tests}};std::ofstream(argv[4])<<out.dump(2)<<'\n';std::cout<<out.dump(2)<<'\n';return out["passed"].get<bool>()?0:1;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
