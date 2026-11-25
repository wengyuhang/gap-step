#include "restored_kernels.hpp"
#include "drolib/solver/minco_snap.hpp"
#include <iostream>
Json read(const std::string& path){Json j;std::ifstream(path)>>j;return j;}
int main(int argc,char**argv){try{
 if(argc!=4)throw std::runtime_error("INPUT SCENE OUTPUT");
 auto m=read(std::string(argv[1])+"/manifest.json"),scene=read(argv[2]),model=read(std::string(argv[1])+"/exchange_dynamics.json");
 model["continuous_attitude_chart"]=true;const auto node=restored_kernels::dynamic_node(model);
 const int n=m["n"],ng=m["m"],vs=m["velocity_start"];const auto seed=m["initials"][0]["original"].get<std::vector<double>>();
 drolib::PVAJ head=drolib::PVAJ::Zero(),tail=head;for(int axis=0;axis<3;++axis){head(axis,0)=scene["start"][axis];tail(axis,0)=scene["goal"][axis];}
 std::vector<Window> gates;for(auto&w:m["windows"])gates.push_back(load_window(w));
 Json records=Json::array();
 for(double scale:{1.,.9998,.999,.995,.99,.98,.95,.9}){
  Eigen::VectorXd times(n);Eigen::Matrix3Xd points(3,ng);double at=0.;
  for(int k=0;k<n;++k){times[k]=std::exp(seed[k])*scale;at+=times[k];if(k<ng)points.col(k)=gates[k].center+gates[k].rotation*gates[k].motion(at);}
  for(int regenerate=0;regenerate<2;++regenerate){
   Eigen::Matrix3Xd states=Eigen::Map<const Eigen::Matrix3Xd>(seed.data()+vs,3,3*ng);
   if(regenerate){drolib::MincoSnap minco;minco.setConditions(head,tail,n);minco.setParameters(points,times);const auto c=minco.getCoeffs();
    for(int order=1;order<=3;++order)for(int gate=0;gate<ng;++gate)for(int axis=0;axis<3;++axis){double value=0.;for(int power=7;power>=order;--power){double f=1.;for(int j=0;j<order;++j)f*=power-j;value=value*times[gate]+f*c(8*gate+power,axis);}states(axis,(order-1)*ng+gate)=value;}}
   togt_pv::MincoPV fitted(head,tail,n,3);if(!fitted.fit(points,states,times))throw std::runtime_error("MINCO fit failed");
   const auto c=fitted.normalizedCoefficients();double penalty=0.,maxr=-INFINITY;
   for(int piece=0;piece<n;++piece){std::vector<double> packed(25);for(int axis=0;axis<3;++axis)for(int power=0;power<8;++power)packed[8*axis+power]=c(8*piece+power,axis);packed[24]=times[piece];
    for(int k=0;k<=16;++k){const auto residual=node(std::vector<casadi::DM>{casadi::DM(packed),casadi::DM(k/16.)})[0].get_elements();for(int component=0;component<6;++component){maxr=std::max(maxr,residual[component]);const double v=residual[component]*model["residual_scale"][component].get<double>();const double loss=v<=0.?0.:v>=.01?v-.005:(.01-.5*v)*std::pow(v/.01,3);penalty+=times[piece]/16.*(k==0||k==16?.5:1.)*loss;}}}
   records.push_back({{"time_scale",scale},{"regenerate_minimum_snap_derivatives",bool(regenerate)},{"time_s",times.sum()},{"objective",times.sum()+1000.*penalty},{"soft_integral",penalty},{"max_normalized_dynamic_residual",maxr}});
  }
 }
 std::ofstream(argv[3])<<Json{{"optimizer_calls",0},{"records",records}}.dump(2)<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
