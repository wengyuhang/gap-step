#include "restored_kernels.hpp"
#include <iostream>
using casadi::DM;using casadi::MX;using casadi::Function;
int main(int argc,char**argv){try{
 if(argc!=1 && argc!=2)throw std::runtime_error("[DYNAMICS_JSON]");
 Json meta=Json::parse(R"({"mass":0.85,"gravity":9.8066,"rotor_midpoint":3.4895,"rotor_radius":3.3895,"inertia":[0.001,0.001,0.0017],"rate_squared_limits":[225,8.9401],"residual_scale":[225,8.9401,11.48871025,11.48871025,11.48871025,11.48871025],"inverse_mixer":[[0.25,-2.357,-2.357,-5],[0.25,2.357,2.357,-5],[0.25,-2.357,2.357,5],[0.25,2.357,-2.357,5]]})");
 if(argc==2)std::ifstream(argv[1])>>meta;
 auto legacy=restored_kernels::dynamic_node(meta);
 meta["continuous_attitude_chart"]=true;dynamics_exchange::Model model(meta);
 auto f=restored_kernels::dynamic_node(meta);
 MX x=MX::sym("x",25);auto r=f(std::vector<MX>{x,MX(0.)})[0];
 Function jac("chart_jac",{x},{MX::jacobian(r,x)});
 auto state=[&](double delta){std::vector<double>s(25,0.);s[24]=1.;double z=delta-1.;
  s[2]=9.81*std::sqrt(1.-z*z)/2.;s[18]=(9.81*z-model.gravity)/2.;
  s[3]=.1/6.;s[11]=-.2/6.;s[19]=.3/6.;s[4]=.5/24.;s[12]=.3/24.;s[20]=-.4/24.;return s;};
 double worst=0.,legacy_error=0.;
 for(double delta:{.0005,.000999,.001001,.5,1.,1.5}){
  auto s=state(delta);auto v=f(std::vector<DM>{DM(s),DM(0.)})[0].get_elements();
  if(delta>.001){auto old=legacy(std::vector<DM>{DM(s),DM(0.)})[0].get_elements();for(int k=0;k<6;++k)legacy_error=std::max(legacy_error,std::abs(v[k]-old[k])/std::max({1.,std::abs(v[k]),std::abs(old[k])}));}
  const auto J=jac(std::vector<DM>{DM(s)})[0];
  for(int col:{2,3,4,10,11,12,18,19,20}){
   double best=INFINITY;
   for(double h:{1e-6,1e-7,1e-8}){
    auto a=s,b=s;a[col]+=h;b[col]-=h;auto va=f(std::vector<DM>{DM(a),DM(0.)})[0].get_elements();auto vb=f(std::vector<DM>{DM(b),DM(0.)})[0].get_elements();
    double error=0.;for(int k=0;k<6;++k){double fd=(va[k]-vb[k])/(2*h),ad=static_cast<double>(J(k,col));error=std::max(error,std::abs(fd-ad)/std::max({1.,std::abs(fd),std::abs(ad)}));}
    best=std::min(best,error);
   }worst=std::max(worst,best);
  }
  for(double value:v)if(!std::isfinite(value))throw std::runtime_error("Nonfinite away from pole");
 }
 auto lo=f(std::vector<DM>{DM(state(.001-1e-10)),DM(0.)})[0].get_elements();
 auto hi=f(std::vector<DM>{DM(state(.001+1e-10)),DM(0.)})[0].get_elements();double jump=0.;
 for(int k=0;k<6;++k)jump=std::max(jump,std::abs(hi[k]-lo[k])/std::max({1.,std::abs(hi[k]),std::abs(lo[k])}));
 Json report{{"gradient_relative_error",worst},{"legacy_regular_branch_relative_error",legacy_error},{"threshold_relative_jump",jump},{"passed",worst<1e-4 && jump<1e-5 && legacy_error<1e-8}};
 std::cout<<report.dump(2)<<'\n';return report["passed"].get<bool>()?0:1;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
