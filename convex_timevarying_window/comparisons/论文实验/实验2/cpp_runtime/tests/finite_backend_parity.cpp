#include "json.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
namespace fs=std::filesystem;using Json=nlohmann::json;
Json read(const fs::path&p){std::ifstream f(p);if(!f)throw std::runtime_error("Missing "+p.string());Json j;f>>j;return j;}
void save(const fs::path&p,const Json&j){fs::create_directories(p.parent_path());std::ofstream(p)<<j.dump(2)<<'\n';}
int run(const std::vector<std::string>&args,const fs::path&log){const pid_t pid=fork();if(pid<0)throw std::runtime_error("fork");if(!pid){const int fd=open(log.c_str(),O_WRONLY|O_CREAT|O_EXCL,0644);if(fd<0)_exit(125);dup2(fd,1);dup2(fd,2);close(fd);for(auto key:{"SIP_DURATION_NORMALIZED_STATES","SIP_SLSQP_EQUILIBRATE","SIP_SLSQP_COST_UNIT","SIP_SLSQP_ACCURACY","SIP_DUMP_NLP","SIP_IPOPT_TRACE","SIP_PROJECTED_IPOPT"})unsetenv(key);for(auto key:{"OPENBLAS_NUM_THREADS","OMP_NUM_THREADS","MKL_NUM_THREADS","SIP_FINITE_MODEL_AUDIT","SIP_DUMP_INPUT"})setenv(key,"1",1);auto copy=args;std::vector<char*>raw;for(auto&v:copy)raw.push_back(v.data());raw.push_back(nullptr);execv(raw[0],raw.data());_exit(126);}int status;while(waitpid(pid,&status,0)<0){}return WIFEXITED(status)?WEXITSTATUS(status):128+WTERMSIG(status);}
double error(double a,double b){return std::abs(a-b)/std::max({1.,std::abs(a),std::abs(b)});}
double vector_error(const Json&a,const Json&b){if(a.size()!=b.size())throw std::runtime_error("Vector dimensions differ");double e=0.;for(size_t j=0;j<a.size();++j){if(!a[j].is_number()||!b[j].is_number())throw std::runtime_error("Non-numeric probe value");const double x=a[j],y=b[j];if(!std::isfinite(x)||!std::isfinite(y))throw std::runtime_error("Nonfinite probe value");e=std::max(e,error(x,y));}return e;}
std::vector<double> dense(const Json&m){const int rows=m["rows"],cols=m["cols"];std::vector<double>out(rows*cols,0.);for(size_t k=0;k<m["values"].size();++k)out[m["column"][k].get<int>()*rows+m["row"][k].get<int>()]=m["values"][k];return out;}
int main(int argc,char**argv){try{
 if(argc!=5)throw std::runtime_error("SIP CURRENT_INPUTS CURRENT_ASSETS NEW_OUTPUT");
 const fs::path sip=fs::absolute(argv[1]),inputs=fs::absolute(argv[2]),assets=fs::absolute(argv[3]),out=fs::absolute(argv[4]);if(fs::exists(out))throw std::runtime_error("Output exists");fs::create_directories(out);
 Json results=Json::array();double worst=0.;
 for(std::string level:{"splits_moderate","alphapilot_moderate","airsim_freq4"})for(std::string form:{"direct","residual_epigraph"}){
  if(form=="residual_epigraph"&&level!="splits_moderate")continue;
  for(std::string method:{"ours","center"}){
   std::vector<Json>pair;
   for(std::string backend:{"ipopt","slsqp"}){
    const auto directory=out/(level+"_"+method+"_"+form)/backend;const auto src=inputs/level/method;
    auto m=read(src/"manifest.json");m["native_finite_solver"]=backend;m["native_soft_formulation"]=form;
    save(directory/"input/manifest.json",m);fs::copy_file(src/"exchange_dynamics.json",directory/"input/exchange_dynamics.json");fs::create_directories(directory/"output");
    const int code=run({sip.string(),(directory/"input").string(),(directory/"output").string(),"0","--soft-mincopv",(assets/level/"scene.json").string(),"1000"},directory/"audit.log");
    if(code)throw std::runtime_error("Audit child failed: "+directory.string());
    const auto result=read(directory/"output/result.json");if(result["found"]!=false||!result["optimizer_calls"].empty()||result["termination_reason"]!="finite_model_audit_only_no_optimization")throw std::runtime_error("Audit unexpectedly optimized or accepted");
    pair.push_back(read(directory/"output/finite_model_audit.json"));if(pair.back()["backend"]!=backend)throw std::runtime_error("Backend label mismatch");
   }
   const auto&a=pair[0];const auto&b=pair[1];
   for(auto key:{"formulation","nvar","constraint_rows","parameters","lower_x","upper_x","lower_g","upper_g","keys","windows","vehicle","dynamics_model","finite_radius","verification_radius","workspace_constraints","duration_bounds","pvaj_order","point_coordinates","point_erosion_radius_m","time_weight","dynamics_weight","quadrature_intervals","slsqp_cost_unit","slsqp_equalities","slsqp_original_rows","slsqp_signs","slsqp_offsets"})if(a.at(key)!=b.at(key))throw std::runtime_error(std::string("Metadata differs: ")+key);
   if(a["workspace_constraints"]!=false||a["duration_bounds"]!=false||a["pvaj_order"]!=3||a["point_coordinates"]!="scaled_xy"||a["quadrature_intervals"]!=16||a["dynamics_weight"]!=1000.)throw std::runtime_error("Stale planner model");
   double maximum=0.,conversion=0.;
   for(int sample=0;sample<3;++sample){const auto&u=a["probes"][sample];const auto&v=b["probes"][sample];maximum=std::max(maximum,error(u["f"],v["f"]));for(auto key:{"x","g","gradient","physical_variables","durations","coefficients","slsqp_g","slsqp_gradient"})maximum=std::max(maximum,vector_error(u.at(key),v.at(key)));
    for(auto key:{"jacobian","slsqp_jacobian"}){for(auto structure:{"rows","cols","row","column"})if(u[key][structure]!=v[key][structure])throw std::runtime_error("Jacobian sparsity differs");maximum=std::max(maximum,vector_error(u[key]["values"],v[key]["values"]));}
    const int original_rows=a["constraint_rows"],nvar=a["nvar"],mapped=v["slsqp_g"].size();const auto j=dense(v["jacobian"]),cj=dense(v["slsqp_jacobian"]);for(int k=0;k<mapped;++k){const int row=a["slsqp_original_rows"][k];const double sign=a["slsqp_signs"][k],offset=a["slsqp_offsets"][k];conversion=std::max(conversion,error(v["slsqp_g"][k],sign*v["g"][row].get<double>()+offset));for(int col=0;col<nvar;++col)conversion=std::max(conversion,error(cj[col*mapped+k],sign*j[col*original_rows+row]));}
   }
   if(maximum>1e-11||conversion>1e-12)throw std::runtime_error("Finite problem numerical mismatch");worst=std::max({worst,maximum,conversion});
   results.push_back({{"level",level},{"method",method},{"formulation",form},{"nvar",a["nvar"]},{"rows",a["constraint_rows"]},{"probes",3},{"maximum_relative_error",maximum},{"adapter_relative_error",conversion},{"passed",true}});
   save(out/"SUMMARY.json",{{"complete",false},{"checks",results}});std::cout<<level<<' '<<method<<' '<<form<<" passed\n"<<std::flush;
  }
 }
 save(out/"SUMMARY.json",{{"complete",true},{"optimization_executed",false},{"model_pairs",results.size()},{"worst_relative_error",worst},{"checks",results}});return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
