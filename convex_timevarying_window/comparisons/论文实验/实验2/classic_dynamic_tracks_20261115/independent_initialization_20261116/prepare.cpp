#include "trajectory.hpp"
#include "drolib/solver/minco_snap.hpp"
#include "minco_pv.hpp"
#include <filesystem>
namespace fs=std::filesystem;
Json read(const fs::path& p){std::ifstream f(p);if(!f)throw std::runtime_error("Missing "+p.string());Json j;f>>j;return j;}
void save(const fs::path& p,const Json& j){fs::create_directories(p.parent_path());std::ofstream(p)<<j.dump(2)<<'\n';}
int main(int argc,char** argv){try{
 if(argc!=4)throw std::runtime_error("Usage: INPUTS LEVEL OUTPUT");fs::path in=argv[1],out=argv[3];std::string level=argv[2];if(fs::exists(out))throw std::runtime_error("Output exists");
 auto scene=read(in/"assets"/level/"scene.json");int m=scene["windows"].size(),n=m+1;drolib::PVAJ head=drolib::PVAJ::Zero(),tail=head;
 for(int axis=0;axis<3;++axis){head(axis,0)=scene["start"][axis];tail(axis,0)=scene["goal"][axis];}
 auto source=read(in/"inputs"/(level+"_center")/"manifest.json");std::vector<double> average(n,0.);for(auto seed:source["initials"])for(int k=0;k<n;++k)average[k]+=seed["original"][k].get<double>()/source["initials"].size();
 Json checks=Json::array();
 for(int seed=0;seed<8;++seed){double speed=std::vector<double>{3.,6.,9.,12.}[seed/2];Eigen::VectorXd times(n);Eigen::Matrix3Xd points(3,m);Vector previous=head.col(0);double at=0.;
 for(int k=0;k<n;++k){Vector target;if(k<m){target=load_window(scene["windows"][k]).center;}else target=tail.col(0);double ratio=std::exp(source["initials"][seed]["original"][k].get<double>()-average[k]);times[k]=(target-previous).norm()/speed*ratio;previous=target;at+=times[k];if(k<m){auto w=load_window(scene["windows"][k]);points.col(k)=w.center+w.rotation*w.motion(at);}}
 drolib::MincoSnap minco;minco.setConditions(head,tail,n);minco.setParameters(points,times);const auto coefficients=minco.getCoeffs();Eigen::Matrix3Xd states(3,3*m);
 for(int order=1;order<=3;++order)for(int gate=0;gate<m;++gate)for(int axis=0;axis<3;++axis){double v=0.;for(int k=7;k>=order;--k){double factor=1.;for(int j=0;j<order;++j)factor*=k-j;v=v*times[gate]+factor*coefficients(8*gate+k,axis);}states(axis,(order-1)*m+gate)=v;}
 togt_pv::MincoPV check(head,tail,n,3);if(!check.fit(points,states,times))throw std::runtime_error("Native fit failed");double err=0.;for(int k=0;k<8*n;++k)for(int axis=0;axis<3;++axis)err=std::max(err,std::abs(coefficients(k,axis)-check.coefficients()(k,axis)));if(err>1e-6)throw std::runtime_error("Initial physical curve mismatch");
 for(std::string method:{"ours","center"}){auto folder=in/"inputs"/(level+"_"+method);auto manifest=read(folder/"manifest.json");int nv=manifest["nvar"],vs=manifest["velocity_start"];std::vector<double> x(nv,0.),params(1+2*n,0.);double t=0.;for(int k=0;k<n;++k){x[k]=std::log(times[k]);t+=times[k];params[1+k]=t;params[1+n+k]=1.;}params[0]=t;for(int order=1;order<=3;++order)for(int gate=0;gate<m;++gate)for(int axis=0;axis<3;++axis)x[vs+3*((order-1)*m+gate)+axis]=states(axis,(order-1)*m+gate);
 manifest["initials"]=Json::array({Json{{"index",seed},{"original",x},{"finite",std::vector<double>(nv+manifest["slack_scale"].size(),0.)},{"parameters",params}}});manifest["initialization"]={{"rule","direct geometry and fixed speed grid; no optimizer or solved-result input"},{"speed_mps",speed},{"seed",seed},{"curve_error",err}};
 manifest["solver_options"]["ipopt.print_level"]=0;for(std::string key:{"bound_push","bound_frac","slack_bound_push","slack_bound_frac","warm_start_bound_push","warm_start_mult_bound_push","warm_start_slack_bound_push","warm_start_bound_frac","warm_start_slack_bound_frac"})manifest["solver_options"]["ipopt."+key]=1e-9;
 fs::path dest=out/(method+"_"+std::to_string(seed));save(dest/"manifest.json",manifest);for(std::string file:{"exchange_dynamics.json","exchange_dynamic_node.casadi"})fs::copy_file(folder/file,dest/file);
 }
 checks.push_back({{"seed",seed},{"speed_mps",speed},{"duration_s",times.sum()},{"curve_error",err}});
 }
 save(out/"INITIALIZATION_AUDIT.json",{{"no_optimizer_calls",true},{"paired_time_point_pvaj_identical",true},{"seeds",checks}});
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;}
