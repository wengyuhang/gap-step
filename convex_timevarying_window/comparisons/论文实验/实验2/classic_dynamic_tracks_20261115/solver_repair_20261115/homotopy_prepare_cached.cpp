#include "trajectory.hpp"
#include "drolib/solver/minco_snap.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
namespace fs=std::filesystem;
Json read(const fs::path&p){Json d;std::ifstream(p)>>d;return d;}
int main(int argc,char**argv){try {
 if(argc!=6 && argc!=7)throw std::runtime_error("Usage: BASE_GRAPH TARGET_SCENE PREVIOUS_RESULT FRACTION OUT");
 fs::path base=argv[1],out=argv[5];if(fs::exists(out))throw std::runtime_error("Output exists");
 auto m=read(base/"manifest.json"),scene=read(argv[2]),previous=read(argv[3]);double fraction=std::stod(argv[4]);
 if(!previous.value("found",false) || fraction<0. || fraction>1.)throw std::runtime_error("Invalid continuation input");
 for(auto&w:m["windows"])w["amplitude"]=fraction*w["amplitude"].get<double>();
 for(auto&w:scene["windows"])w["amplitude"]=fraction*w["amplitude"].get<double>();
 auto x=previous.at("variables").get<std::vector<double>>();const int n=m["n"],nv=m["nvar"];
 if(x.size()!=nv)throw std::runtime_error("Trajectory dimension mismatch");
 auto p=m["initials"][0]["parameters"].get<std::vector<double>>();double total=0.;
 for(int k=0;k<n;++k){total+=std::exp(x[k]);p[k+1]=total;}p[0]=total;
 if(argc==7 && std::string(argv[6])=="--minimum-snap") {
  Eigen::VectorXd times(n);Eigen::Matrix3Xd points(3,n-1);double at=0.;
  for(int k=0;k<n;++k){times[k]=std::exp(x[k]);at+=times[k];if(k<n-1){const auto w=load_window(scene["windows"][k]);points.col(k)=w.center+w.rotation*w.motion(at);}}
  drolib::PVAJ head=drolib::PVAJ::Zero(),tail=head;for(int axis=0;axis<3;++axis){head(axis,0)=scene["start"][axis];tail(axis,0)=scene["goal"][axis];}
  drolib::MincoSnap minco;minco.setConditions(head,tail,n);minco.setParameters(points,times);const auto&co=minco.getCoeffs();const int vs=m["velocity_start"],gates=n-1;
  for(int d=1;d<=3;++d)for(int k=0;k<gates;++k)for(int axis=0;axis<3;++axis){double value=0.;for(int power=7;power>=d;--power){double factor=1.;for(int j=0;j<d;++j)factor*=power-j;value=value*times[k]+factor*co(8*k+power,axis);}x[vs+3*((d-1)*gates+k)+axis]=value;}
  m["initial_derivative_rule"]="Minimum snap at target stage points and transferred independent segment times; full PVAJ released during finite optimization";
 }
 if(argc==7 && std::string(argv[6])=="--multipliers") {
  const auto finite=read(fs::path(argv[3]).parent_path()/"finite_input.json");
  p[0]=finite.at("parameters").at(0);
  const auto warm=previous.at("finite_incumbent");
  if(warm.at("status")!="Solve_Succeeded" || warm.at("variables")!=previous.at("variables"))throw std::runtime_error("Incumbent is not the accepted converged trajectory");
  std::map<std::pair<int,int>,Json> cached;
  if(previous.contains("geometry_witnesses"))for(const auto& entry:previous["geometry_witnesses"])cached[{entry["gate"],entry["piece"]}]=entry;
  else for(const auto& record:previous.at("records"))for(const auto& added:record.at("geometry_added_witnesses")) {
      const int gate=added.at("gate"),piece=added.at("piece");auto key=std::make_pair(gate,piece);
      if(!cached.count(key))cached[key]={{"gate",gate},{"piece",piece},{"kind",piece==gate ? "before" : piece==gate+1 ? "after" : "normalized"},{"values",Json::array()}};
      cached[key]["values"].push_back(added.at("offset"));
  }
  m["initial_geometry_witnesses"]=Json::array();for(const auto& entry:cached)m["initial_geometry_witnesses"].push_back(entry.second);
  m["initial_finite_warm_start"]=warm;
  for(const auto& name:{"ipopt.warm_start_bound_push","ipopt.warm_start_mult_bound_push","ipopt.warm_start_slack_bound_push"})m["solver_options"][name]=1e-9;
 }
 m["initials"]=Json::array({Json{{"index",0},{"original",x},{"parameters",p},{"finite",std::vector<double>(nv+m["slack_scale"].size(),0.)}}});
 m["initialization"]={{"rule","same-target motion-amplitude continuation; previous strictly converged and verified stage"},{"fraction",fraction},{"source",fs::absolute(argv[3]).string()}};
 m["solver_options"]["ipopt.print_level"]=5;
 fs::create_directories(out);fs::copy_file(base/"exchange_dynamic_node.casadi",out/"exchange_dynamic_node.casadi");fs::copy_file(base/"exchange_dynamics.json",out/"exchange_dynamics.json");
 std::ofstream(out/"manifest.json")<<m.dump(2)<<'\n';std::ofstream(out/"scene.json")<<scene.dump()<<'\n';
 std::cout<<fraction<<' '<<total<<'\n';
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
