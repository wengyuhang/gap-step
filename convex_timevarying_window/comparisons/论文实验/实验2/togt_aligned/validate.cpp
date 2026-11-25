#include "restored_kernels.hpp"
#include "drolib/system/quadrotor_params.hpp"
#include <filesystem>
namespace fs=std::filesystem;
Json read(const fs::path&p){Json j;std::ifstream(p)>>j;return j;}
void require(bool x,const char*s){if(!x)throw std::runtime_error(s);}
int main(int argc,char**argv){try{
 if(argc!=4)throw std::runtime_error("Usage: aligned_validate INPUTS_DIR ORIGINAL_SOURCE OUTPUT");fs::path in=argv[1],source=argv[2];auto vehicle=read(in/"vehicle.json"),dynamics=read(in/"exchange_dynamics.json");drolib::QuadParams author;require(author.load(source/"parameters/cpc/cpc_quad.yaml"),"author QuadA load");
 require(std::abs(vehicle["mass"].get<double>()-author.mass)<1e-15,"mass mismatch");double inverse_error=0.;for(int i=0;i<4;++i)for(int k=0;k<4;++k)inverse_error=std::max(inverse_error,std::abs(vehicle["inverse_mixer"][i][k].get<double>()-author.T_mb(i,k)));require(inverse_error<1e-14,"mixer mismatch");
 auto node=restored_kernels::dynamic_node(dynamics);dynamics_exchange::Model model(dynamics);double rotor_error=0.;for(double f:{.05,.1,2.,6.879,7.}){
  std::vector<double>state(25,0.);state[24]=1.;state[18]=.5*(4*f/author.mass-9.8066);
  const auto values=node(std::vector<casadi::DM>{casadi::DM(state),casadi::DM(.5)})[0].get_elements();
  const double expected=(f-.1)*(f-6.879)/(model.radius*model.radius);
  for(int k=2;k<6;++k)rotor_error=std::max(rotor_error,std::abs(values[k]-expected));}
 require(rotor_error<1e-10,"nonzero original minThr was not correctly restored");
 auto protocol=read(in/"PROTOCOL.json");int scenes=0,starts=0;double point_error=0.;
 for(const auto&level:protocol["levels"]){auto scene=read(in/"assets"/level.get<std::string>()/"scene.json");require(scene["windows"].size()==7,"seven gates required");int circles=0;for(const auto&w:scene["windows"])circles+=w["circle"].get<bool>();require(circles==3,"three circles required");++scenes;
  for(const auto&method:{"ours","center"}){auto meta=read(in/"inputs"/(level.get<std::string>()+"_"+method)/"manifest.json");planar_points::validate(meta);for(const auto&seed:meta["initials"]){++starts;for(int k=8;k<meta["velocity_start"].get<int>();++k)require(seed["original"][k]==0.,"point initial is not center");}
   native_minco::remove_domain_bounds(meta);require(!meta["restore_historical_domain"].get<bool>() && meta["workspace_rows"]==0,"old domain flag leaked into new native graph");
   meta["dynamic_residual_scale"]=dynamics["residual_scale"];std::vector<Window>windows;for(const auto&w:meta["windows"])windows.push_back(load_window(w));native_minco::Graph graph(meta,scene,windows,node,1000.);
   require(graph.expressions[6].numel()==static_cast<casadi_int>(meta["lower_g"].size()),"finite bound rows mismatch");
   auto x=meta["initials"][0]["original"].get<std::vector<double>>();int nv=meta["nvar"],vs=meta["velocity_start"];for(int k=0;k<8;++k){double t=exp(x[k]);x[k]=t>1?sqrt(2*t-1)-1:1-sqrt(2/t-1);}for(int k=vs;k<nv;++k)x[k]/=meta["state_scale"][k-vs].get<double>();x.resize(nv+1584,0.);
   const auto output=graph.trajectory(std::vector<casadi::DM>{casadi::DM(x),casadi::DM(meta["initials"][0]["parameters"].get<std::vector<double>>())});
   const auto p=output[3].get_elements(),times=output[1].get_elements();double at=0.;for(int k=0;k<7;++k){at+=times[k];const Vector expected=windows[k].center+windows[k].rotation*windows[k].motion(at);for(int a=0;a<3;++a)point_error=std::max(point_error,std::abs(p[3*k+a]-expected[a]));}
  }
 }
 require(point_error<1e-10,"moving center query mismatch");Eigen::VectorXd p(6);p<<-.4,2.6,-6.4,7.6,-4.4,1.; // (u-1)^4*(u-.4)
 const auto q=without_terminal_hover_contact(p,true);const auto roots_left=roots(q);require(roots_left.size()==1 && std::abs(roots_left[0]-.4)<1e-8,"terminal contact handling removed a real crossing");
 Json out{{"passed",true},{"scene_count",scenes},{"initializations",starts},{"vehicle_mass_kg",author.mass},{"inverse_mixer_error",inverse_error},{"rotor_interval_residual_error",rotor_error},{"moving_center_error",point_error},{"terminal_hover_contact_deflation_preserves_other_crossings",true}};std::ofstream(argv[3])<<out.dump(2)<<'\n';std::cout<<out.dump()<<'\n';
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}return 0;}
