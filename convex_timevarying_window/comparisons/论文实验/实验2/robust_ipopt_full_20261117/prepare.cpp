#include "planar_points.hpp"
#include <filesystem>
namespace fs=std::filesystem;
Json read(const fs::path&p){std::ifstream f(p);if(!f)throw std::runtime_error("Missing "+p.string());Json j;f>>j;return j;}
void save(const fs::path&p,const Json&j){fs::create_directories(p.parent_path());std::ofstream(p)<<j.dump(2)<<'\n';}
int main(int argc,char**argv){try{
 if(argc!=4)throw std::runtime_error("SEVEN CLASSICS NEW_OUTPUT");fs::path out=fs::absolute(argv[3]);if(fs::exists(out))throw std::runtime_error("Output exists");
 auto v=read(fs::path(argv[2])/"vehicle.json");save(out/"vehicle.json",v);Json levels=Json::array(),audit=Json::array();
 for(int group=1;group<=2;++group){fs::path in=fs::absolute(argv[group]);auto protocol=read(in/"PROTOCOL.json");
 for(auto lv:protocol["levels"]){std::string name=lv;auto scene=read(in/"assets"/name/"scene.json");const auto original=scene;
 scene["vehicle"]=v;scene["collision_radius_m"]=.15;scene["physical_collision_radius_m"]=.15;scene["planning_safety_sphere_radius_m"]=.20;
 for(auto key:{"windows","start","goal"})if(scene[key]!=original[key])throw std::runtime_error("Scene changed");
 save(out/"assets"/name/"scene.json",scene);
 for(std::string method:{"ours","center"}){auto src=in/"inputs"/(name+"_"+method),dst=out/"inputs"/(name+"_"+method);auto m=read(src/"manifest.json");
 if(m["windows"].size()!=scene["windows"].size())throw std::runtime_error("Window mismatch");
 m["vehicle"]=v;m["body_sphere_radius_m"]=.15;m["finite_radius"]=.2851;m["verification_radius"]=.285;m["point_erosion_radius_m"]=.2001;m["finite_safety_sphere_radius_m"]=.2001;m["verification_safety_sphere_radius_m"]=.20;m["config"]["radius"]=.20;
 for(int k=0;k<m["m"].get<int>();++k){auto d=planar_points::domain(load_window(m["windows"][k]),.2001);for(int axis=0;axis<d.scale.size();++axis){m["point_scale_m"][k][axis]=d.scale[axis];m["point_lower_m"][k][axis]=d.lower[axis];m["point_upper_m"][k][axis]=d.upper[axis];}}
 planar_points::validate(m);save(dst/"manifest.json",m);auto dyn=read(src/"exchange_dynamics.json");
 if(dyn["mass"]!=v["mass"] || dyn["gravity"]!=v["gravity"])throw std::runtime_error("Dynamics vehicle mismatch");save(dst/"exchange_dynamics.json",dyn);
 }
 levels.push_back(name);audit.push_back({{"level",name},{"source",in.string()},{"windows_and_endpoints_unchanged",true},{"planning_radius",.20},{"collision_radius",.15}});
 }}save(out/"PROTOCOL.json",{{"levels",levels},{"planning_only",true},{"methods",{"ours","center"}}});save(out/"SCENE_AUDIT.json",audit);std::cout<<levels.size()<<" conditions prepared\n";
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
