#include "planar_points.hpp"
#include <filesystem>
namespace fs=std::filesystem;
Json read(const fs::path& p){std::ifstream f(p);if(!f)throw std::runtime_error("Missing "+p.string());Json j;f>>j;return j;}
void save(const fs::path& p,const Json& j){fs::create_directories(p.parent_path());std::ofstream(p)<<j.dump(2)<<'\n';}
int main(int argc,char** argv){try{
 fs::path in=fs::absolute(argv[1]),out=fs::absolute(argv[2]);if(fs::exists(out))throw std::runtime_error("Output exists");fs::create_directories(out);fs::copy(in,out,fs::copy_options::recursive);
 auto vehicle=read(in/"vehicle.json");
 for(const std::string key:{"planning_proxy_radius_m","planning_safety_sphere_radius_m","nominal_point_inset_m"})vehicle[key]=.20;
 vehicle["finite_safety_radius_m"]=.2851;vehicle["verification_safety_radius_m"]=.285;vehicle["extra_point_clearance_m"]=.05;
 vehicle["collision_proxy_note"]="physical collision sphere0.15m; planning safety sphere0.20m";
 save(out/"vehicle.json",vehicle);
 auto protocol=read(out/"PROTOCOL.json");Json audit=Json::array();
 for(auto level:protocol["levels"]){std::string name=level;auto scene=read(in/"assets"/name/"scene.json");auto originalWindows=scene["windows"];
 scene["vehicle"]=vehicle;scene["planning_safety_sphere_radius_m"]=.20;save(out/"assets"/name/"scene.json",scene);
 if(scene["windows"]!=originalWindows || scene["physical_collision_radius_m"]!=.15)throw std::runtime_error("Scene geometry or body collision radius changed");
 for(std::string method:{"ours","center"}){
 auto path=out/"inputs"/(name+"_"+method)/"manifest.json";auto m=read(path);m["vehicle"]=vehicle;m["finite_radius"]=.2851;m["verification_radius"]=.285;m["point_erosion_radius_m"]=.2001;m["finite_safety_sphere_radius_m"]=.2001;m["verification_safety_sphere_radius_m"]=.20;m["config"]["radius"]=.20;
 for(int gate=0;gate<m["m"].get<int>();++gate){auto d=planar_points::domain(load_window(m["windows"][gate]),.2001);for(int axis=0;axis<2;++axis){m["point_scale_m"][gate][axis]=d.scale[axis];m["point_lower_m"][gate][axis]=d.lower[axis];m["point_upper_m"][gate][axis]=d.upper[axis];}}
 planar_points::validate(m);save(path,m);
 }
 audit.push_back({{"level",name},{"windows_unchanged",true},{"collision_sphere_radius_m",.15},{"planning_sphere_radius_m",.20},{"finite_point_erosion_m",.2001}});
 }
 save(out/"SAFETY_AUDIT.json",audit);
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;}
