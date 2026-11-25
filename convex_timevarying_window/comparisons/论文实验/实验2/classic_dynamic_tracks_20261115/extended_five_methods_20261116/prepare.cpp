#include "geometry.hpp"
#include <filesystem>
namespace fs=std::filesystem;
Json read(const fs::path& p){std::ifstream f(p);if(!f)throw std::runtime_error("Missing "+p.string());Json j;f>>j;return j;}
void save(const fs::path& p,const Json& j){fs::create_directories(p.parent_path());std::ofstream(p)<<j.dump(2)<<'\n';}
int main(int argc,char** argv){try{
 fs::path source=fs::absolute(argv[1]),out=fs::absolute(argv[2]);if(fs::exists(out))throw std::runtime_error("Output exists");
 fs::create_directories(out);fs::copy_file(source/"vehicle.json",out/"vehicle.json");Json levels=Json::array(),audit=Json::array();
 for(std::string course:{"splits","alphapilot","airsim"})for(std::string tier:{"moderate","high","freq3","freq4"}){
  std::string origin=course+"_"+(tier=="moderate"?"moderate":"high"),level=course+"_"+tier;
  auto scene=read(source/"assets"/origin/"scene.json");double multiplier=tier=="freq3"?1.5:tier=="freq4"?2.:1.;
  scene["level"]=level;scene["difficulty"]=tier;
  for(auto& w:scene["windows"])w["frequency"]=w["frequency"].get<double>()*multiplier;
  save(out/"assets"/level/"scene.json",scene);levels.push_back(level);
  for(std::string method:{"ours","center"}){
   auto target=out/"inputs"/(level+"_"+method);fs::create_directories(target);
   for(const auto& entry:fs::directory_iterator(source/"inputs"/(origin+"_"+method)))if(entry.is_regular_file())fs::copy_file(entry.path(),target/entry.path().filename());
   auto manifest=read(target/"manifest.json");manifest["level"]=level;manifest["windows"]=scene["windows"];for(auto& w:manifest["windows"])w.erase("mesh");save(target/"manifest.json",manifest);
  }
  audit.push_back({{"level",level},{"source_level",origin},{"frequency_multiplier",multiplier},{"geometry_amplitude_motion_angle_phase_preserved",true}});
 }
 save(out/"PROTOCOL.json",{{"levels",levels},{"cases",audit}});save(out/"INPUT_AUDIT.json",audit);
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;}
