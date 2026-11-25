#include "trajectory.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
namespace fs=std::filesystem;
Json load(const fs::path&p){Json d;std::ifstream f(p);if(!f)throw std::runtime_error("Missing "+p.string());f>>d;return d;}
int main(int argc,char**argv){try {
 if(argc!=4)throw std::runtime_error("Usage: ORIGINAL_GRAPH TARGET_SCENE NEW_STATIC_INPUT");
 const fs::path base=fs::absolute(argv[1]),out=fs::absolute(argv[3]);if(fs::exists(out))throw std::runtime_error("Output exists");
 auto m=load(base/"manifest.json"),scene=load(argv[2]);
 for(auto&w:m["windows"])w["amplitude"]=0.;for(auto&w:scene["windows"])w["amplitude"]=0.;
 m["solver_options"]["ipopt.print_level"]=5;
 fs::create_directories(out);for(const auto* f:{"exchange_dynamic_node.casadi","exchange_dynamics.json"})fs::copy_file(base/f,out/f);
 std::ofstream(out/"manifest.json")<<m.dump(2)<<'\n';std::ofstream(out/"scene.json")<<scene.dump()<<'\n';
 std::cout<<"Prepared zero-amplitude initializer; original target remains unchanged\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
