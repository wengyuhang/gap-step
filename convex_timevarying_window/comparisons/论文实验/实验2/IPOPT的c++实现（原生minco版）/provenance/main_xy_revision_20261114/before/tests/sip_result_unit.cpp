#include "json.hpp"
#include "sip_exchange_policy.hpp"
#include <fstream>
#include <iostream>
#include <set>
using Json=nlohmann::json;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
Json load(const std::string& path){Json value;std::ifstream file(path);if(!file)throw std::runtime_error("Cannot open "+path);file>>value;return value;}
int main(int argc,char** argv){
 try {
  if(argc<3)throw std::runtime_error("Usage: sip_result_unit CONFIG_JSON RESULT_DIRECTORY...");
  const auto configuration=load(argv[1]);
  require(configuration.at("finite_safety_radius_m").get<double>()==sip_exchange::default_finite_radius,"config and compiled radius differ");
  for(int i=2;i<argc;++i){
   const std::string directory=argv[i];const auto result=load(directory+"/result.json");
   require(result.at("found").get<bool>(),"real smoke trajectory must pass complete verification");
   require(result.at("finite_safety_radius_m").get<double>()==.63094472,"runtime did not rebuild finite geometry at the new radius");
   require(result.at("verification_safety_radius_m").get<double>()==.62094472,"verification radius changed unexpectedly");
   const auto& calls=result.at("optimizer_calls");const auto& records=result.at("records");const auto& best=result.at("finite_incumbent");
   require(best.at("status")=="Solve_Succeeded","incumbent has unqualified status");
   double objective=INFINITY;
   for(const auto& call:calls){
    if(call.at("warm_start_used").get<bool>()) {
     require(call.at("dual_source_status")=="Solve_Succeeded","invalid multiplier source");
     require(call.at("previous_solve_status")=="Solve_Succeeded","failed previous solve reused incumbent multipliers");
    }
    if(call.at("status")=="Solve_Succeeded" && call.at("minimum_constraint").get<double>()>=-1e-5)
     objective=std::min(objective,call.at("native_objective").get<double>());
   }
   require(std::abs(best.at("objective").get<double>()-objective)<1e-9,"incumbent was overwritten by worse or unsuccessful solve");
   for(const auto& record:records){
    std::set<int> gates;
    for(const auto& point:record.at("geometry_added_witnesses"))require(gates.insert(point.at("gate").get<int>()).second,"more than one witness added to a gate");
   }
   require(calls.back().at("status")=="Solve_Succeeded","unconverged result accepted");
   require(records.back().at("global_passed").get<bool>() && records.back().at("dynamics_passed").get<bool>(),"incomplete verification accepted");
   require(std::abs(result.at("objective").get<double>()-calls.back().at("native_objective").get<double>())<1e-9,"accepted result replaced by a different incumbent");
   const auto input=load(directory+"/finite_input.json");
   if(input.at("dual_source_status")!="Solve_Succeeded" || input.at("previous_solve_status")!="Solve_Succeeded") {
    for(double lambda:input.at("lam_g0"))require(lambda==0.,"non-success constraint dual not zero");
    for(double lambda:input.at("lam_x0"))require(lambda==0.,"non-success bound dual not zero");
   } else {
    const auto& duals=best.at("lam_g");
    for(size_t row=0;row<input.at("keys").size();++row){
     const auto key=input.at("keys")[row].get<std::string>();
     if(!duals.contains(key))require(input.at("lam_g0")[row].get<double>()==0.,"new constraint row inherited another row's multiplier");
    }
   }
   std::cout<<directory<<": real SIP result invariants passed\n";
  }
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
 return 0;
}
