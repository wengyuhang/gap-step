#include "json.hpp"
#include "sip_exchange_policy.hpp"
#include <fstream>
#include <iostream>
#include <set>
#include <map>
using Json=nlohmann::json;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
Json load(const std::string& path){Json value;std::ifstream file(path);if(!file)throw std::runtime_error("Cannot open "+path);file>>value;return value;}
int main(int argc,char** argv){
 try {
  if(argc<3)throw std::runtime_error("Usage: sip_result_unit CONFIG_JSON RESULT_DIRECTORY...");
  const auto configuration=load(argv[1]);
  const double expected_finite=configuration.at("finite_safety_radius_m");
  const double expected_verification=configuration.value("verification_safety_radius_m",.62094472);
  if(!configuration.contains("name"))require(expected_finite==sip_exchange::default_finite_radius,"legacy config and compiled radius differ");
  for(int i=2;i<argc;++i){
   const std::string directory=argv[i];const auto result=load(directory+"/result.json");
   require(result.at("found").get<bool>(),"real smoke trajectory must pass complete verification");
   require(result.at("finite_safety_radius_m").get<double>()==expected_finite,"runtime finite radius differs from selected vehicle profile");
   require(result.at("verification_safety_radius_m").get<double>()==expected_verification,"verification radius differs from selected vehicle profile");
   const auto& calls=result.at("optimizer_calls");const auto& records=result.at("records");
   double objective=INFINITY;
   std::map<int,Json> call_by_round;
   for(const auto& call:calls){
    call_by_round[call.at("round")]=call;
    if(call.at("warm_start_used").get<bool>()) {
     require(call.at("dual_source_status")=="Solve_Succeeded","invalid multiplier source");
     require(call.at("previous_solve_status")=="Solve_Succeeded","failed previous solve reused incumbent multipliers");
    }
    if(call.at("status")=="Solve_Succeeded" && call.at("minimum_constraint").get<double>()>=-1e-5)
     objective=call.at("native_objective").get<double>();
   }
   if(result.contains("finite_incumbent")) {
    const auto& best=result.at("finite_incumbent");
    require(best.at("status")=="Solve_Succeeded","finite archive has unqualified status");
    require(std::abs(best.at("objective").get<double>()-objective)<1e-9,"finite archive must be from latest successful constraint set");
   }
   for(const auto& record:records){
    const auto& call=call_by_round.at(record.at("round"));
    require(sip_exchange::finite_ready(call.at("status"),call.at("constraint_violation_inf"),call.at("finite_curve")),"verification or witness exchange occurred before finite problem was ready");
    std::set<int> gates;
    for(const auto& point:record.at("geometry_added_witnesses"))require(gates.insert(point.at("gate").get<int>()).second,"more than one witness added to a gate");
   }
   require(sip_exchange::optimizer_success(calls.back().at("status")),"unconverged result accepted");
   require(records.back().at("global_passed").get<bool>() && records.back().at("dynamics_passed").get<bool>(),"incomplete verification accepted");
   require(std::abs(result.at("objective").get<double>()-calls.back().at("native_objective").get<double>())<1e-9,"accepted result replaced by a different incumbent");
   const auto input=load(directory+"/finite_input.json");
   if(input.at("dual_source_status")!="Solve_Succeeded" || input.at("previous_solve_status")!="Solve_Succeeded") {
    for(double lambda:input.at("lam_g0"))require(lambda==0.,"non-success constraint dual not zero");
    for(double lambda:input.at("lam_x0"))require(lambda==0.,"non-success bound dual not zero");
   }

   std::cout<<directory<<": real SIP result invariants passed\n";
  }
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
 return 0;
}
