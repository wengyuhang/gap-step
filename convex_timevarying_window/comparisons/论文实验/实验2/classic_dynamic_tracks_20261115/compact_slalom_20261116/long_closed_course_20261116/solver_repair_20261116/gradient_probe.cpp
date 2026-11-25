#define SIP_EXPORT_PROBE
#define main sip_solver_entry
#include "sip_probe_source.cpp"
#undef main
int main(int argc,char** argv){try{
 Json raw,input;std::ifstream(argv[2])>>raw;std::ifstream(argv[3])>>input;
 Json rows=Json::array();for(auto w:raw["records"][0]["geometry_added_witnesses"]){std::string kind=w["piece"]==w["gate"]?"before":w["piece"].get<int>()==w["gate"].get<int>()+1?"after":"normalized";rows.push_back({{"gate",w["gate"]},{"piece",w["piece"]},{"kind",kind},{"values",{w["offset"]}}});}
 Planner planner(argv[1],argv[5],false,false,false,false,false,false,false,false,false,false,false,false,argv[4],true,1000.);
 planner.audit_finite(rows,input,std::string(argv[5])+"/GRADIENT.json");
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;}
