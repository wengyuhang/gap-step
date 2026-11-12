#define SIP_EXPORT_PROBE
#define main sip_solver_entry
#include "../src/sip_planner.cpp"
#undef main
int main(int argc,char** argv) {
    try {
        if(argc!=4 && argc!=6)throw std::runtime_error("Usage: sip_probe GRAPH_DIRECTORY WITNESSES_JSON OUTPUT_DIRECTORY [--exchange DYNAMIC_WITNESSES_JSON]");
        Json witnesses;std::ifstream(argv[2])>>witnesses;
        const bool exchange=argc==6 && std::string(argv[4])=="--exchange";
        if(argc==6 && !exchange)throw std::runtime_error("Unknown probe option");
        std::filesystem::create_directories(argv[3]);
        Json dynamics=Json::array();if(exchange)std::ifstream(argv[5])>>dynamics;
        Planner planner(argv[1],argv[3],false,false,exchange,exchange,exchange);
        planner.export_problem(witnesses,argv[3],dynamics);
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
