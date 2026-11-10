#define SIP_EXPORT_PROBE
#define main sip_solver_entry
#include "../src/sip_planner.cpp"
#undef main
int main(int argc,char** argv) {
    try {
        if(argc!=4)throw std::runtime_error("Usage: sip_probe GRAPH_DIRECTORY WITNESSES_JSON OUTPUT_DIRECTORY");
        Json witnesses;std::ifstream(argv[2])>>witnesses;
        Planner planner(argv[1],argv[3]);planner.export_problem(witnesses,argv[3]);
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
