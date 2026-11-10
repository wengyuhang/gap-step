#include "geometry.hpp"
#include "process.hpp"
#include <filesystem>
#include <thread>
#include <chrono>
namespace fs=std::filesystem;
int main(int argc,char** argv) {
    try {
        if(argc!=6)throw std::runtime_error("Usage: wang_batch PLANNER GRAPH_ROOT COMPILED_ROOT OUTPUT_ROOT ASSET_ROOT");
        const std::vector<std::string> levels{"static","mixed_base","a25_f3","a25_f6","a50_f1","a50_f3","a50_f6","a75_f1","a75_f3","a75_f6"};
        const fs::path graphs=fs::absolute(argv[2]),compiled=fs::absolute(argv[3]),out=fs::absolute(argv[4]);
        fs::create_directories(out);Json summary=Json::array();
        for(const auto& level:levels) {
            while(true) {
                bool ready=true;
                for(int gate=0;gate<7;++gate) {
                    std::ifstream log("/tmp/window_cpp_compile_wang_"+level+"_"+std::to_string(gate)+".log");
                    const std::string text((std::istreambuf_iterator<char>(log)),std::istreambuf_iterator<char>());
                    ready=ready && text.find("COMPILED ")!=std::string::npos;
                }
                if(ready)break;
                std::this_thread::sleep_for(std::chrono::seconds(5));
            }
            const auto folder=out/level;fs::create_directories(folder);
            run_process({argv[1],(graphs/(level+"_wang2023")).string(),folder.string(),"verify",(compiled/level).string()});
            Process solver;solver.start({"taskset","-c","23",argv[1],(graphs/(level+"_wang2023")).string(),folder.string(),"solve",(compiled/level).string()},(folder/"solve.log").string());
            if(solver.wait()!=0)throw std::runtime_error("Compiled Wang solve failed at "+level);
            Json raw;std::ifstream(folder/"result.json")>>raw;
            summary.push_back(Json{{"level",level},{"found",raw["found"]},{"solve_s",raw["solve_s"]},
                {"nominal_time_s",raw["nominal_time_s"]},{"source",(folder/"result.json").string()}});
            std::ofstream(out/"summary.json")<<summary.dump(2)<<'\n';
            std::cout<<"WANG_COMPILED "<<level<<' '<<raw["found"]<<' '<<raw["solve_s"]<<std::endl;
        }
        std::ofstream(out/"status.json")<<Json{{"state","complete"},{"completed",summary.size()}}.dump(2)<<'\n';
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
