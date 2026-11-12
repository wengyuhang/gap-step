#include "dynamic_togt.hpp"
#include <fstream>
#include <iostream>
int main(int argc,char** argv) {
    try {
        if(argc<4)throw std::runtime_error("Usage: dynamic_togt_planner SCENE_JSON OUTPUT_DIRECTORY free|center [AUTHOR_PARAMETER_DIRECTORY] [VEHICLE_JSON] [--center-result=RESULT_JSON] [--initial-manifest=MANIFEST_JSON --initial-index=INDEX]");
        const std::string mode=argv[3];if(mode!="free" && mode!="center")throw std::runtime_error("Expected free or center");
        std::vector<std::string> positional;
        std::filesystem::path center_file,initial_file;
        int initial_index=0;
        for(int i=4;i<argc;++i) {
            const std::string argument=argv[i];
            if(argument.rfind("--center-result=",0)==0)center_file=argument.substr(16);
            else if(argument.rfind("--initial-manifest=",0)==0)initial_file=argument.substr(19);
            else if(argument.rfind("--initial-index=",0)==0)initial_index=std::stoi(argument.substr(16));
            else positional.push_back(argument);
        }
        if(positional.size()>2 || (mode=="center" && !center_file.empty()) || (mode!="center" && !initial_file.empty()))
            throw std::runtime_error("Invalid dynamic TOGT arguments");
        const std::filesystem::path params=positional.empty() ? TOGT_UPSTREAM_PARAMETER_DIR : positional[0];
        const std::filesystem::path vehicle_file=positional.size()<2 ? TOGT_X500_CONFIG : positional[1];
        Json scene,vehicle;std::ifstream(argv[1])>>scene;std::ifstream(vehicle_file)>>vehicle;
        const std::filesystem::path output=argv[2];std::filesystem::create_directories(output);
        const auto began=dynamic_togt::Clock::now();
        Json center=nullptr;
        double center_compute_s=0.;
        if(mode=="free") {
            if(center_file.empty()) {
                const auto center_begin=dynamic_togt::Clock::now();
                dynamic_togt::Planner center_planner(scene,true,params,vehicle);
                center=center_planner.solve();
                center["total_compute_s"]=std::chrono::duration<double>(dynamic_togt::Clock::now()-center_begin).count();
                center["scene_source"]=std::filesystem::absolute(argv[1]).string();
                center_file=output/"center_initial"/"result.json";
                std::filesystem::create_directories(center_file.parent_path());
                std::ofstream(center_file)<<center.dump(2)<<'\n';
            } else {
                std::ifstream(center_file)>>center;
            }
            center_compute_s=center.at("total_compute_s");
        }
        Json result;
        const auto stage_begin=dynamic_togt::Clock::now();
        if(mode=="free" && !center.value("found",false)) {
            result={{"found",false},{"method","dynamic_togt"},{"solve_s",0.},
                {"message","Center optimization did not succeed"},{"status",center.at("status")}};
        } else {
            const auto build_begin=dynamic_togt::Clock::now();
            dynamic_togt::Planner planner(scene,mode=="center",params,vehicle);
            const double build_s=std::chrono::duration<double>(dynamic_togt::Clock::now()-build_begin).count();
            Eigen::VectorXd supplied;
            if(!initial_file.empty()) {
                Json archive;std::ifstream(initial_file)>>archive;
                if(archive.at("n").get<int>()!=planner.pieces())throw std::runtime_error("Archived timing dimension mismatch");
                const auto row=archive.at("initials").at(initial_index).at("original");
                Eigen::VectorXd durations(planner.pieces());supplied.resize(planner.pieces());
                for(int piece=0;piece<planner.pieces();++piece)durations[piece]=std::exp(row.at(piece).get<double>());
                if(!durations.allFinite() || (durations.array()<=0.).any())throw std::runtime_error("Invalid archived segment durations");
                Eigen::Map<Eigen::VectorXd> k(supplied.data(),supplied.size());drolib::TrajSolver::backwardT(durations,k);
            }
            result=planner.solve(center,supplied);result["build_s"]=build_s;
            if(!initial_file.empty()) {
                result["initial_sample_index"]=initial_index;result["initial_manifest_source"]=std::filesystem::absolute(initial_file).string();
            }
        }
        result["stage_compute_s"]=std::chrono::duration<double>(dynamic_togt::Clock::now()-stage_begin).count();
        result["center_generation_s"]=center_compute_s;
        result["total_compute_s"]=center_compute_s+result["stage_compute_s"].get<double>();
        result["process_compute_s"]=std::chrono::duration<double>(dynamic_togt::Clock::now()-began).count();
        if(mode=="free")result["center_initial_source"]=std::filesystem::absolute(center_file).string();
        result["case"]={{"layout","togt_whole"},{"level",scene.value("level",std::string("custom"))}};
        bool higher_swing=true;
        for(const auto& window:scene["windows"])
            if(window["swing"].get<bool>() && std::abs(window["length"].get<double>()-2.2)>1e-9)higher_swing=false;
        result["scenario_variant"]=scene.value("scenario_variant",std::string(higher_swing ? "higher_swing_length_2p20" : "fixed_plane_dynamic_gates"));
        result["scene_source"]=std::filesystem::absolute(argv[1]).string();
        std::ofstream(output/"result.json")<<result.dump(2)<<'\n';
        std::cout<<"DYNAMIC_TOGT "<<mode<<' '<<result["found"]<<' '<<result.value("nominal_time_s",0.)<<' '<<result["total_compute_s"]<<'\n';
    } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
