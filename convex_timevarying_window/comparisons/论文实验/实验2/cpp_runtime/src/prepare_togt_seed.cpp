#include "trajectory.hpp"
#include "minco_pv.hpp"
#include <filesystem>
#include <chrono>
#include <fstream>
#include <iostream>

int main(int argc,char** argv) {
    try {
        if(argc!=5)throw std::runtime_error("Usage: prepare_togt_seed GRAPH_DIRECTORY TOGT_CENTER_RESULT SCENE_JSON OUTPUT_GRAPH_DIRECTORY");
        const auto began=std::chrono::steady_clock::now();
        namespace fs=std::filesystem;
        const fs::path input=argv[1],output=argv[4];
        if(fs::exists(output))throw std::runtime_error("Output graph directory already exists");
        Json manifest,source,scene;
        std::ifstream(input/"manifest.json")>>manifest;
        std::ifstream(argv[2])>>source;std::ifstream(argv[3])>>scene;
        if(!source.value("found",false) || source.at("method")!="dynamic_togt_center")
            throw std::runtime_error("A solved dynamic TOGT center trajectory is required");
        const int n=manifest.at("n"),m=manifest.at("m"),nv=manifest.at("nvar"),vs=manifest.at("velocity_start");
        const auto trajectory=Trajectory::load(source);
        if(n!=m+1 || trajectory.durations.size()!=static_cast<size_t>(n) ||
           trajectory.coefficients.size()!=static_cast<size_t>(n) || scene.at("windows").size()!=static_cast<size_t>(m))
            throw std::runtime_error("Seed structure does not match the graph");
        Eigen::Matrix3Xd points(3,m),states(3,3*m);
        auto original=manifest.at("initials").at(0).at("original").get<std::vector<double>>();
        if(original.size()!=static_cast<size_t>(nv))throw std::runtime_error("Invalid physical seed length");
        double arrival=0.,point_error=0.;
        for(int piece=0;piece<n;++piece) {
            if(!(trajectory.durations[piece]>0.))throw std::runtime_error("Invalid seed time");
            original[piece]=std::log(trajectory.durations[piece]);
            if(piece>=m)continue;
            arrival+=trajectory.durations[piece];
            for(int axis=0;axis<3;++axis)points(axis,piece)=source.at("crossing_points").at(piece).at(axis);
            const auto window=load_window(scene.at("windows").at(piece));
            point_error=std::max(point_error,(points.col(piece)-window.center-window.rotation*window.motion(arrival)).norm());
            for(int order=1;order<=3;++order)for(int axis=0;axis<3;++axis) {
                double value=0.;
                for(int power=order;power<8;++power) {
                    double factor=1.;for(int k=0;k<order;++k)factor*=power-k;
                    value+=factor*trajectory.coefficients[piece](power,axis);
                }
                value/=std::pow(trajectory.durations[piece],order);
                states(axis,(order-1)*m+piece)=value;
                original[vs+3*((order-1)*m+piece)+axis]=value;
            }
        }
        if(point_error>1e-8)throw std::runtime_error("Source trajectory does not cross the moving centers of this scene");
        drolib::PVAJ head=drolib::PVAJ::Zero(),tail=drolib::PVAJ::Zero();
        for(int axis=0;axis<3;++axis){head(axis,0)=scene.at("start").at(axis);tail(axis,0)=scene.at("goal").at(axis);}
        togt_pv::MincoPV minco(head,tail,n,3);
        Eigen::Map<const Eigen::VectorXd> times(trajectory.durations.data(),n);
        if(!minco.fit(points,states,times))throw std::runtime_error("Seed MincoPV reconstruction failed");
        double coefficient_error=0.;
        for(int piece=0;piece<n;++piece)for(int power=0;power<8;++power)for(int axis=0;axis<3;++axis)
            coefficient_error=std::max(coefficient_error,std::abs(minco.coefficients()(8*piece+power,axis)*
                std::pow(times[piece],power)-trajectory.coefficients[piece](power,axis)));
        if(coefficient_error>1e-5)throw std::runtime_error("Seed MincoPV reconstruction differs from source trajectory");
        auto parameters=manifest.at("initials").at(0).at("parameters").get<std::vector<double>>();
        parameters[0]=times.sum();double total=0.;
        for(int piece=0;piece<n;++piece){total+=times[piece];parameters[1+piece]=total;}
        std::vector<double> finite(nv+manifest.at("slack_scale").size(),0.);
        std::copy(original.begin(),original.end(),finite.begin());
        manifest["initials"]=Json::array({Json{{"index",0},{"original",original},{"finite",finite},{"parameters",parameters}}});
        manifest["initialization"]={{"source",fs::absolute(argv[2]).string()},
            {"source_method",source.at("method")},{"source_nominal_time_s",times.sum()},
            {"number_of_starts",1},{"coefficient_error",coefficient_error},{"center_error_m",point_error},
            {"pvaj_rule","actual derivatives of the optimized center trajectory"}};
        fs::create_directories(output);
        fs::copy_file(input/"exchange_dynamics.json",output/"exchange_dynamics.json");
        fs::copy_file(input/"exchange_dynamic_node.casadi",output/"exchange_dynamic_node.casadi");
        for(int gate=0;gate<m;++gate)for(const auto& kind:{"before","after","normalized"}) {
            const auto filename="frame_"+std::to_string(gate)+"_"+kind+".casadi";
            fs::copy_file(input/filename,output/filename);
        }
        std::ofstream(output/"manifest.json")<<manifest.dump(2)<<'\n';
        const Json diagnostic{{"preparation_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()},
            {"source",fs::absolute(argv[2]).string()},{"coefficient_error",coefficient_error},{"center_error_m",point_error}};
        std::ofstream(output/"initial_diagnostic.json")<<diagnostic.dump(2)<<'\n';
        std::cout<<diagnostic.dump()<<'\n';
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
