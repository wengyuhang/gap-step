#include "trajectory.hpp"
#include "minco_pv.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
namespace fs=std::filesystem;
Json read(const fs::path &file) {
    std::ifstream stream(file);
    if(!stream)throw std::runtime_error("Missing "+file.string());
    Json value;stream>>value;return value;
}
int main(int argc,char **argv) {
    try {
        if(argc!=5)throw std::runtime_error("Usage: PREPARED_INPUT RAW_RESULT SCENE NEW_OUTPUT");
        const auto manifest=read(fs::path(argv[1])/"manifest.json"),raw=read(argv[2]),scene=read(argv[3]);
        const fs::path output=argv[4];
        if(fs::exists(output))throw std::runtime_error("Output exists");
        const auto snapshot=raw.at("finite_incumbent");
        if(snapshot.at("status")!="Solve_Succeeded")throw std::runtime_error("Strict finite snapshot required");
        const auto physical=snapshot.at("variables").get<std::vector<double>>();
        const int n=manifest.at("n"),m=manifest.at("m"),vs=manifest.at("velocity_start");
        Eigen::VectorXd times(n);Eigen::Matrix3Xd points(3,m),states(3,3*m);
        double at=0.;Json crossings=Json::array();
        for(int piece=0;piece<n;++piece) {
            times[piece]=std::exp(physical.at(piece));at+=times[piece];
            if(piece>=m)continue;
            const auto w=load_window(scene.at("windows").at(piece));Vector q=Vector::Zero();
            if(manifest.at("free_points").get<bool>()){q.x()=physical.at(n+2*piece);q.y()=physical.at(n+2*piece+1);}
            points.col(piece)=w.center+w.rotation*(q+w.motion(at));
            for(int order=1;order<=3;++order)for(int axis=0;axis<3;++axis)
                states(axis,(order-1)*m+piece)=physical.at(vs+3*((order-1)*m+piece)+axis);
            crossings.push_back({{"gate",piece+1},{"at_s",at},{"point",{points(0,piece),points(1,piece),points(2,piece)}},{"local",{q.x(),q.y()}}});
        }
        drolib::PVAJ head=drolib::PVAJ::Zero(),tail=head;
        for(int axis=0;axis<3;++axis){head(axis,0)=scene.at("start").at(axis);tail(axis,0)=scene.at("goal").at(axis);}
        togt_pv::MincoPV minco(head,tail,n,3);
        if(!minco.fit(points,states,times))throw std::runtime_error("Snapshot reconstruction failed");
        Json coefficients=Json::array();double nodeError=0.;
        for(int piece=0;piece<n;++piece) {
            Curve curve;for(int power=0;power<8;++power)for(int axis=0;axis<3;++axis)
                curve(power,axis)=minco.coefficients()(8*piece+power,axis)*std::pow(times[piece],power);
            coefficients.push_back(curve_json(curve));
            if(piece<m)nodeError=std::max(nodeError,(evaluate(curve,1.)-points.col(piece)).norm());
        }
        if(nodeError>1e-8)throw std::runtime_error("Snapshot node mismatch");
        fs::create_directories(output);
        Json result{{"found",false},{"method",manifest.at("method")},{"level",manifest.at("level")},
            {"durations",std::vector<double>(times.data(),times.data()+times.size())},{"coefficients",coefficients},
            {"nominal_time_s",times.sum()},{"objective",snapshot.at("objective")},{"source_round",snapshot.at("round")},
            {"source",fs::absolute(argv[2]).string()},{"crossings",crossings},{"reconstructed_node_max_error_m",nodeError},
            {"evidence_status","strict finite-feasible diagnostic snapshot; original full SIP not passed"}};
        std::ofstream(output/"result.json")<<result.dump(2)<<'\n';
        std::cout<<times.sum()<<' '<<nodeError<<'\n';
    } catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
