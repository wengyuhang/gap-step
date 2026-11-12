#include "geometry.hpp"
#include "drolib/solver/minco_snap.hpp"
#include <filesystem>
#include <chrono>
#include <numeric>

namespace fs=std::filesystem;

std::vector<double> center_coordinates(const Window& window,double radius) {
    if(window.circle)return {0.,0.};
    const auto vertices=inset_frame_vertices(window,radius);
    const int count=vertices.size();
    std::vector<double> distances(count),tangents(count),weights(count),result(count);
    for(int k=0;k<count;++k)distances[k]=vertices[k].norm();
    for(int k=0;k<count;++k) {
        const int next=(k+1)%count;
        const double cross=vertices[k].x()*vertices[next].y()-vertices[k].y()*vertices[next].x();
        tangents[k]=cross/(distances[k]*distances[next]+vertices[k].dot(vertices[next]));
    }
    double total=0.;
    for(int k=0;k<count;++k) {
        weights[k]=(tangents[(k+count-1)%count]+tangents[k])/distances[k];
        total+=weights[k];
    }
    for(int k=0;k<count;++k)result[k]=std::sqrt(weights[(k+1)%count]/total);
    return result;
}

int main(int argc,char** argv) {
    try {
        if(argc!=5 && argc!=6)throw std::runtime_error("Usage: restored_inputs TIMING_ARCHIVE_DIRECTORY ASSET_DIRECTORY DYNAMICS_JSON OUTPUT_DIRECTORY [DERIVATIVE_ORDER]");
        const int derivative_order=argc==6?std::stoi(argv[5]):1;
        if(derivative_order!=1 && derivative_order!=3)throw std::runtime_error("Expected PV order1 or PVAJ order3");
        const auto began=std::chrono::steady_clock::now();
        const fs::path archive=argv[1],assets=argv[2],output=argv[4];
        fs::create_directories(output);
        const std::vector<std::string> levels{"static","mixed_base","a25_f3","a25_f6","a50_f1","a50_f3","a50_f6","a75_f1","a75_f3","a75_f6"};
        Json audits=Json::array();
        for(const auto& level:levels)for(const auto& method:{"ours","center"}) {
            Json meta,scene;
            std::ifstream(archive/(level+"_"+method)/"manifest.json")>>meta;
            std::ifstream(assets/level/"scene.json")>>scene;
            const int n=meta["n"],m=meta["m"],vs=meta["velocity_start"];
            const bool free=meta["free_points"];
            const int nv=vs+3*m*derivative_order;
            meta["nvar"]=nv;meta["minco_derivative_order"]=derivative_order;
            meta["restore_historical_domain"]=true;meta["domain_bounds_removed"]=false;
            meta["point_coordinates"]="togt_shape_map";
            auto scales=meta["state_scale"].get<std::vector<double>>();scales.resize(3*m*derivative_order);meta["state_scale"]=scales;
            std::vector<Window> windows;
            std::vector<double> point_coordinates;
            for(const auto& row:scene["windows"]) {
                windows.push_back(load_window(row));
                if(free) {
                    const auto coordinates=center_coordinates(windows.back(),meta["finite_radius"]);
                    point_coordinates.insert(point_coordinates.end(),coordinates.begin(),coordinates.end());
                }
            }
            drolib::PVAJ head=drolib::PVAJ::Zero(),tail=drolib::PVAJ::Zero();
            for(int axis=0;axis<3;++axis){head(axis,0)=scene["start"][axis];tail(axis,0)=scene["goal"][axis];}
            drolib::MincoSnap minco;minco.setConditions(head,tail,n);
            double maximum_velocity_error=0.;
            Json seeds=Json::array();
            for(const auto& archived:meta["initials"]) {
                const auto previous=archived["original"].get<std::vector<double>>();
                Eigen::VectorXd times(n);Eigen::Matrix3Xd points(3,m);
                std::vector<double> original(nv),parameters(1+2*n);
                double arrival=0.;
                for(int piece=0;piece<n;++piece) {
                    original[piece]=previous[piece];times[piece]=std::exp(previous[piece]);
                    arrival+=times[piece];parameters[1+piece]=arrival;
                    if(piece<m)points.col(piece)=windows[piece].center+windows[piece].rotation*windows[piece].motion(arrival);
                    const double frequency=piece<m ? windows[piece].frequency :
                        std::accumulate(windows.begin(),windows.end(),0.,[](double sum,const Window& w){return sum+w.frequency;})/m;
                    parameters[1+n+piece]=1./std::max(1.,2.*pi*frequency);
                }
                parameters[0]=times.sum();
                std::copy(point_coordinates.begin(),point_coordinates.end(),original.begin()+n);
                minco.setParameters(points,times);
                const auto& coefficients=minco.getCoeffs();
                for(int order=1;order<=derivative_order;++order)for(int gate=0;gate<m;++gate)for(int axis=0;axis<3;++axis) {
                    double value=0.;
                    for(int power=7;power>=order;--power) {
                        double factor=1.;for(int k=0;k<order;++k)factor*=power-k;
                        value=value*times[gate]+factor*coefficients(8*gate+power,axis);
                    }
                    const int index=vs+3*((order-1)*m+gate)+axis;
                    original[index]=value;
                    maximum_velocity_error=std::max(maximum_velocity_error,std::abs(value-previous[index]));
                }
                std::vector<double> finite(nv+meta["slack_scale"].size(),0.);
                seeds.push_back(Json{{"index",archived["index"]},{"original",original},{"finite",finite},{"parameters",parameters}});
            }
            meta["initials"]=seeds;
            meta["initialization"]=derivative_order==3 ? "frozen original Sobol durations; C++ moving centers and position-only MINCO PVAJ; native PVAJ optimization" : "frozen original Sobol durations; C++ moving centers and position-only MINCO velocity; native PV optimization";
            const fs::path folder=output/(level+"_"+method);fs::create_directories(folder);
            std::ofstream(folder/"manifest.json")<<meta.dump(2)<<'\n';
            fs::copy_file(argv[3],folder/"exchange_dynamics.json",fs::copy_options::overwrite_existing);
            audits.push_back(Json{{"level",level},{"method",method},{"starts",seeds.size()},
                {"trajectory_variables",nv},{"derivative_order",derivative_order},{"initial_derivative_error_vs_archive",maximum_velocity_error}});
        }
        std::ofstream(output/"INITIALIZATION.json")<<Json{{"passed",true},{"runs",audits},
            {"preparation_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()}}.dump(2)<<'\n';
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
