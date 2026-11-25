// Input construction below is reused verbatim from ../generate.cpp.
#include "frame_mesh.hpp"
#include "restored_kernels.hpp"
#include "drolib/solver/minco_snap.hpp"
#include <Eigen/Geometry>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <numeric>

namespace fs=std::filesystem;
Json read(const fs::path &path) {
    std::ifstream file(path);
    if(!file)throw std::runtime_error("Missing "+path.string());
    Json value;file>>value;return value;
}
void save(const fs::path &path,const Json &value) {
    fs::create_directories(path.parent_path());std::ofstream(path)<<value.dump(2)<<'\n';
}
Json vec(const Vector &v) {return {v.x(),v.y(),v.z()};}
Json mat(const Eigen::Matrix3d &v) {
    Json out=Json::array();for(int k=0;k<3;++k)out.push_back({v(k,0),v(k,1),v(k,2)});return out;
}
Vector vector(const Json &v) {return Vector(v[0],v[1],v[2]);}
std::pair<Vector,Vector> world_bounds(const Json &row) {
    const auto w=load_window(row);
    Vector low=Vector::Constant(INFINITY),high=Vector::Constant(-INFINITY);
    for(const auto &triangle:row.at("mesh"))for(const auto &v:triangle) {
        const Vector p=w.rotation*vector(v);low=low.cwiseMin(p);high=high.cwiseMax(p);
    }
    const auto motion=w.bounds(0.,w.frequency>0.?1./w.frequency:1.);
    for(int axis=0;axis<3;++axis)for(int j=0;j<3;++j) {
        low[axis]+=std::min(w.rotation(axis,j)*motion.first[j],w.rotation(axis,j)*motion.second[j]);
        high[axis]+=std::max(w.rotation(axis,j)*motion.first[j],w.rotation(axis,j)*motion.second[j]);
    }
    return {low+w.center,high+w.center};
}
void input(const Json &scene,const Json &vehicle,const fs::path &source,const fs::path &dest) {
    auto m=read(source/"manifest.json");const auto dyn=read(source/"exchange_dynamics.json");
    const auto old_seeds=m.at("initials");
    const int count=scene.at("windows").size(),n=count+1;
    const bool free=m.at("free_points");
    const int vs=n+(free?2*count:0),nv=vs+9*count;
    m["n"]=n;m["m"]=count;m["nvar"]=nv;m["velocity_start"]=vs;
    m["level"]=scene.at("level");m["vehicle"]=vehicle;
    m["windows"]=scene.at("windows");for(auto &w:m["windows"])w.erase("mesh");
    m["workspace_rows"]=0;m["restore_historical_domain"]=false;m["domain_bounds_removed"]=true;
    m["config"]["workspace_lower"]=nullptr;m["config"]["workspace_upper"]=nullptr;
    m["config"]["radius"]=.30;m["config"]["witness_slack"]=.0001;
    m["finite_radius"]=.3851;m["verification_radius"]=.385;
    m["point_erosion_radius_m"]=.3001;m["finite_safety_sphere_radius_m"]=.3001;
    m["verification_safety_sphere_radius_m"]=.30;m["body_sphere_radius_m"]=.15;
    m["scenario_variant"]=scene.at("scenario_variant");
    m["point_coordinates"]="scaled_xy";m["point_gauge_rows"]=0;
    for(const std::string key:{"point_scale_m","point_lower_m","point_upper_m","slices","state_scale","slack_scale","initials"})m[key]=Json::array();
    int domain_rows=0;
    std::vector<Window> windows;
    for(int k=0;k<count;++k) {
        const auto w=load_window(scene["windows"][k]);windows.push_back(w);
        const auto box=planar_points::domain(w,.3001);
        m["point_scale_m"].push_back(std::vector<double>(box.scale.data(),box.scale.data()+2));
        m["point_lower_m"].push_back(std::vector<double>(box.lower.data(),box.lower.data()+2));
        m["point_upper_m"].push_back(std::vector<double>(box.upper.data(),box.upper.data()+2));
        if(free) {
            m["slices"].push_back(Json{{"begin",n+2*k},{"end",n+2*k+2},{"circle",w.circle},{"navigation_waypoint",false}});
            domain_rows+=w.circle?1:4;
        }
    }
    m["point_domain_rows"]=domain_rows;
    for(int d=0;d<3;++d)for(int k=0;k<3*count;++k)m["state_scale"].push_back(d==0?10.:d==1?30.:100.);
    for(int k=0;k<n*33;++k)for(int j=0;j<6;++j)m["slack_scale"].push_back(.01/dyn["residual_scale"][j].get<double>());
    m["dynamic_residual_scale"]=dyn["residual_scale"];
    m["lower_g"]=std::vector<double>(count+domain_rows+n*198,0.);
    m["upper_g"]=Json::array();for(int k=0;k<count+domain_rows+n*198;++k)m["upper_g"].push_back(nullptr);
    std::vector<double> typical(8,0.);
    for(const auto &seed:old_seeds)for(int k=0;k<8;++k)typical[k]+=seed["original"][k].get<double>()/old_seeds.size();
    const Vector start=vector(scene.at("start")),goal=vector(scene.at("goal"));
    std::vector<double> geometric(n);Vector previous=start;
    for(int k=0;k<n;++k) {
        const Vector point=k<count?windows[k].center:goal;
        geometric[k]=(point-previous).norm()/vehicle["initial_speed_guess_mps"].get<double>();
        if(geometric[k]<.1)throw std::runtime_error("Degenerate initialization leg");
        previous=point;
    }
    drolib::PVAJ head=drolib::PVAJ::Zero(),tail=head;head.col(0)=start;tail.col(0)=goal;
    drolib::MincoSnap minco;minco.setConditions(head,tail,n);
    for(const auto &old:old_seeds) {
        Eigen::VectorXd times(n);Eigen::Matrix3Xd points(3,count);
        std::vector<double> x(nv,0.),parameters(1+2*n,0.);double at=0.;
        for(int k=0;k<n;++k) {
            const int pattern=k%8;
            times[k]=geometric[k]*std::exp(old["original"][pattern].get<double>()-typical[pattern]);
            x[k]=std::log(times[k]);at+=times[k];parameters[1+k]=at;
            parameters[1+n+k]=1./std::max(1.,2*pi*windows[std::min(k,count-1)].frequency);
            if(k<count)points.col(k)=windows[k].center+windows[k].rotation*windows[k].motion(at);
        }
        parameters[0]=at;minco.setParameters(points,times);const auto &co=minco.getCoeffs();
        for(int d=1;d<=3;++d)for(int k=0;k<count;++k)for(int axis=0;axis<3;++axis) {
            double value=0.;for(int power=7;power>=d;--power) {
                double factor=1.;for(int j=0;j<d;++j)factor*=power-j;
                value=value*times[k]+factor*co(8*k+power,axis);
            }
            x[vs+3*((d-1)*count+k)+axis]=value;
        }
        m["initials"].push_back(Json{{"index",old["index"]},{"original",x},{"parameters",parameters},
            {"finite",std::vector<double>(nv+n*198,0.)}});
    }
    planar_points::validate(m);
    auto node=restored_kernels::dynamic_node(dyn);
    native_minco::Graph graph(m,scene,windows,node,1000.);
    if(graph.expressions[6].numel()!=m["lower_g"].size())throw std::runtime_error("Finite dimension mismatch");
    for(const auto &seed:m["initials"]) {
        auto z=seed["original"].get<std::vector<double>>();
        for(int k=0;k<n;++k) {
            const double t=std::exp(z[k]);z[k]=t>1.?std::sqrt(2.*t-1.)-1.:1.-std::sqrt(2./t-1.);
        }
        const auto scales=m["state_scale"].get<std::vector<double>>();
        for(int k=vs;k<nv;++k)z[k]/=scales[k-vs];
        z.resize(nv+n*198,0.);
        const auto values=graph.trajectory(std::vector<casadi::DM>{casadi::DM(z),casadi::DM(seed["parameters"].get<std::vector<double>>())});
        const auto times=values[1].get_elements(),points=values[3].get_elements();
        double at=0.;
        for(int k=0;k<count;++k) {
            at+=times[k];const Vector expected=windows[k].center+windows[k].rotation*windows[k].motion(at);
            const Vector actual(points[3*k],points[3*k+1],points[3*k+2]);
            if((expected-actual).norm()>1e-8)throw std::runtime_error("Symbolic moving-center mismatch");
        }
    }
    save(dest/"manifest.json",m);save(dest/"exchange_dynamics.json",dyn);
    node.save((dest/"exchange_dynamic_node.casadi").string());
}


int main(int argc,char** argv) {
    try {
        if(argc!=4)throw std::runtime_error("Usage: DYNAMIC_SLALOM_INPUTS ORIGINAL_CLASSIC_INPUTS NEW_OUTPUT");
        const fs::path original=fs::absolute(argv[1]),classic=fs::absolute(argv[2]),out=fs::absolute(argv[3]);
        if(fs::exists(out))throw std::runtime_error("Output exists");
        const auto dynamic=read(original/"assets/compact_slalom/scene.json");
        auto scene=dynamic;
        scene["level"]="compact_slalom_static";
        scene["scenario_variant"]="compact_slalom_nominal_static_control_v1";
        scene["display_track"]="Compact alternating slalom, nominal static control";
        scene["difficulty"]="same nominal layout, zero motion amplitude";
        for(auto& w:scene["windows"])w["amplitude"]=0.;
        const auto vehicle=read(original/"vehicle.json");
        for(int gate=0;gate<scene.at("windows").size();++gate) {
            const auto w=load_window(scene.at("windows").at(gate));
            for(double t:{0.,.17,1.,3.,10.,100.}) {
                if(w.motion(t).norm()!=0. || w.motion_velocity(t).norm()!=0.)
                    throw std::runtime_error("Static scene still moves");
            }
            auto restored=scene.at("windows").at(gate);
            restored["amplitude"]=dynamic.at("windows").at(gate).at("amplitude");
            if(restored!=dynamic.at("windows").at(gate))throw std::runtime_error("Non-motion window geometry changed");
        }
        const auto prefix=original/"inputs/compact_slalom_center/manifest.json";
        const auto previous=read(prefix);
        save(out/"vehicle.json",vehicle);
        save(out/"assets/compact_slalom_static/scene.json",scene);
        Json initialParity=Json::array();
        for(const std::string method:{"ours","center"}) {
            const auto destination=out/"inputs"/("compact_slalom_static_"+method);
            input(scene,vehicle,classic/"inputs"/("splits_moderate_"+method),destination);
            const auto generated=read(destination/"manifest.json");
            const auto before=read(original/"inputs"/("compact_slalom_"+method)/"manifest.json");
            double error=0.;
            for(int seed=0;seed<8;++seed)for(int piece=0;piece<8;++piece)
                error=std::max(error,std::abs(generated.at("initials").at(seed).at("original").at(piece).get<double>()-
                    before.at("initials").at(seed).at("original").at(piece).get<double>()));
            if(error!=0.)throw std::runtime_error("Original eight timing patterns changed");
            initialParity.push_back({{"method",method},{"segment_log_time_difference",error}});
        }
        save(out/"STATIC_PARITY.json",{{"passed",true},{"static_reference","nominal stored centers; all amplitudes zero"},
            {"dynamic_scene",(original/"assets/compact_slalom/scene.json").string()},
            {"window_changes",{"amplitude only"}},{"gate_count",scene.at("windows").size()},
            {"geometries_meshes_planes_order_endpoints_vehicle_equal",true},{"initial_timing_parity",initialParity},
            {"pvaj_rule","same original MINCO generation rule, applied to static target centers"}});
        std::cout<<"Static input generated; all 7 amplitudes zero; geometry and 8 timing patterns identical\n";
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
