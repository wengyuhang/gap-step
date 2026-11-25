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

double polyline_length(const std::vector<Vector> &points) {
    double length=0.;
    for(size_t k=1;k<points.size();++k)length+=(points[k]-points[k-1]).norm();
    return length;
}

int main(int argc,char **argv) {
    try {
        if(argc!=3)throw std::runtime_error("Usage: ORIGINAL_CLASSIC_INPUTS NEW_OUTPUT");
        const fs::path source=fs::absolute(argv[1]),out=fs::absolute(argv[2]);
        if(fs::exists(out))throw std::runtime_error("Output exists");
        const auto vehicle=read(source/"vehicle.json");
        const std::string level="compact_slalom";
        const Vector start(0.,0.,3.),goal(16.,0.,3.);
        const double amplitude[7]={.20,.24,.18,.26,.22,.30,.25};
        const double frequency[7]={1.7,2.0,2.3,1.8,2.1,1.9,2.2};
        const double phase[7]={.35,1.40,2.65,3.55,4.60,5.35,.95};
        const double angles[7]={0.,0.,pi/4.,pi/2.,pi/2.,pi/4.,-pi/6.};
        const bool swing[7]={false,true,false,true,false,true,false};
        Json windows=Json::array(),rows=Json::array();
        std::vector<Vector> center_line{start},inner_line{start};
        for(int k=0;k<7;++k) {
            const double side=k%2==0?1.:-1.;
            const Vector center(2.*(k+1),side*1.1,3.);
            const double yaw=side*40.*pi/180.;
            const Vector forward(std::cos(yaw),std::sin(yaw),0.);
            Eigen::Matrix3d rotation;
            rotation.col(0)=Vector::UnitZ();
            rotation.col(2)=-forward;
            rotation.col(1)=rotation.col(2).cross(rotation.col(0));
            const bool circle=k%2==1;
            Json w{{"name","Gate"+std::to_string(k+1)},{"center",vec(center)},{"rotation",mat(rotation)},
                {"circle",circle},{"navigation_waypoint",false},{"opening_radius",circle?Json(.75):Json(nullptr)},
                {"vertices",circle?Json::array():Json{{-.75,-.75},{.75,-.75},{.75,.75},{-.75,.75}}},
                {"swing",swing[k]},{"frequency",frequency[k]},{"amplitude",amplitude[k]},{"phase",phase[k]},
                {"length",1.6},{"motion_angle_rad",angles[k]},{"sign",1.},
                {"rpy_radians",{0.,-pi/2.,yaw}}};
            w["mesh"]=frame_mesh(load_window(w));
            windows.push_back(w);
            const Vector local(0.,-side*.4499,0.);
            const Vector inner=center+rotation*local;
            center_line.push_back(center);inner_line.push_back(inner);
            rows.push_back({{"gate",k+1},{"nominal_center",vec(center)},{"normal_yaw_deg",side*40.},
                {"shape",circle?"circle":"square"},{"motion",swing[k]?"swing":"translation"},
                {"amplitude_m",amplitude[k]},{"frequency_hz",frequency[k]},{"phase_rad",phase[k]},
                {"motion_angle_deg",angles[k]*180./pi},{"geometric_inward_point",vec(inner)},
                {"geometric_inward_point_local",vec(local)}});
        }
        center_line.push_back(goal);inner_line.push_back(goal);
        Json scene{{"level",level},{"display_track","Compact alternating slalom"},{"difficulty","compact_mixed_motion"},
            {"scenario_variant","compact_slalom_fixed_planes_v1"},{"start",vec(start)},{"goal",vec(goal)},
            {"windows",windows},{"vehicle",vehicle},{"collision_radius_m",.15},{"physical_collision_radius_m",.15},
            {"planning_safety_sphere_radius_m",.30},{"stationary_terminal_contact_is_crossing",false},
            {"opening_side_or_diameter_m",1.5},{"finish_rule","given endpoint (16,0,3), zero VAJ"},
            {"gate_plane_rule","fixed vertical planes with alternating +40/-40 deg traversal normal yaw"},
            {"scene_origin","self-designed controlled slalom, geometry fixed before trajectory solves"}};
        Json pairs=Json::array();double minimum_gap=INFINITY;
        for(int j=0;j<7;++j)for(int k=0;k<j;++k) {
            const auto a=world_bounds(windows[j]),b=world_bounds(windows[k]);
            const Vector gap=(a.first-b.second).cwiseMax(b.first-a.second).cwiseMax(Vector::Zero());
            minimum_gap=std::min(minimum_gap,gap.norm());
            if(gap.norm()<.10)throw std::runtime_error("Swept frame bounds are too close");
            pairs.push_back({{"gate1",k+1},{"gate2",j+1},{"minimum_swept_aabb_separation_m",gap.norm()}});
        }
        const double center_length=polyline_length(center_line),inner_length=polyline_length(inner_line);
        if((center_length-inner_length)/center_length<.10)
            throw std::runtime_error("Predeclared geometric shortening is below 10 percent");
        save(out/"vehicle.json",vehicle);
        save(out/"assets"/level/"scene.json",scene);
        Json preflight{{"passed",true},{"selection_rule","geometric-only: inward in-aperture polyline at nominal reference centers at least 10 percent shorter than center polyline; full-motion frame bounds separated >=0.10m"},
            {"nominal_center_polyline_m",center_length},{"nominal_inward_polyline_m",inner_length},
            {"nominal_geometric_shortening_percent",100.*(center_length-inner_length)/center_length},
            {"longitudinal_spacing_m",2.},{"lateral_offset_m",1.1},{"adjacent_center_distance_m",std::sqrt(2.*2.+2.2*2.2)},
            {"interior_center_polyline_turn_deg",2.*std::atan2(2.2,2.)*180./pi},
            {"adjacent_gate_normal_change_deg",80.},{"minimum_swept_frame_aabb_gap_m",minimum_gap},
            {"geometry_metric_note","Nominal-reference polylines only. These points do not assert a continuous safe, dynamically feasible or time-optimal trajectory. Actual planning queries each original fixed motion law."},
            {"swept_frame_pairs",pairs},{"gates",rows}};
        save(out/"PREFLIGHT.json",preflight);
        for(const std::string method:{"ours","center"})
            input(scene,vehicle,source/"inputs"/("splits_moderate_"+method),out/"inputs"/(level+"_"+method));
        Json parity=Json::array();
        for(const std::string method:{"ours","center"}) {
            const auto old_dir=source/"inputs"/("splits_moderate_"+method);
            const auto old_scene=read(source/"assets/splits_moderate/scene.json");
            const auto target=out/"input_rule_parity"/method;
            input(old_scene,vehicle,old_dir,target);
            const auto before=read(old_dir/"manifest.json"),after=read(target/"manifest.json");
            double error=0.;
            for(int seed=0;seed<8;++seed)for(size_t k=0;k<before["initials"][seed]["original"].size();++k)
                error=std::max(error,std::abs(before["initials"][seed]["original"][k].get<double>()-after["initials"][seed]["original"][k].get<double>()));
            if(error>1e-8)throw std::runtime_error("Initialization rule differs from existing fixed protocol");
            parity.push_back({{"method",method},{"original_physical_seed_max_error",error},{"passed",true}});
        }
        save(out/"INITIALIZATION_RULE_PARITY.json",parity);
        save(out/"PROTOCOL.json",{{"levels",{level}},{"cases",{{{"level",level},{"gate_count",7},{"all_gates_moving",true}}}},
            {"scene_selection","geometric condition frozen before trajectory results"}});
        std::cout<<preflight.dump(2)<<'\n';
    } catch(const std::exception &error) {std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
