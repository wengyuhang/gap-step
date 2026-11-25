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
int main(int argc,char **argv) {
    try {
        const auto began=std::chrono::steady_clock::now();
        if(argc!=5)throw std::runtime_error("Usage: classic_inputs SOURCE_JSON FULL7_TEMPLATE VEHICLE_CONFIG NEW_OUTPUT");
        const auto tracks=read(argv[1]);const fs::path templates=argv[2],out=fs::absolute(argv[4]);
        if(fs::exists(out))throw std::runtime_error("Use a new input directory");
        const auto vehicle=read(argv[3]);save(out/"vehicle.json",vehicle);
        Json levels=Json::array(),cases=Json::array(),preflight=Json::array();
        for(const auto &track:tracks) {
            const std::string id=track["name"],display=track["display_name"];
            const int count=track["gates"].size();
            double minimum_z=INFINITY;for(const auto &g:track["gates"])minimum_z=std::min(minimum_z,g["position"][2].get<double>());
            const double lift=3.-minimum_z;
            const Vector start=vector(track["source_start"])+Vector(0,0,lift);
            std::vector<int> selected;
            if(id=="splits")selected={2,3,6,7};
            else if(id=="alphapilot")selected={2,3,5,7,9,10};
            else for(int k=2;k<=count;k+=2)selected.push_back(k);
            for(int tier=0;tier<2;++tier) {
                const std::string difficulty=tier==0?"moderate":"high",level=id+"_"+difficulty;
                levels.push_back(level);Json windows=Json::array(),motion_table=Json::array();
                Vector previous=start;
                for(int k=0;k<count;++k) {
                    const auto source=track["gates"][k];const auto q=source["rotation"];
                    const Vector center=vector(source["position"])+Vector(0,0,lift);
                    const Vector next=k+1<count ? vector(track["gates"][k+1]["position"])+Vector(0,0,lift)
                                                 : vector(track["source_goal"])+Vector(0,0,lift);
                    const Vector incoming=(center-previous).normalized();
                    Vector outgoing=next-center;
                    if(outgoing.norm()>1e-8)outgoing.normalize();
                    Vector forward=incoming+outgoing;forward.z()=0.;
                    if(forward.norm()<1e-8) {forward=incoming;forward.z()=0.;}
                    if(forward.norm()<1e-8)throw std::runtime_error("Undefined horizontal gate normal");
                    forward.normalize();
                    Eigen::Matrix3d R;R.col(0)=Vector::UnitZ();R.col(2)=-forward;
                    R.col(1)=R.col(2).cross(R.col(0));
                    const double sign=1.;previous=center;
                    const auto found=std::find(selected.begin(),selected.end(),k+1);
                    const bool moving=found!=selected.end();const int index=found-selected.begin();
                    int kind=index%6;
                    if(id=="splits" && moving){const int kinds[4]={0,1,3,2};kind=kinds[index];}
                    const bool swing=moving && (kind==0 || kind==3 || kind==5);
                    const double angle=kind==1 || kind==3?pi/2.:kind==2 || kind==5?pi/4.:0.;
                    const double amp=moving?(tier==0?.25:.60)*(.80+.05*(k%5)):0.;
                    const double frequency=moving?(tier==0?.60:2.)*(.85+.05*((k*3)%7)):0.;
                    const double phase=std::fmod((k+1)*2.399963229728653,2*pi);
                    const bool circle=k%2==1;
                    Json w{{"name","Gate"+std::to_string(k+1)},{"center",vec(center)},{"rotation",mat(R)},
                        {"circle",circle},{"navigation_waypoint",false},{"source_rotation_wxyz",q},{"opening_radius",circle?Json(.75):Json(nullptr)},
                        {"vertices",circle?Json::array():Json{{-.75,-.75},{.75,-.75},{.75,.75},{-.75,.75}}},
                        {"swing",swing},{"frequency",frequency},{"amplitude",amp},{"phase",phase},
                        {"length",1.6},{"motion_angle_rad",angle},{"sign",sign},
                        {"rpy_radians",{0.,-pi/2.,std::atan2(-R(1,2),-R(0,2))}}};
                    w["mesh"]=frame_mesh(load_window(w));windows.push_back(w);
                    const std::string label=!moving?"static":kind==0?"top_pivot_swing":kind==1?"vertical_translation":kind==2?"diagonal_translation":kind==3?"side_pivot_swing":kind==4?"horizontal_translation":"diagonal_pivot_swing";
                    motion_table.push_back(Json{{"gate",k+1},{"motion",label},{"amplitude_m",amp},{"frequency_hz",frequency},{"phase_rad",phase}});
                }
                const auto last=load_window(windows.back());
                const Vector goal=last.center-5.*windows.back()["sign"].get<double>()*last.rotation.col(2);
                Json scene{{"level",level},{"display_track",display},{"difficulty",difficulty},
                    {"scenario_variant","classic_fixed_planes_mixed_motion_v1"},{"start",vec(start)},{"goal",vec(goal)},
                    {"windows",windows},{"vehicle",vehicle},{"collision_radius_m",.15},
                    {"physical_collision_radius_m",.15},{"planning_safety_sphere_radius_m",.30},
                    {"stationary_terminal_contact_is_crossing",false},{"source_url",track["source_url"]},
                    {"source_sha256",track["sha256"]},{"height_translation_m",lift},
                    {"gate_plane_rule","fixed vertical plane, normal follows horizontal incoming/outgoing unit-vector bisector"},{"opening_side_or_diameter_m",1.5},
                    {"finish_rule","5 m beyond final plane along nominal traversal normal, zero VAJ"}};
                double min_gap=INFINITY,route_length=0.;previous=start;
                Json pairs=Json::array();
                for(int j=0;j<count;++j) {
                    const auto a=world_bounds(windows[j]);route_length+=(load_window(windows[j]).center-previous).norm();previous=load_window(windows[j]).center;
                    for(int k=0;k<j;++k) {
                        const auto b=world_bounds(windows[k]);
                        const Vector gap=(a.first-b.second).cwiseMax(b.first-a.second).cwiseMax(Vector::Zero());
                        const double separation=gap.norm();min_gap=std::min(min_gap,separation);
                        if(separation<.10)throw std::runtime_error("Swept frame AABBs overlap or approach within 0.1m: "+level+" gates "+std::to_string(k+1)+"/"+std::to_string(j+1));
                        pairs.push_back(Json{{"gate1",k+1},{"gate2",j+1},{"swept_aabb_gap_m",separation}});
                    }
                }
                route_length+=(goal-previous).norm();
                save(out/"assets"/level/"scene.json",scene);
                save(out/"assets"/level/"SCENE_AUDIT.json",Json{{"passed",true},{"swept_frame_pairs",pairs},
                    {"minimum_swept_frame_aabb_gap_m",min_gap},{"motion_table",motion_table},
                    {"all_gate_planes_fixed",true},{"opening_m",1.5},{"source_gate_count",count}});
                for(const std::string method:{"ours","center"})input(scene,vehicle,templates/"inputs"/("static_"+method),out/"inputs"/(level+"_"+method));
                cases.push_back(Json{{"level",level},{"display_track",display},{"difficulty",difficulty},{"gate_count",count},
                    {"moving_gates",selected},{"center_route_length_m",route_length},{"height_translation_m",lift},
                    {"minimum_swept_frame_aabb_gap_m",min_gap},{"motion_table",motion_table}});
                preflight.push_back(Json{{"level",level},{"passed",true},{"gate_count",count},{"starts_per_method",8}});
                std::cout<<"GENERATED "<<level<<" gates "<<count<<" route "<<route_length<<std::endl;
            }
        }
        save(out/"PROTOCOL.json",Json{{"levels",levels},{"cases",cases},{"planning_safety_sphere_radius_m",.30},
            {"physical_collision_radius_m",.15},{"opening_m",1.5},{"all_planning_code","C++"},
            {"predeclared_source_tracks",{"SplitS","AlphaPilot","AirSim"}},
            {"source_version","uzh-rpg/agile_flight 03dc1179c011f414aba1264bb6db92679be66b4f"}});
        save(out/"PREFLIGHT.json",Json{{"passed",true},{"checks",preflight},
            {"input_generation_wall_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()}});
    }catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
}
