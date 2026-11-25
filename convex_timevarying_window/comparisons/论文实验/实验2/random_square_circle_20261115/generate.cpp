#include "frame_mesh.hpp"
#include "restored_kernels.hpp"
#include "drolib/solver/minco_snap.hpp"
#include <filesystem>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>
#include <chrono>

namespace fs = std::filesystem;
Json read(const fs::path &path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Missing " + path.string());
    Json value; file >> value; return value;
}
void save(const fs::path &path, const Json &value) {
    fs::create_directories(path.parent_path());
    std::ofstream(path) << value.dump(2) << '\n';
}
Json vec(const Vector &v) { return {v.x(), v.y(), v.z()}; }
Json mat(const Eigen::Matrix3d &v) {
    Json result = Json::array();
    for (int r = 0; r < 3; ++r) result.push_back({v(r,0), v(r,1), v(r,2)});
    return result;
}
struct Spline {
    std::vector<Vector> controls;
    Vector sample(double at) const {
        at -= std::floor(at);
        const double x = at * controls.size();
        const int i = static_cast<int>(x);
        const double u = x-i, v = 1.-u;
        const double b[4] = {v*v*v/6., (3*u*u*u-6*u*u+4)/6.,
                             (-3*u*u*u+3*u*u+3*u+1)/6., u*u*u/6.};
        Vector p = Vector::Zero();
        for (int k=0;k<4;++k) p += b[k]*controls[(i+k)%controls.size()];
        return p;
    }
    Vector tangent(double at) const { return (sample(at+1e-5)-sample(at-1e-5)).normalized(); }
};
struct Layout {
    Spline spline;
    std::vector<double> parameters;
    std::vector<Vector> centers;
    std::vector<bool> circles, swings;
    std::vector<double> amplitude_factor, frequency_factor, phases;
    double spline_length;
    int attempts;
};
Layout generate(unsigned seed, int family) {
    std::mt19937 rng(seed);
    auto uniform = [&](double low,double high) {
        return low+(high-low)*(rng()/4294967296.);
    };
    for(int attempt=0;attempt<1000;++attempt) {
        Layout l;
        const double rx = (family==0?11.:family==1?12.:11.)*uniform(.95,1.05);
        const double ry = (family==0?9.:family==1?8.5:10.)*uniform(.95,1.05);
        const double phase=uniform(0.,2*pi);
        for(int k=0;k<10;++k) {
            const double a=2*pi*k/10.+uniform(-.075,.075);
            const double modulation=1.+(family==0?.05:family==1?.16:.11)*std::sin(3*a+phase);
            const double z=5.+(family==2?1.15:.65)*std::sin(2*a+phase)+uniform(-.25,.25);
            l.spline.controls.push_back(Vector(rx*modulation*std::cos(a),ry*modulation*std::sin(a),z));
        }
        const int samples=8192;
        std::vector<double> length(samples+1,0.);
        Vector previous=l.spline.sample(0.);
        for(int k=1;k<=samples;++k) {
            const Vector p=l.spline.sample(k/double(samples));
            length[k]=length[k-1]+(p-previous).norm();previous=p;
        }
        l.spline_length=length.back();
        for(int k=0;k<7;++k) {
            const double target=(k+.5)*l.spline_length/7.;
            const int j=std::lower_bound(length.begin(),length.end(),target)-length.begin();
            const double t=(j-1+(target-length[j-1])/(length[j]-length[j-1]))/samples;
            l.parameters.push_back(t);l.centers.push_back(l.spline.sample(t));
        }
        l.circles={false,true,false,true,false,true,false};
        l.swings={false,true,false,true,false,true,false};
        for(int i=6;i>0;--i) {
            const int j=rng()%(i+1);
            const bool b=l.circles[i];l.circles[i]=l.circles[j];l.circles[j]=b;
            const int q=rng()%(i+1);
            const bool c=l.swings[i];l.swings[i]=l.swings[q];l.swings[q]=c;
        }
        for(int k=0;k<7;++k) {
            l.amplitude_factor.push_back(uniform(.8,1.));
            l.frequency_factor.push_back(uniform(.8,1.2));
            l.phases.push_back(uniform(0.,2*pi));
        }
        // Geometric filtering uses only the prescribed scene, never a method's result.
        bool valid=true;
        for(int i=0;i<7;++i) for(int j=0;j<i;++j) {
            auto extent=[&](int k) {
                const double frame=l.circles[k]?1.37:std::sqrt(2.)*1.285+.085;
                const double a=.9*l.amplitude_factor[k];
                const double rise=l.swings[k]?1.6-std::sqrt(1.6*1.6-a*a):0.;
                return frame+std::hypot(a,rise)+.30;
            };
            if((l.centers[i]-l.centers[j]).norm()<extent(i)+extent(j)+.20) valid=false;
        }
        if(!valid)continue;
        l.attempts=attempt+1;
        return l;
    }
    throw std::runtime_error("No geometrically valid layout in 1000 attempts");
}
int main(int argc,char **argv) {
    try {
        const auto began=std::chrono::steady_clock::now();
        if(argc!=4)throw std::runtime_error("Usage: random_inputs FULL7_TEMPLATE VEHICLE_CONFIG NEW_OUTPUT");
        const fs::path old=argv[1],profile=argv[2],out=fs::absolute(argv[3]);
        if(fs::exists(out))throw std::runtime_error("Use a new input directory");
        auto vehicle=read(profile);
        vehicle["planning_proxy_radius_m"]=.30;
        vehicle["planning_safety_sphere_radius_m"]=.30;
        vehicle["physical_collision_radius_m"]=.15;
        save(out/"vehicle.json",vehicle);
        const std::vector<std::string> tiers{"low","medium","high"};
        const double amplitudes[3]={.25,.55,.90},frequencies[3]={.35,.85,1.80};
        Json levels=Json::array(),cases=Json::array(),preflight=Json::array();
        for(int family=0;family<3;++family) {
            const unsigned seed=2026111501u+family;
            const auto layout=generate(seed,family);
            Json controls=Json::array();for(const auto &p:layout.spline.controls)controls.push_back(vec(p));
            const auto start=layout.spline.sample(0.);
            Json preview=Json::array();for(int k=0;k<=700;++k)preview.push_back(vec(layout.spline.sample(k/700.)));
            save(out/"layouts"/("seed"+std::to_string(seed)+".json"),
                Json{{"seed",seed},{"controls",controls},{"reference_curve",preview},
                     {"spline_length_m",layout.spline_length},{"accepted_attempt",layout.attempts}});
            for(int tier=0;tier<3;++tier) {
                const std::string level="s"+std::to_string(family+1)+"_"+tiers[tier];
                levels.push_back(level);
                Json windows=Json::array();
                for(int k=0;k<7;++k) {
                    const Vector direction=layout.spline.tangent(layout.parameters[k]);
                    const double yaw=std::atan2(direction.y(),direction.x());
                    const auto rotation=(Eigen::AngleAxisd(yaw,Vector::UnitZ())*
                                         Eigen::AngleAxisd(-pi/2.,Vector::UnitY())).toRotationMatrix();
                    Json w{{"name","Gate"+std::to_string(k+1)},
                        {"center",vec(layout.centers[k])},{"rotation",mat(rotation)},
                        {"rpy_radians",{0.,-pi/2.,yaw}},{"circle",layout.circles[k]},
                        {"navigation_waypoint",false},
                        {"opening_radius",layout.circles[k]?Json(1.2):Json(nullptr)},
                        {"vertices",layout.circles[k]?Json::array():Json{{-1.2,-1.2},{1.2,-1.2},{1.2,1.2},{-1.2,1.2}}},
                        {"swing",layout.swings[k]},{"frequency",frequencies[tier]*layout.frequency_factor[k]},
                        {"amplitude",amplitudes[tier]*layout.amplitude_factor[k]},
                        {"length",1.6},{"phase",layout.phases[k]},{"sign",1.}};
                    w["mesh"]=frame_mesh(load_window(w));windows.push_back(w);
                }
                Json scene{{"level",level},{"scenario_variant","random_square_circle_closed7_v1"},
                    {"start",vec(start)},{"goal",vec(start)},{"windows",windows},{"vehicle",vehicle},
                    {"collision_radius_m",.15},{"physical_collision_radius_m",.15},
                    {"planning_safety_sphere_radius_m",.30},{"stationary_terminal_contact_is_crossing",false},
                    {"random_seed",seed},{"difficulty",tiers[tier]},
                    {"reference_spline_length_m",layout.spline_length}};
                save(out/"assets"/level/"scene.json",scene);
                double minimum_spacing=INFINITY,total_distance=0.;Vector previous=start;
                for(int i=0;i<7;++i) {
                    total_distance+=(layout.centers[i]-previous).norm();previous=layout.centers[i];
                    for(int j=0;j<i;++j)minimum_spacing=std::min(minimum_spacing,(layout.centers[i]-layout.centers[j]).norm());
                }
                total_distance+=(start-previous).norm();
                cases.push_back(Json{{"level",level},{"seed",seed},{"difficulty",tiers[tier]},
                    {"base_amplitude_m",amplitudes[tier]},{"base_frequency_hz",frequencies[tier]},
                    {"center_route_length_m",total_distance},{"minimum_gate_spacing_m",minimum_spacing},
                    {"spline_length_m",layout.spline_length},{"geometric_sampling_attempt",layout.attempts}});
                for(const std::string method:{"ours","center"}) {
                    const auto source=old/"inputs"/("static_"+method);
                    auto m=read(source/"manifest.json");
                    native_minco::remove_domain_bounds(m);
                    const auto original_seeds=m["initials"];
                    const int n=m["n"],count=m["m"],vs=m["velocity_start"];
                    if(n!=8 || count!=7)throw std::runtime_error("Expected seven physical gates");
                    m["windows"]=windows;for(auto &w:m["windows"])w.erase("mesh");
                    m["level"]=level;m["scenario_variant"]=scene["scenario_variant"];
                    m["vehicle"]=vehicle;m["finite_radius"]=.3851;m["verification_radius"]=.385;
                    m["config"]["radius"]=.30;m["config"]["witness_slack"]=.0001;
                    m["point_erosion_radius_m"]=.3001;m["finite_safety_sphere_radius_m"]=.3001;
                    m["verification_safety_sphere_radius_m"]=.30;m["body_sphere_radius_m"]=.15;
                    m["point_scale_m"]=Json::array();m["point_lower_m"]=Json::array();m["point_upper_m"]=Json::array();
                    for(int k=0;k<count;++k) {
                        const auto domain=planar_points::domain(load_window(windows[k]),.3001);
                        m["point_scale_m"].push_back(std::vector<double>(domain.scale.data(),domain.scale.data()+2));
                        m["point_lower_m"].push_back(std::vector<double>(domain.lower.data(),domain.lower.data()+2));
                        m["point_upper_m"].push_back(std::vector<double>(domain.upper.data(),domain.upper.data()+2));
                        if(m["free_points"].get<bool>())m["slices"][k]["circle"]=windows[k]["circle"];
                    }
                    std::vector<double> typical(n,0.),distance(n);
                    for(const auto &seedrow:original_seeds)for(int k=0;k<n;++k)
                        typical[k]+=seedrow["original"][k].get<double>()/original_seeds.size();
                    previous=start;
                    for(int k=0;k<n;++k) {
                        const Vector p=k<count?layout.centers[k]:start;
                        distance[k]=(p-previous).norm()/vehicle["initial_speed_guess_mps"].get<double>();previous=p;
                    }
                    drolib::PVAJ head=drolib::PVAJ::Zero();head.col(0)=start;
                    drolib::MincoSnap minco;minco.setConditions(head,head,n);
                    for(auto &seedrow:m["initials"]) {
                        auto x=seedrow["original"].get<std::vector<double>>();
                        std::vector<double> parameters(1+2*n,0.);
                        Eigen::VectorXd times(n);Eigen::Matrix3Xd points(3,count);double at=0.;
                        for(int k=0;k<n;++k) {
                            times[k]=distance[k]*std::exp(x[k]-typical[k]);x[k]=std::log(times[k]);at+=times[k];
                            parameters[1+k]=at;parameters[1+n+k]=1./std::max(1.,2*pi*windows[std::min(k,6)]["frequency"].get<double>());
                            if(k<count){const auto w=load_window(windows[k]);points.col(k)=w.center+w.rotation*w.motion(at);}
                        }
                        parameters[0]=at;
                        for(int j=n;j<vs;++j)x[j]=0.;
                        minco.setParameters(points,times);const auto &co=minco.getCoeffs();
                        for(int d=1;d<=3;++d)for(int k=0;k<count;++k)for(int axis=0;axis<3;++axis) {
                            double value=0.;for(int power=7;power>=d;--power) {
                                double factor=1.;for(int j=0;j<d;++j)factor*=power-j;
                                value=value*times[k]+factor*co(8*k+power,axis);
                            }
                            x[vs+3*((d-1)*count+k)+axis]=value;
                        }
                        seedrow["original"]=x;seedrow["parameters"]=parameters;
                        seedrow["finite"]=std::vector<double>(x.size()+m["slack_scale"].size(),0.);
                    }
                    planar_points::validate(m);
                    const auto dyn=read(source/"exchange_dynamics.json");
                    m["dynamic_residual_scale"]=dyn["residual_scale"];
                    std::vector<Window> physical_windows;for(const auto &w:windows)physical_windows.push_back(load_window(w));
                    auto node=restored_kernels::dynamic_node(dyn);
                    native_minco::Graph graph(m,scene,physical_windows,node,1000.);
                    if(graph.expressions[6].numel()!=m["lower_g"].size())throw std::runtime_error("Finite row count mismatch");
                    const auto dest=out/"inputs"/(level+"_"+method);
                    save(dest/"manifest.json",m);save(dest/"exchange_dynamics.json",dyn);
                    node.save((dest/"exchange_dynamic_node.casadi").string());
                    preflight.push_back(Json{{"level",level},{"method",method},{"nvar",m["nvar"]},
                        {"initials",m["initials"].size()},{"finite_rows",m["lower_g"].size()},{"validated",true}});
                }
                std::cout<<"GENERATED "<<level<<std::endl;
            }
        }
        save(out/"PROTOCOL.json",Json{{"levels",levels},{"cases",cases},
            {"seeds",{2026111501u,2026111502u,2026111503u}},
            {"shapes","4 squares and 3 circles, randomized order"},{"motion","4 translations and 3 swings independently shuffled"},
            {"planning_safety_sphere_radius_m",.30},{"physical_collision_radius_m",.15},
            {"start_equals_goal_zero_vaj",true},{"race_time_includes_return",true},
            {"source_reference","https://arxiv.org/html/2606.27348v1#A2"},
            {"layout_rule","periodic cubic B-spline from 10 bounded polar control points; 7 equal arc-length gates; horizontal tangent planes"},
            {"resampling_rule","only swept-extent spacing before any solver is run"},
            {"all_planning_code","C++"}});
        save(out/"PREFLIGHT.json",Json{{"passed",true},{"checks",preflight},
            {"input_generation_wall_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()}});
    } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
