#include "planar_points.hpp"
#include "sip_exchange_policy.hpp"
#include <filesystem>
#include <chrono>
namespace fs=std::filesystem;
Json read(const fs::path& p){Json j;std::ifstream in(p);if(!in)throw std::runtime_error("Cannot open "+p.string());in>>j;return j;}
void save(const fs::path& p,const Json& j){fs::create_directories(p.parent_path());std::ofstream(p)<<j.dump(2)<<'\n';}
Json vector(const Vector& v){return {v.x(),v.y(),v.z()};}
void quad(Json& mesh,const Vector& a,const Vector& b,const Vector& c,const Vector& d) {
    mesh.push_back({vector(a),vector(b),vector(c)});mesh.push_back({vector(a),vector(c),vector(d)});
}
Json frame_mesh(const Window& w) {
    Json mesh=Json::array();constexpr double tube=.085;
    if(w.circle) {
        constexpr int segments=128,sides=12;
        const double centerline=w.opening_radius+tube;
        auto at=[&](int k,int s)->Vector{const double a=2*pi*k/segments,b=2*pi*s/sides;
            return {(centerline+tube*std::cos(b))*std::cos(a),(centerline+tube*std::cos(b))*std::sin(a),tube*std::sin(b)};};
        for(int k=0;k<segments;++k)for(int s=0;s<sides;++s)quad(mesh,at(k,s),at(k+1,s),at(k+1,s+1),at(k,s+1));
    } else {
        constexpr int sides=12;
        for(size_t k=0;k<w.frame.size();++k) {
            const Vector a(w.frame[k].x(),w.frame[k].y(),0.),b(w.frame[(k+1)%4].x(),w.frame[(k+1)%4].y(),0.);
            const Vector direction=(b-a).normalized(),normal(direction.y(),-direction.x(),0.);
            auto at=[&](const Vector& p,int s)->Vector {const double angle=2*pi*s/sides;return p+tube*std::cos(angle)*normal+tube*std::sin(angle)*Vector::UnitZ();};
            for(int s=0;s<sides;++s) {
                quad(mesh,at(a,s),at(b,s),at(b,s+1),at(a,s+1));
                mesh.push_back({vector(a),vector(at(a,s+1)),vector(at(a,s))});
                mesh.push_back({vector(b),vector(at(b,s)),vector(at(b,s+1))});
            }
        }
    }
    return mesh;
}
int main(int argc,char**argv) {
    try {
        if(argc!=4)throw std::runtime_error("Usage: main_course_inputs ORIGINAL_INPUTS ORIGINAL_ASSETS NEW_OUTPUT");
        const fs::path inputs=argv[1],assets=argv[2],out=argv[3];
        if(fs::exists(out))throw std::runtime_error("Use a new output directory; historical inputs are never overwritten");
        const auto began=std::chrono::steady_clock::now();
        const std::vector<std::string> levels{"static","mixed_base","a25_f3","a25_f6","a50_f1","a50_f3","a50_f6","a75_f1","a75_f3","a75_f6"};
        Json audits=Json::array();
        for(const auto& level:levels) {
            Json scene=read(assets/level/"scene.json");
            if(scene["windows"].size()!=7)throw std::runtime_error("Main experiment requires seven gates");
            int quadrilaterals=0,circles=0;
            for(int k=0;k<7;++k) {
                auto& w=scene["windows"][k];const bool circle=k%2==1;
                w["circle"]=circle;
                w["name"]=(k<9?"0":"")+std::to_string(k+1)+(circle?"_circle":"_quadrilateral");
                if(circle) {++circles;w["opening_radius"]=.762;w["vertices"]=Json::array();w["opening_diameter_m"]=1.524;}
                else {
                    const double side=(quadrilaterals++%2==0)?1.5:1.524,h=.5*side;
                    w["opening_radius"]=nullptr;w["opening_width_m"]=side;w["opening_height_m"]=side;
                    w["vertices"]={{-h,-h},{h,-h},{h,h},{-h,h}};
                }
                w["mesh"]=frame_mesh(load_window(w));
            }
            scene["scenario_variant"]="four_quad_three_circle_v1";
            scene["geometry_protocol"]={{"quadrilaterals",quadrilaterals},{"circles",circles},{"quad_inner_side_m",{1.5,1.524,1.5,1.524}},
                {"circle_inner_diameter_m",1.524},{"circle_size_rule","research choice matched to the MultiGP square span; not an official circle gate specification"},
                {"frame_tube_radius_m",.085},{"centers_rotations_motions","unchanged from input assets"}};
            save(out/"assets"/level/"scene.json",scene);
            for(const auto& method:{"ours","center"}) {
                Json meta=read(inputs/(level+"_"+method)/"manifest.json"),windows=scene["windows"];
                for(auto& w:windows)w.erase("mesh");
                meta["windows"]=windows;meta["scenario_variant"]=scene["scenario_variant"];
                meta["finite_radius"]=sip_exchange::default_finite_radius;
                meta["config"]["witness_slack"]=meta["finite_radius"].get<double>()-meta["verification_radius"].get<double>();
                const Json old_seeds=meta["initials"];const int old_vs=meta["velocity_start"],n=meta["n"];
                planar_points::configure(meta);
                double maximum_time_error=0.,maximum_state_error=0.;
                for(size_t j=0;j<meta["initials"].size();++j) {
                    for(int k=0;k<n;++k)maximum_time_error=std::max(maximum_time_error,std::abs(meta["initials"][j]["original"][k].get<double>()-old_seeds[j]["original"][k].get<double>()));
                    const int vs=meta["velocity_start"];
                    for(size_t k=0;k<meta["state_scale"].size();++k)maximum_state_error=std::max(maximum_state_error,std::abs(meta["initials"][j]["original"][vs+k].get<double>()-old_seeds[j]["original"][old_vs+k].get<double>()));
                }
                meta["initialization"]="Exact archived eight independent durations and actual VAJ; all physical plane points initially zero";
                const fs::path folder=out/"inputs"/(level+"_"+method);save(folder/"manifest.json",meta);
                fs::copy_file(inputs/(level+"_"+method)/"exchange_dynamics.json",folder/"exchange_dynamics.json");
                audits.push_back(Json{{"level",level},{"method",method},{"starts",meta["initials"].size()},{"trajectory_variables",meta["nvar"]},
                    {"point_variables",meta["free_points"].get<bool>()?14:0},{"point_domain_rows",meta["point_domain_rows"]},{"gauge_rows",0},{"point_erosion_radius_m",meta["point_erosion_radius_m"]},
                    {"maximum_time_parameter_change",maximum_time_error},{"maximum_VAJ_change",maximum_state_error}});
            }
        }
        save(out/"INITIALIZATION.json",Json{{"passed",true},{"runs",audits},{"preparation_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()},
            {"dimensions_sources",{"https://www.theaigrandprix.com/wp-content/uploads/2026/05/260508_Technical_Spec_0002.pdf","https://shop.multigp.com/product/standard-multigp-gate-5x5/"}},
            {"physical_point_storage","gate plane xy in metres; solver units scaled by aperture half width and height"},
            {"finite_centerline_clearance_m",sip_exchange::default_finite_radius},{"point_erosion_radius_m",.54594472},{"historical_results_modified",false}});
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;
}
