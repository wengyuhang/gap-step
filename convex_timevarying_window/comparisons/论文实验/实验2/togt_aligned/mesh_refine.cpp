#include "mesh.hpp"
#include <filesystem>
#include <map>
namespace fs=std::filesystem;
Json read(const fs::path&p){Json j;std::ifstream(p)>>j;return j;}
void save(const fs::path&p,const Json&j){fs::create_directories(p.parent_path());std::ofstream(p)<<j.dump(2)<<'\n';}
Json vec(const Vector&v){return {v.x(),v.y(),v.z()};}
Json circular_mesh(double r,int n){Json out=Json::array();constexpr double tube=.085;auto at=[&](int k,int s)->Vector{double a=2*pi*k/n,b=2*pi*s/12;return {(r+tube+tube*cos(b))*cos(a),(r+tube+tube*cos(b))*sin(a),tube*sin(b)};};for(int k=0;k<n;++k)for(int s=0;s<12;++s){const auto a=at(k,s),b=at(k+1,s),c=at(k+1,s+1),d=at(k,s+1);out.push_back({vec(a),vec(b),vec(c)});out.push_back({vec(a),vec(c),vec(d)});}return out;}
int main(int argc,char**argv){try{
 if(argc==5&&std::string(argv[1])=="study"){
 const auto trajectory=Trajectory::load(read(argv[2]));const auto original=read(argv[3]);Json rows=Json::array();for(int n:{128,256,512,1024}){auto scene=original;for(auto&w:scene["windows"])if(w["circle"].get<bool>())w["mesh"]=circular_mesh(w["opening_radius"],n);auto check=mesh_evaluate(trajectory,scene);save(fs::path(argv[4])/("resolution_"+std::to_string(n)+".json"),check);rows.push_back(Json{{"ring_segments",n},{"maximum_inner_radius_contraction_m",1.2*(1.-cos(pi/n))},{"passed",check["passed"]},{"collision",check["collision_detected"]},{"check_s",check["check_s"]}});}save(fs::path(argv[4])/"SUMMARY.json",rows);std::cout<<rows.dump()<<'\n';return 0;}
 if(argc!=5)throw std::runtime_error("Usage: mesh_refine recheck OLD_GROUP SCENE NEW_GROUP | study TRAJECTORY SCENE OUTPUT");
 const fs::path old=argv[2],out=argv[4];if(fs::exists(out))throw std::runtime_error("Use a new derived group");const auto scene=read(argv[3]);auto rows=read(old/"candidate_summary.json");auto timing=read(old/"PAIRED_WALL.json");std::map<std::string,double>extra;const auto began=std::chrono::steady_clock::now();
 for(auto&row:rows){if(!row["found"].get<bool>())continue;const auto start=std::chrono::steady_clock::now();auto raw=read(row["source"].get<std::string>());const auto assessment=mesh_evaluate(Trajectory::load(raw),scene);const fs::path file=out/"assessments"/(row["job"].get<std::string>()+".json");save(file,assessment);row["passed"]=assessment["passed"];row["geometry_verdict"]=assessment["verdict"];row["assessment_source"]=file.string();extra[row["method"]]+=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();}
 save(out/"candidate_summary.json",rows);
 for(const auto&method:{"ours","center"}){Json best=nullptr,own=Json::array();for(const auto&row:rows)if(row["method"]==method){own.push_back(row);if(row["passed"].get<bool>()&&(best.is_null()||row["objective"].get<double>()<best["objective"].get<double>()))best=row;}
  Json selected=best.is_null()?Json{{"found",false}}:read(best["source"].get<std::string>());const std::string level=own[0]["level"];selected["candidate_records"]=own;selected["method"]=method;selected["scenario_variant"]=scene["scenario_variant"];selected["case"]={{"layout","togt_whole"},{"level",level}};selected["selected_candidate_job"]=best.is_null()?Json(nullptr):best["job"];
  const auto key=std::string(method)=="ours"?"free_wall_s":"center_wall_s";const double original_wall=timing[key];timing[key]=original_wall+extra[method];selected["planning_wall_s"]=timing[key];selected["solve_s"]=timing[key];selected["mesh_refinement_recheck_s"]=extra[method];selected["original_planning_group"]=fs::absolute(old).string();
  const fs::path folder=out/"plans"/(level+"_"+method);save(folder/"result.json",selected);if(!best.is_null())save(folder/"assessment.json",read(best["assessment_source"].get<std::string>()));}
 save(out/"PAIRED_WALL.json",timing);save(out/"RECHECK.json",Json{{"newton_or_ipopt_rerun",false},{"trajectory_coefficients_changed",false},{"radius_or_collision_threshold_changed",false},{"original_group",fs::absolute(old).string()},{"scene",fs::absolute(argv[3]).string()},{"recheck_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()}});
 save(out/"status.json",Json{{"state","planning_complete"},{"completed",rows.size()},{"coordinator_pid",0},{"derived_from",fs::absolute(old).string()},{"action","same raw optimized trajectories; refined circular mesh only"}});
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}return 0;}
