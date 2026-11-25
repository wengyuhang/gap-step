#include "mesh.hpp"
#include <filesystem>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <thread>
#include <map>
#include <sstream>
#include <sys/file.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
Json read(const fs::path&p){Json j;std::ifstream f(p);if(!f)throw std::runtime_error("Cannot read "+p.string());f>>j;return j;}
void save(const fs::path&p,const Json&j){fs::create_directories(p.parent_path());const fs::path tmp=p.string()+".writing";std::ofstream(tmp)<<j.dump(2)<<'\n';fs::rename(tmp,p);}
double execute(const std::vector<std::string>&args,const fs::path&log){fs::create_directories(log.parent_path());const auto start=Clock::now();pid_t pid=fork();if(pid<0)throw std::runtime_error("fork failed");if(pid==0){int out=open(log.c_str(),O_CREAT|O_WRONLY|O_TRUNC,0644);dup2(out,1);dup2(out,2);close(out);std::vector<char*>raw;for(const auto&v:args)raw.push_back(const_cast<char*>(v.c_str()));raw.push_back(nullptr);execv(raw[0],raw.data());_exit(127);}int status;waitpid(pid,&status,0);if(!WIFEXITED(status)||WEXITSTATUS(status)!=0)throw std::runtime_error("Process failed; see "+log.string());return std::chrono::duration<double>(Clock::now()-start).count();}
std::string cell(const Json&j,bool best){if(j["outcome"]!="通过")return j["outcome"].get<std::string>();std::ostringstream o;if(best)o<<"**";o<<std::fixed<<std::setprecision(3)<<j["trajectory_time_s"].get<double>();if(best)o<<"**";return o.str();}
void report(const fs::path&out,const Json&rows,const std::vector<std::string>&levels){
 const std::vector<std::string>methods{"ours","center","dynamic_togt","dynamic_togt_center"};std::ofstream f(out/"PLANNING.md");f<<"# TOGT公开七门布局、QuadA与三圆动态适配：规划\n\n"
 <<"全部为新场景的名义规划与独立网格/任务复查，无Gazebo竞速执行。完整时间包含起点、七门和作者原终点。碰撞/未找到直接标注；最短仅在通过者之间加粗。\n\n"
 <<"位置/朝向来自作者方门YAML前七门，四方门2.4m、三圆直径2.4m；每侧名义内缩0.15m，有限问题另有0.1mm数值间隙。本文两方各八独立初值，不额外中心释放；动态TOGT两方用原C++求解器和原停止条件，单初值中心后释放自由点。\n\n"
 <<"|幅度比|频率倍率|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|\n|---|---|---:|---:|---:|---:|\n";
 for(const auto&level:levels){std::map<std::string,Json>r;for(const auto&j:rows)if(j["level"]==level)r[j["method"]]=j;if(r.empty())continue;double best=INFINITY;for(const auto&[m,j]:r)if(j["outcome"]=="通过")best=std::min(best,j["trajectory_time_s"].get<double>());const std::string a=level=="static"?"0":level=="mixed_base"?"0.25":level.substr(0,3)=="a25"?"0.25":level.substr(0,3)=="a50"?"0.50":"0.75",freq=level=="static"||level=="mixed_base"?"1":level.substr(5);f<<'|'<<a<<'|'<<freq<<'|';for(const auto&m:methods)f<<(r.count(m)?cell(r.at(m),r.at(m)["outcome"]=="通过"&&std::abs(r.at(m)["trajectory_time_s"].get<double>()-best)<1e-9):"待完成")<<'|';f<<'\n';}
 f<<"\n## 计算墙钟/s\n\n计时包含本方法求解与独立几何检查；本文为八初值并行结束的墙钟，不累加worker。TOGT自由点另计其依赖的中心初始化。输入文件生成未单独计时，不虚填为零。\n\n|条件|方法|计算墙钟/s|合格独立初值|\n|---|---|---:|---|\n";
 for(const auto&j:rows)f<<'|'<<j["level"].get<std::string>()<<'|'<<j["method"].get<std::string>()<<'|'<<std::fixed<<std::setprecision(3)<<j["compute_wall_s"].get<double>()<<'|'<<j["qualified_starts"].get<std::string>()<<"|\n";
}
int main(int argc,char**argv){try{
 if(argc!=5 && argc!=6)throw std::runtime_error("Usage: aligned_run BIN INPUTS AUTHOR_PARAMS OUTPUT [SINGLE_LEVEL]");const fs::path bin=fs::absolute(argv[1]),inputs=fs::absolute(argv[2]),params=fs::absolute(argv[3]),out=fs::absolute(argv[4]);fs::create_directories(out);int lock=open((out/"full_coordinator.lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC,0644);if(flock(lock,LOCK_EX|LOCK_NB))throw std::runtime_error("Full coordinator already running");
 const std::vector<std::string>levels=argc==6?std::vector<std::string>{argv[5]}:std::vector<std::string>{"static","mixed_base","a25_f3","a25_f6","a50_f1","a50_f3","a50_f6","a75_f1","a75_f3","a75_f6"};Json rows=Json::array();const auto began=Clock::now();
 for(const auto&level:levels){const fs::path group=out/level;fs::create_directories(group);save(out/"status.json",Json{{"state","planning"},{"level",level},{"pid",getpid()},{"finished_rows",rows.size()},{"gazebo",false}});
  // A previously launched condition retains its one matrix coordinator; never duplicate it.
  if(fs::exists(group/"status.json")){while(true){auto state=read(group/"status.json");if(state.value("state",std::string())=="planning_complete")break;int pid=state.value("coordinator_pid",-1);if(pid>0&&kill(pid,0)==0){std::this_thread::sleep_for(std::chrono::seconds(1));continue;}throw std::runtime_error("Existing condition stopped before completion: "+level);}}
  else execute({(bin/"matrix").string(),bin.string(),(inputs/"inputs").string(),(inputs/"assets").string(),group.string(),"16",level,"--main-xy-eight"},group/"matrix.log");
  const auto scene=read(inputs/"assets"/level/"scene.json");
  for(const auto&method:{"ours","center"}){const auto file=group/"plans"/(level+"_"+method)/"result.json";const auto plan=read(file);bool pass=false,collision=false;int qualified=0,finished=0;for(const auto&c:plan["candidate_records"]){qualified+=c["passed"].get<bool>();++finished;}if(plan["found"].get<bool>()){auto a=read(file.parent_path()/"assessment.json");pass=a["passed"];collision=a["collision_detected"];}
   rows.push_back(Json{{"level",level},{"method",method},{"outcome",!plan["found"].get<bool>()?"未找到合格规划":collision?"碰撞":pass?"通过":"证据不足"},{"trajectory_time_s",plan.value("nominal_time_s",Json(nullptr))},{"compute_wall_s",plan["planning_wall_s"]},{"qualified_starts",std::to_string(qualified)+"/"+std::to_string(finished)},{"source",file.string()}});
  }
  double center_wall=0.;for(const auto&method:{"dynamic_togt_center","dynamic_togt"}){const fs::path folder=group/"baselines"/method;fs::create_directories(folder);const auto file=folder/"result.json",timing=folder/"TIMING.json";if(!fs::exists(timing)){std::vector<std::string>args{(bin/"dynamic_togt_planner").string(),(inputs/"assets"/level/"scene.json").string(),folder.string(),std::string(method)=="dynamic_togt_center"?"center":"free",params.string(),(inputs/"vehicle.json").string()};if(std::string(method)=="dynamic_togt")args.push_back("--center-result="+(group/"baselines/dynamic_togt_center/result.json").string());double wall=execute(args,folder/"process.log");const auto start=Clock::now();auto plan=read(file);if(plan.value("found",false))save(folder/"assessment.json",mesh_evaluate(Trajectory::load(plan),scene));wall+=std::chrono::duration<double>(Clock::now()-start).count();save(timing,Json{{"stage_wall_s",wall},{"compute_wall_s",wall+(std::string(method)=="dynamic_togt"?center_wall:0.)}});}
   const auto t=read(timing),plan=read(file);if(std::string(method)=="dynamic_togt_center")center_wall=t["compute_wall_s"];bool pass=false,collision=false;if(plan.value("found",false)){auto a=read(folder/"assessment.json");pass=a["passed"];collision=a["collision_detected"];}
   rows.push_back(Json{{"level",level},{"method",method},{"outcome",!plan.value("found",false)?"未找到规划":collision?"碰撞":pass?"通过":"证据不足"},{"trajectory_time_s",plan.value("nominal_time_s",Json(nullptr))},{"compute_wall_s",t["compute_wall_s"]},{"qualified_starts",pass?"1/1":"0/1"},{"source",file.string()}});
  }
  save(out/"SUMMARY.json",rows);report(out,rows,levels);std::cout<<"CONDITION_COMPLETE "<<level<<std::endl;
 }
 save(out/"status.json",Json{{"state","complete"},{"pid",getpid()},{"rows",rows.size()},{"conditions",levels.size()},{"elapsed_s",std::chrono::duration<double>(Clock::now()-began).count()},{"gazebo",false}});
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';if(argc>=5)save(fs::absolute(argv[4])/"status.json",Json{{"state","error"},{"message",e.what()},{"pid",getpid()}});return 1;}return 0;}
