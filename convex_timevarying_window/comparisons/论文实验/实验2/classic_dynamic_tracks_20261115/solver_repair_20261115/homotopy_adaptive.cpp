#include "trajectory.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <chrono>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>
namespace fs=std::filesystem;using Clock=std::chrono::steady_clock;
Json read(const fs::path&p){Json d;std::ifstream f(p);if(!f)throw std::runtime_error("Missing "+p.string());f>>d;return d;}
int run(std::vector<std::string>args,const fs::path&log){pid_t pid=fork();if(pid<0)throw std::runtime_error("fork failed");if(pid==0){int fd=open(log.c_str(),O_WRONLY|O_CREAT|O_EXCL,0644);if(fd<0)_exit(125);dup2(fd,1);dup2(fd,2);close(fd);setenv("OPENBLAS_NUM_THREADS","1",1);setenv("OMP_NUM_THREADS","1",1);setenv("MKL_NUM_THREADS","1",1);setenv("SIP_DUMP_INPUT","1",1);std::vector<char*>raw;for(auto&s:args)raw.push_back(s.data());raw.push_back(nullptr);execv(raw[0],raw.data());_exit(126);}int status;while(waitpid(pid,&status,0)<0 && errno==EINTR){}return WIFEXITED(status)?WEXITSTATUS(status):128+WTERMSIG(status);}
int main(int argc,char**argv){try {
 if(argc!=9)throw std::runtime_error("Usage: SOLVER PREP BASE SCENE PREVIOUS OUTPUT START_FRACTION_PER1000 STEP_PER1000");
 const fs::path out=fs::absolute(argv[6]);if(fs::exists(out))throw std::runtime_error("Output exists");fs::create_directories(out);
 fs::path previous=fs::absolute(argv[5]);int current=std::stoi(argv[7]),step=std::stoi(argv[8]);
 if(current<0 || current>=1000 || step<1 || step>100)throw std::runtime_error("Invalid continuation fraction");
 Json attempts=Json::array();const auto began=Clock::now();bool final_found=false;
 for(int attempt=0;attempt<64 && current<1000;++attempt) {
  const int next=std::min(1000,current+step);const auto stage=out/("attempt_"+std::to_string(attempt)+"_t"+std::to_string(next));fs::create_directory(stage);
  const auto started=Clock::now();const fs::path source=previous;
  const int prepared=run({argv[2],argv[3],argv[4],source.string(),std::to_string(next/1000.),(stage/"input").string(),"--multipliers"},stage/"prepare.log");
  if(prepared)throw std::runtime_error("Preparation failed");
  const int code=run({argv[1],(stage/"input").string(),(stage/"solve").string(),"0","--soft-mincopv",(stage/"input/scene.json").string(),"1000"},stage/"solve.log");
  Json item{{"attempt",attempt},{"source_successful_fraction",current/1000.},{"source",source.string()},{"motion_fraction",next/1000.},{"step_per1000",step},{"exit_code",code},{"stage_wall_s",std::chrono::duration<double>(Clock::now()-started).count()}};
  const auto file=stage/"solve/result.json";bool accepted=false;
  if(fs::exists(file)){const auto d=read(file);accepted=code==0 && d.value("found",false);item["found"]=accepted;item["nominal_time_s"]=d.value("nominal_time_s",Json(nullptr));item["calls"]=Json::array();for(const auto& c:d.at("optimizer_calls"))item["calls"].push_back({{"status",c["status"]},{"iterations",c["iterations"]},{"solver_s",c["solver_s"]},{"finite_violation",c["constraint_violation_inf"]}});}else item["found"]=false;
  attempts.push_back(item);
  if(accepted){current=next;previous=file;step=std::min(100,step*2);final_found=current==1000;}
  else step/=2;
  std::ofstream(out/"STAGES.json")<<Json{{"attempts",attempts},{"current_successful_fraction",current/1000.},{"last_successful_result",previous.string()},{"target_found",final_found},{"wall_s",std::chrono::duration<double>(Clock::now()-began).count()}}.dump(2)<<'\n';
  std::cout<<item.dump()<<std::endl;
  if(step<1)return 2;
 }
 return final_found?0:2;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
