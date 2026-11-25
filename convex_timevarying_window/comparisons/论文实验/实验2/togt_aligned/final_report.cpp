#include "json.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <map>
#include <set>
#include <cmath>
#include <numeric>
namespace fs=std::filesystem;using Json=nlohmann::json;
Json read(const fs::path&p){Json j;std::ifstream f(p);if(!f)throw std::runtime_error("Missing "+p.string());f>>j;return j;}
void save(const fs::path&p,const Json&j){std::ofstream(p)<<j.dump(2)<<'\n';}
int main(int argc,char**argv){try{
 if(argc!=2)throw std::runtime_error("Usage: aligned_final_report COMPLETED_RESULT_DIRECTORY");const fs::path out=argv[1];const auto status=read(out/"status.json");if(status["state"]!="complete")throw std::runtime_error("Complete the batch first; do not finalize a running batch");auto rows=read(out/"SUMMARY.json");if(rows.size()!=40)throw std::runtime_error("Expected ten conditions by four methods");Json corrections=Json::array(),audit=Json::array();
 for(auto&row:rows){const fs::path file=row["source"];const auto raw=read(file);const std::string method=row["method"];std::string result="未找到合格规划";Json assessment=nullptr;
  if(raw.value("found",false)){assessment=read(file.parent_path()/"assessment.json");result=assessment["collision_detected"].get<bool>()?"碰撞":!assessment["ordered_exactly_once"].get<bool>()||!assessment["endpoints_passed"].get<bool>()?"未完成":assessment["passed"].get<bool>()?"通过":"证据不足";double total=std::accumulate(raw["durations"].begin(),raw["durations"].end(),0.,[](double s,const Json&v){return s+v.get<double>();});if(std::abs(total-raw["nominal_time_s"].get<double>())>1e-8)throw std::runtime_error("Duration mismatch "+file.string());row["trajectory_time_s"]=total;}
  if(row["outcome"]!=result)corrections.push_back(Json{{"level",row["level"]},{"method",method},{"from",row["outcome"]},{"to",result},{"reason",assessment.is_null()?"optimizer did not produce a selected trajectory":assessment["verdict"]}});row["outcome"]=result;row["assessment_verdict"]=assessment.is_null()?Json(nullptr):assessment["verdict"];
  if(!assessment.is_null()){double low=INFINITY;bool contact=false;for(const auto&g:assessment["ground"]){low=std::min(low,g["minimum_clearance_m"].get<double>());contact=contact||g["outcome"]=="collision";}row["ground_contact"]=contact;row["minimum_ground_clearance_m"]=low;}else{row["ground_contact"]=nullptr;row["minimum_ground_clearance_m"]=nullptr;}

  if(method=="ours"||method=="center"){
   if(raw["candidate_records"].size()!=8)throw std::runtime_error("Expected eight independent candidates per method");
   for(const auto&candidate:raw["candidate_records"]){const auto c=read(candidate["source"].get<std::string>());std::map<int,Json>calls;int previous_revision=-1;for(const auto&call:c["optimizer_calls"]){const int revision=call["constraint_revision"].get<int>();if(revision<=previous_revision)throw std::runtime_error("Repeated unchanged finite problem");previous_revision=revision;calls[call["round"].get<int>()]=call;if(call.value("warm_start_used",false)&&(call["previous_solve_status"]!="Solve_Succeeded"||call["dual_source_status"]!="Solve_Succeeded"))throw std::runtime_error("Invalid multiplier warm start");}
    for(const auto&record:c["records"]){const auto&call=calls.at(record["round"].get<int>());if((call["status"]!="Solve_Succeeded"&&call["status"]!="Solved_To_Acceptable_Level")||call["constraint_violation_inf"].get<double>()>1e-5||call.value("variable_bound_violation_inf",0.)>1e-5)throw std::runtime_error("Exchange before finite readiness");std::set<int>gates;for(const auto&w:record["geometry_added_witnesses"])if(!gates.insert(w["gate"].get<int>()).second)throw std::runtime_error("Too many witnesses per gate");}
    Json failed_checks=Json::array();if(!c["records"].empty())for(const auto&check:c["records"].back()["checks"])if(!check.value("passed",false))failed_checks.push_back(check);
    Json last_status=nullptr;if(!c["optimizer_calls"].empty())last_status=c["optimizer_calls"].back()["status"];
    audit.push_back(Json{{"level",row["level"]},{"method",method},{"job",candidate["job"]},{"found",c["found"]},{"termination_reason",c["termination_reason"]},{"last_finite_status",last_status},{"last_failed_checks",failed_checks},{"finite_calls",c["optimizer_calls"].size()},{"checked_rounds",c["records"].size()}});
   }
  }
 }
 if(audit.size()!=160)throw std::runtime_error("Expected 160 independent candidates");
 save(out/"FINAL_SUMMARY.json",rows);save(out/"REPORT_CLASSIFICATION_CORRECTIONS.json",corrections);save(out/"FINAL_CANDIDATE_AUDIT.json",Json{{"passed",true},{"candidates",audit.size()},{"rows",audit}});
 const std::vector<std::string>levels{"static","mixed_base","a25_f3","a25_f6","a50_f1","a50_f3","a50_f6","a75_f1","a75_f3","a75_f6"},methods{"ours","center","dynamic_togt","dynamic_togt_center"};std::ofstream f(out/"FINAL_PLANNING.md");f<<"# 四方法十档完整规划结果\n\n原TOGT公开七门位置/朝向、QuadA；4个2.4m方门与3个直径2.4m圆门，已知平移/摆动。每侧名义安全内缩0.15m，有限求解数值间隙0.1mm，圆窗解析模型、独立网格512段。只做规划，未执行Gazebo竞速。完整参考从作者起点出发，包含七门及到作者原终点的末段。表中通过指窗框、门序与端点；按用户约定地面相交单列，不算窗框碰撞。\n\n本文自由/中心各8个独立初值、16路并行，无额外中心精修。动态TOGT采用现有C++动态适配、原停止设置、单初值中心后自由流程；不将本变体成绩称为作者原七方门论文成绩。失败与负结果保留。\n\n|幅度比|频率倍率|本文自由点/s|本文中心/s|动态TOGT自由点/s|动态TOGT中心/s|\n|---|---|---:|---:|---:|---:|\n";
 std::map<std::string,int>passed;
 for(const auto&level:levels){double best=INFINITY;std::map<std::string,Json>group;for(const auto&row:rows)if(row["level"]==level){group[row["method"].get<std::string>()]=row;if(row["outcome"]=="通过"){best=std::min(best,row["trajectory_time_s"].get<double>());++passed[row["method"].get<std::string>()];}}f<<'|'<<(level=="static"?"0":level=="mixed_base"||level.substr(0,3)=="a25"?"0.25":level.substr(0,3)=="a50"?"0.50":"0.75")<<'|'<<(level=="static"||level=="mixed_base"?"1":level.substr(5))<<'|';for(const auto&m:methods){const auto&row=group.at(m);if(row["outcome"]!="通过")f<<row["outcome"].get<std::string>();else{const double t=row["trajectory_time_s"];const bool bold=std::abs(t-best)<1e-9;f<<(bold?"**":"")<<std::fixed<<std::setprecision(3)<<t<<(bold?"**":"");}f<<'|';}f<<'\n';}
 f<<"\n## 赛道适配对对照结果的影响\n\n作者公开混合七门文件的第7门原是SingleBall，未定义门平面；本轮按方门文件取第7门yaw=200°并设为实体方框，终点沿用作者原位置（与第6门中心重合），所以第7门是折返段。静态动态TOGT在通过第7门后、转向终点时碰框；部分本文候选在折返时重复穿过该开口，被一门仅穿一次的任务检查拒收。因此这些失败不能全部归因于高速窗口运动，也不等同于作者原场景成绩。\n\n通过仅指名义曲线的窗框、门序与端点检查，动力学沿用软惩罚；不是实飞结果或严格区间证书。\n";
 f<<"\n## 计算时间\n\n以下为实际求解、建图与独立几何检查墙钟；本文计算八初值并行的等待时间，不累计worker。TOGT自由点包含其中心初始化成本。前两档包括圆网格收敛重查开销，轨迹未重算。共同输入生成没有单独计时，未填零，不作为完整初始化成本已测量的结论。\n\n|条件|方法|计算墙钟/s|合格初值|\n|---|---|---:|---|\n";for(const auto&row:rows)f<<'|'<<row["level"].get<std::string>()<<'|'<<row["method"].get<std::string>()<<'|'<<std::fixed<<std::setprecision(3)<<row["compute_wall_s"].get<double>()<<'|'<<row["qualified_starts"].get<std::string>()<<"|\n";
 f<<"\n## 未找到合格规划的初值停止原因\n\n|条件|方法|停止原因|初值数|\n|---|---|---|---:|\n";for(const auto&level:levels)for(const auto&method:std::vector<std::string>{"ours","center"}){std::map<std::string,int>counts;for(const auto&candidate:audit)if(candidate["level"]==level&&candidate["method"]==method&&!candidate["found"].get<bool>()){std::string reason=candidate["termination_reason"];std::set<std::string>checks;for(const auto&check:candidate["last_failed_checks"])checks.insert(check["outcome"].get<std::string>());for(const auto&check:checks)reason+=" / "+check;++counts[reason];}for(const auto&entry:counts)f<<'|'<<level<<'|'<<method<<'|'<<entry.first<<'|'<<entry.second<<"|\n";}
 f<<"\nno_new_witness表示当前候选没有可新增的距离约束见证点；若后面同时标注repeat_aperture_crossing，具体障碍是重复过窗任务检查，不能把它写成IPOPT未收敛或物理无解。逐初值的有限求解状态和失败检查见FINAL_CANDIDATE_AUDIT.json。\n";
 f<<"\n名义地面相交记录（不并入窗框碰撞）：";bool any_ground=false;for(const auto&row:rows)if(row["ground_contact"].is_boolean()&&row["ground_contact"].get<bool>()){any_ground=true;f<<row["level"].get<std::string>()<<'/'<<row["method"].get<std::string>()<<"；";}if(!any_ground)f<<"无";f<<'\n';
 f<<"\n通过条件数：";for(const auto&m:methods)f<<m<<' '<<passed[m]<<"/10；";f<<"\n\n碰撞为名义曲线与实体窗框的接触；未完成为明确的穿窗顺序/次数或端点任务不合格；证据不足仅用于尚未判定的检查。未找到不等于物理不可行。地面接触另记在assessment中，不并入窗框碰撞。原始SUMMARY/PLANNING保留，分类纠正见REPORT_CLASSIFICATION_CORRECTIONS.json。最终数据FINAL_SUMMARY.json、逐初值审计FINAL_CANDIDATE_AUDIT.json。\n";
 std::cout<<"FINAL_REPORT "<<rows.size()<<" rows; "<<audit.size()<<" independent candidates audited\n";
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}return 0;}
