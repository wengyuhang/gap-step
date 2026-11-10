#include "trajectory.hpp"
#include <filesystem>
#include <iomanip>
#include <map>
#include <sstream>
namespace fs=std::filesystem;
Json read_json(const fs::path& path){Json value;std::ifstream(path)>>value;return value;}
const std::vector<std::string> report_methods{"ours","center","dynamic_togt","dynamic_togt_center","wang2023"};
const std::vector<std::string> report_levels{"static","mixed_base","a25_f3","a25_f6","a50_f1","a50_f3","a50_f6","a75_f1","a75_f3","a75_f6"};
const std::map<std::string,std::string> method_labels{{"ours","本文自由点"},{"center","本文中心"},
    {"dynamic_togt","动态TOGT自由点"},{"dynamic_togt_center","动态TOGT中心"},{"wang2023","Wang独立复现"}};
const std::map<std::string,std::string> level_labels{{"static","静态"},{"mixed_base","0.25／1×"},{"a25_f3","0.25／3×"},
    {"a25_f6","0.25／6×"},{"a50_f1","0.50／1×"},{"a50_f3","0.50／3×"},{"a50_f6","0.50／6×"},
    {"a75_f1","0.75／1×"},{"a75_f3","0.75／3×"},{"a75_f6","0.75／6×"}};

double trajectory_distance(const Trajectory& trajectory) {
    constexpr int order=128;Eigen::MatrixXd jacobi=Eigen::MatrixXd::Zero(order,order);
    for(int k=1;k<order;++k)jacobi(k,k-1)=jacobi(k-1,k)=k/std::sqrt(4.*k*k-1.);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(jacobi);double length=0.;
    for(size_t piece=0;piece<trajectory.durations.size();++piece)for(int k=0;k<order;++k) {
        const double u=(solver.eigenvalues()[k]+1.)/2.;Vector velocity=Vector::Zero();
        for(int power=7;power>=1;--power)velocity=u*velocity+power*trajectory.coefficients[piece].row(power).transpose();
        length+=solver.eigenvectors()(0,k)*solver.eigenvectors()(0,k)*velocity.norm();
    }
    return length;
}

void write_table(const fs::path& out,const Json& rows,bool flight) {
    const std::string prefix=flight?"FLIGHT":"PLANNING";std::ofstream markdown(out/(prefix+".md")),csv(out/(prefix+".csv"));
    markdown<<"# "<<(flight?"Gazebo实飞":"规划轨迹")<<"\n\n时间单位为秒，覆盖七窗与完整返航。每行无碰撞完成中的最短时间加粗。\n\n";
    markdown<<"|幅度比／频率倍数";csv<<"level";
    for(const auto& method:report_methods){markdown<<'|'<<method_labels.at(method);csv<<','<<method;}
    markdown<<"|\n|---|---:|---:|---:|---:|---:|\n";csv<<'\n';
    for(const auto& level:report_levels) {
        std::map<std::string,Json> cells;double best=INFINITY;
        for(const auto& row:rows)if(row["level"]==level) {
            const std::string method=row[flight?"method":"variant"];cells[method]=row;
            if(row["outcome"]==(flight?"无碰撞完赛":"无碰撞完成") && !row[flight?"flight_time_s":"full_trajectory_time_s"].is_null())
                best=std::min(best,row[flight?"flight_time_s":"full_trajectory_time_s"].get<double>());
        }
        markdown<<'|'<<level_labels.at(level);csv<<level;
        for(const auto& method:report_methods) {
            std::string text="未完成",display=text;
            if(cells.count(method)) {
                const auto& row=cells.at(method);const std::string outcome=row["outcome"];
                if(outcome==(flight?"无碰撞完赛":"无碰撞完成")) {
                    const double value=row[flight?"flight_time_s":"full_trajectory_time_s"];
                    std::ostringstream stream;stream<<std::fixed<<std::setprecision(3)<<value;text=stream.str();display=text;
                    if(std::abs(value-best)<1e-9)display="**"+text+"**";
                } else {text=outcome;display=outcome;}
            }
            markdown<<'|'<<display;csv<<','<<text;
        }
        markdown<<"|\n";csv<<'\n';
    }
    if(flight)markdown<<"\n实飞时间来自实际世界时钟和测量时间，有限开口穿越与实体窗框接触由原生记录独立核对。地面接触单列保存在原始审计。\n";
    else markdown<<"\n规划碰撞由连续局部包络和实体STL距离复查判定，原始候选及失败完整保留。\n";
}

void write_planning_review(const fs::path& out,const Json& planning) {
    write_table(out,planning,false);
    Json details=Json::array(),failures=Json::array();
    std::ofstream costs(out/"PLANNING_COST.csv"),candidate_csv(out/"PLANNING_CANDIDATES.csv");
    costs<<"level,method,outcome,trajectory_time_s,trajectory_distance_m,candidate_count,selected_job,selected_candidate_s,candidate_time_sum_s,shared_center_stage_sum_s,source_file\n";
    candidate_csv<<"level,method,job,found,assessment_passed,trajectory_time_s,objective,solve_s,process_wall_s,source_file\n";
    std::map<std::string,Json> by_key;
    for(const auto& row:planning)by_key[row["level"].get<std::string>()+"/"+row["variant"].get<std::string>()]=row;
    for(const auto& level:report_levels)for(const auto& method:report_methods) {
        const auto& row=by_key.at(level+"/"+method);const auto source=fs::path(row["source_file"].get<std::string>());
        const auto raw=read_json(source);Json detail=row;
        Json candidates=raw.value("candidate_records",Json::array());
        if(method=="wang2023")candidates=Json::array({Json{{"job",level+"_compiled_wang"},{"level",level},{"method",method},
            {"found",raw.at("found")},{"passed",row["outcome"]=="无碰撞完成"},{"nominal_time_s",row["full_trajectory_time_s"]},
            {"objective",raw.value("objective",Json(nullptr))},{"solve_s",raw.value("solve_s",0.)},{"source",source.string()}}});
        const Json selected=raw.value("selected_candidate_job",method=="wang2023"?Json(level+"_compiled_wang"):Json(nullptr));
        Json selected_s=nullptr;double total=0.;
        for(const auto& c:candidates) {
            total+=c.value("solve_s",0.);if(c["job"]==selected)selected_s=c.value("solve_s",Json(nullptr));
            auto field=[&](const char* key){return !c.contains(key)||c[key].is_null()?std::string(""):c[key].is_string()?c[key].get<std::string>():c[key].dump();};
            candidate_csv<<level<<','<<method<<','<<field("job")<<','<<field("found")<<','<<field("passed")<<','<<field("nominal_time_s")<<','
                <<field("objective")<<','<<field("solve_s")<<','<<field("observed_process_wall_s")<<','<<field("source")<<'\n';
            if((method=="ours"||method=="center")&&!c.value("found",false)) {
                const auto failed=read_json(c["source"].get<std::string>());const auto calls=failed.value("optimizer_calls",Json::array());
                Json record{{"level",level},{"method",method},{"job",c["job"]},{"solve_s",c.value("solve_s",0.)},{"finite_calls",calls.size()},
                    {"source_file",c["source"]}};
                if(!calls.empty()) {
                    const auto& last=calls.back();
                    for(const auto* key:{"status","iterations","minimum_constraint","constraint_violation_inf","native_objective"})
                        record[key]=last.value(key,Json(nullptr));
                }
                failures.push_back(record);
            }
        }
        detail["candidate_count"]=candidates.size();detail["selected_candidate_job"]=selected;
        detail["selected_candidate_s"]=selected_s;detail["candidate_time_sum_s"]=total;
        detail["shared_center_stage_sum_s"]=method=="ours"?by_key.at(level+"/center")["planning_total_s"]:Json(0.);
        detail["trajectory_distance_m"]=raw.value("found",false)?Json(trajectory_distance(Trajectory::load(raw))):Json(nullptr);
        auto value=[](const Json& j){return j.is_null()?std::string(""):j.is_string()?j.get<std::string>():j.dump();};
        costs<<level<<','<<method<<','<<row["outcome"].get<std::string>()<<','<<value(row["full_trajectory_time_s"])<<','
            <<value(detail["trajectory_distance_m"])<<','<<candidates.size()<<','<<value(selected)<<','<<value(selected_s)<<','<<total<<','
            <<value(detail["shared_center_stage_sum_s"])<<','<<source.string()<<'\n';
        details.push_back(detail);
    }
    std::ofstream(out/"PLANNING_DETAILS.json")<<details.dump(2)<<'\n';
    std::ofstream(out/"PLANNING_FAILURES.json")<<failures.dump(2)<<'\n';
    std::ofstream report(out/"PLANNING.md",std::ios::app);
    report<<"\n## 统计口径\n\n";
    report<<"上表只评估规划轨迹，未使用Gazebo实飞成绩。数字为从起点出发、依次穿越七个窗口并返回固定终点的完整轨迹时长，等于各段时间之和，包含最后一窗后的返航和终点减速。粗体仅表示本行无碰撞且任务完整的已选轨迹中时间最短，不表示已求得全局最优。\n\n";
    report<<"碰撞表示名义轨迹的机体外包球与实体STL窗框发生相交，地面项另存，不计入窗框碰撞。无碰撞完成还要求七个有限开口按序正向各穿越一次，并满足起终点PVAJ。判定采用普通浮点自适应Bernstein包络与解析运动范围，非严格舍入区间证书。动力学沿用本轮软约束，因此此处无碰撞不表示全时域动力学硬可行或实际飞行已验证。\n\n";
    report<<"幅度比是各窗运动幅度相对其水平开口尺度的比例，0.25/0.50/0.75不是米。频率列是基准频率的1/3/6倍，不是统一1/3/6 Hz。七窗基准频率依次为0.30、0.40、0.50、0.60、0.55、0.45、0.35 Hz，因此6倍组为1.8至3.6 Hz。W1/3/5/7平移，W2/4/6绕窗外支点摆动，摆长2.20 m，窗口平面方向固定，初相保持原设置。\n\n";
    report<<"四个全局分支各8个初值，本文自由点另做一次最佳合格中心结果开放选点的优化。按原目标J择优，不单独按飞行时间择优。Wang为既有独立复现的局部流程，每次局部4初值，正式结果取同图编译版。失败与碰撞均保留，未替换为旧Python实验成绩。\n\n";
    report<<"## 各方法的名义轨迹结果\n\n|方法|无碰撞完成|碰撞|未找到合格规划|其他未完成|\n|---|---:|---:|---:|---:|\n";
    for(const auto& method:report_methods) {
        int good=0,hit=0,none=0,other=0;
        for(const auto& r:planning)if(r["variant"]==method) {
            const std::string outcome=r["outcome"];if(outcome=="无碰撞完成")++good;else if(outcome=="碰撞")++hit;else if(outcome=="未找到合格规划")++none;else ++other;
        }
        report<<'|'<<method_labels.at(method)<<'|'<<good<<"/10|"<<hit<<'|'<<none<<'|'<<other<<"|\n";
    }
    int paired=0;double minimum=INFINITY,maximum=0.,sum=0.;
    for(const auto& level:report_levels) {
        const auto& free=by_key.at(level+"/ours");const auto& center=by_key.at(level+"/center");
        if(free["outcome"]!="无碰撞完成"||center["outcome"]!="无碰撞完成")continue;
        const double saving=100.*(1.-free["full_trajectory_time_s"].get<double>()/center["full_trajectory_time_s"].get<double>());
        ++paired;minimum=std::min(minimum,saving);maximum=std::max(maximum,saving);sum+=saving;
    }
    report<<"\n本文自由点与中心共同成功"<<paired<<"档，自由点节时"<<std::fixed<<std::setprecision(3)<<minimum<<"%至"<<maximum<<"%，逐档百分比的算术平均为"<<sum/paired<<"%。最高档自由点未找到，未纳入速度优势统计。这是当前十个确定条件的覆盖，不能当作重复随机试验成功率。\n\n";
    report<<"## 求解耗时\n\n下表为本方法全部候选solve_s之和，单位秒。包含有限问题构建、优化与候选内的复查，图导出、编译、加载及协调端独立STL复查未全部计入。并行任务时间相加后不是用户等待时间，也不能当作单次规划延迟。本文自由点9次尝试的和未重复加上中心8初值的共享前置成本，该成本在中心列及PLANNING_COST.csv的shared_center_stage_sum_s单独列出。Wang列为一个完整七窗局部规划流程内部所有求解的累计时间。\n\n";
    report<<"|幅度比／频率倍数|本文自由点9候选|本文中心8候选|TOGT自由8候选|TOGT中心8候选|Wang完整局部流程|\n|---|---:|---:|---:|---:|---:|\n";
    for(const auto& level:report_levels) {
        report<<'|'<<level_labels.at(level);
        for(const auto& method:report_methods)report<<'|'<<std::fixed<<std::setprecision(2)<<by_key.at(level+"/"+method)["planning_total_s"].get<double>();
        report<<"|\n";
    }
    const auto status=read_json(out/"status.json");
    report<<"\n340个候选的协调批次已全部结束，记录墙钟"<<std::fixed<<std::setprecision(3)<<status["elapsed_s"].get<double>()<<"秒（"<<std::setprecision(2)<<status["elapsed_s"].get<double>()/60.<<"分钟），实际8路并行。此批包含Wang解释图开发求解，正式Wang结果另取同图编译版，不能将此墙钟写成50个最终规划各自的耗时。旧批并行16路，本轮初期还存在绑核争用，整批时长不能直接用于证明C++相对Python加速。\n\n";
    report<<"## 最高档自由点未找到的具体情况\n\n0.75／6×的8个独立自由点初值及1个中心释放初值均未获接纳，9次最后一次有限NLP全部达到5000迭代上限，并且最后有限几何约束的违反量超过接纳阈值，因而未进入新的连续复查。并非24轮交换用完，也不是该场景被证明无解。该档中心第3号初值得到20.119989941秒合格轨迹，已经证明当前场景存在合格中心解。\n\n";
    report<<"|自由点初值|有限NLP调用次数|最后状态|末轮迭代数|末轮最小几何约束余量|累计秒|\n|---|---:|---|---:|---:|---:|\n";
    for(const auto& f:failures)if(f["level"]=="a75_f6"&&f["method"]=="ours")
        report<<'|'<<f["job"].get<std::string>()<<'|'<<f["finite_calls"]<<'|'<<f["status"].get<std::string>()<<'|'<<f["iterations"]
            <<'|'<<std::scientific<<std::setprecision(3)<<f["minimum_constraint"].get<double>()<<'|'<<std::fixed<<std::setprecision(2)<<f["solve_s"].get<double>()<<"|\n";
    report<<"\n与旧结果相比，0.75／3×自由点本轮18.062665465秒，旧批约16.307秒；0.75／6×旧批有合格自由解，本轮没有。数学值与一阶导数核对、320组冷初值数组核对均通过，但并不意味着局部非线性优化的迭代路径逐位一致。当前证据能定位本轮失败的直接停止条件，尚未定位与旧批分岔的首个数值步骤。报告保留差异，未据此改算法或补用旧解。\n\n";
    report<<"## 文件与执行状态\n\nPLANNING.json为正式50格汇总，每格source_file指向原始结果。PLANNING_DETAILS.json补充路程、所选候选、候选个数和耗时口径，PLANNING_COST.csv可直接制表，PLANNING_CANDIDATES.csv保留330个全局候选和10个正式编译Wang流程，原解释图开发记录仍在candidate_summary.json。PLANNING_FAILURES.json列出本文方法所有失败初值的最后状态。\n\n";
    report<<"用户要求先不实飞后，已停止实飞协调与终结进程，暂停Gazebo并保留已有记录，见user_stop_flights.json。本文档不引用这些实飞成绩。优化、补点和几何复查运行在C++，数学图/场景的一次性导出及部分离线审计仍使用Python。历史结果和本轮原始候选均未覆盖。\n";
    report.close();
    std::ofstream summary(out/"REPORT.md");summary<<"# C++实验当前状态\n\n规划已完成，按用户要求停止后续实飞。完整规划表、耗时与最高档失败原因见[规划结果](PLANNING.md)。现有实飞数据保留，不作为本次规划结论。\n\n";
    summary<<"原始50格见PLANNING.json，340个正式候选/局部流程明细见PLANNING_CANDIDATES.csv。历史数据未覆盖。\n";
}

int main(int argc,char** argv) {
    try {
        if(argc!=2 && argc!=3)throw std::runtime_error("Usage: native_report RESULT_DIRECTORY [--planning-only]");const fs::path out=fs::absolute(argv[1]);
        if(argc==3 && std::string(argv[2])!="--planning-only")throw std::runtime_error("Unknown report mode");
        auto planning=read_json(out/"PLANNING.json");
        if(argc==3){write_planning_review(out,planning);return 0;}
        for(auto& row:planning) {
            const auto raw=read_json(row["source_file"].get<std::string>());
            if(raw["found"].get<bool>())row["trajectory_distance_m"]=trajectory_distance(Trajectory::load(raw));
        }
        std::ofstream(out/"PLANNING.json")<<planning.dump(2)<<'\n';write_table(out,planning,false);
        Json flight=fs::exists(out/"FLIGHT.json")?read_json(out/"FLIGHT.json"):Json::array();
        if(!flight.empty())write_table(out,flight,true);
        std::ofstream report(out/"REPORT.md");report<<"# C++运行版完整实验\n\n";
        report<<"10档高起伏七窗赛道，5个方法。四个全局分支各8初值，本文自由点另加一次最佳中心释放，Wang沿用原局部四初值流程。\n\n";
        report<<"优化、复查、补约束、MPC、MAVLink及实验调度由C++运行。数学图导出和独立ULog审计保留离线Python。\n\n";
        report<<"规划结果见[规划表](PLANNING.md)，实飞结果见[实飞表](FLIGHT.md)。规划与实飞分别展示，碰撞和未找到直接标注。\n\n";
        report<<"## 条件覆盖\n\n|方法|规划无碰撞完成|实飞无窗框碰撞完赛|实飞碰撞|实飞未完成|\n|---|---:|---:|---:|---:|\n";
        for(const auto& method:report_methods) {
            int nominal=0,complete=0,collision=0,incomplete=0;
            for(const auto& row:planning)if(row["variant"]==method&&row["outcome"]=="无碰撞完成")++nominal;
            for(const auto& row:flight)if(row["method"]==method) {
                const std::string outcome=row["outcome"];
                if(outcome=="无碰撞完赛")++complete;else if(outcome=="碰撞")++collision;else ++incomplete;
            }
            report<<'|'<<method_labels.at(method)<<'|'<<nominal<<"/10|"<<complete<<"/10|"<<collision<<'|'<<incomplete<<"|\n";
        }
        report<<"\n## 实现与证据\n\n";
        report<<"旧实验与补飞数据均保留，SHA-256保护清单见protocol.json。C++运行源、依赖与二进制冻结在runtime_frozen。Wang解释图开发结果保留，正式汇总统一使用同图编译版，见consolidation.json。\n\n";
        report<<"原50轨迹4900项边框检查、50轨迹STL/开口/端点检查、MPC目标与梯度、400条状态观测器重演均完成对应核对。补约束后的中心与自由点有限问题也通过数值及一阶导数核对。\n\n";
        report<<"初期绑核争用记录见resource_adjustment.json和affinity.csv，早期墙钟包含资源争用。有限迭代优化受浮点运算与迭代历史影响，轨迹和控制量允许出现数值差异。完整实飞结果以新原生记录为依据。\n\n";
        report<<"原矩阵采用16路并行，本轮实际采用8路。运行中接管调度未通过自动审批，脚本未执行，原8路继续完成，记录见parallelism_note.json。整批墙钟差异需结合并行度和早期绑核争用解释。\n\n";
        report<<"此处无碰撞完赛计数表示当前十条件各一次执行的覆盖，重复数为1。普通浮点包络、名义模型与Gazebo实飞分别保留证据口径。\n";
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
