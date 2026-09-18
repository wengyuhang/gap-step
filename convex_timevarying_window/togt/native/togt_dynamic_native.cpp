// Pure-C++ comparison: globally joint Dynamic-TOGT versus rolling Dynamic-PVAJ.
// Dynamic gate geometry, MINCO, analytic chain rules and L-BFGS all execute here.
#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "drolib/race/race_params.hpp"
#include "drolib/race/race_track.hpp"
#include "drolib/shape/polygon.h"
#include "drolib/solver/minco_snap.hpp"

extern "C" int togt_objective_gradient(int,const double*,const double*,const double*,const double*,double*,double*,double*,char*,int);
extern "C" int togt_objective_gradient_tail(int,const double*,const double*,const double*,const double*,double*,double*,double*,double*,char*,int);
using RawEval=double(*)(void*,int,const double*,double*);
extern "C" int togt_released_lbfgs(int,double*,double*,RawEval,void*,int,int,double,int,int,double,double,int*);

namespace {
using drolib::PVAJ;
struct Motion { Eigen::Vector3d at, ar; double pt, pr, phase; };
struct Gate { drolib::Waypoint waypoint; Motion motion; int global_index; };

Eigen::Matrix3d rpy(const Eigen::Vector3d& a) {
  return (Eigen::AngleAxisd(a.z(),Eigen::Vector3d::UnitZ())*
          Eigen::AngleAxisd(a.y(),Eigen::Vector3d::UnitY())*
          Eigen::AngleAxisd(a.x(),Eigen::Vector3d::UnitX())).toRotationMatrix();
}
struct GateState { Eigen::Vector3d p, rate; Eigen::Matrix3d rotation; };
GateState gate_state(const Gate& gate,const Eigen::VectorXd& d,double t) {
  const auto& m=gate.motion; const Eigen::Vector3d c0=gate.waypoint.shape->position;
  const auto polygon=std::dynamic_pointer_cast<drolib::Polygon>(gate.waypoint.shape);
  if(!polygon)throw std::runtime_error("dynamic native runner expects polygon gates");
  const Eigen::Matrix3d base=polygon->R_wb.toRotationMatrix();
  const Eigen::Vector3d tangent=base.col(0),normal=base.col(2);
  auto position_at=[&](double q) -> Eigen::Vector3d {
    const Eigen::Vector3d center=c0+tangent*m.at.y()*std::sin(m.phase+2*M_PI*q/m.pt);
    const double angle=m.ar.x()*std::sin(m.phase+.73+2*M_PI*q/m.pr);
    const Eigen::Matrix3d rotation=Eigen::AngleAxisd(angle,normal).toRotationMatrix();
    return (center+rotation*(gate.waypoint.shape->toP(d)-c0)).eval();
  };
  const double h=1e-6;
  const double angle=m.ar.x()*std::sin(m.phase+.73+2*M_PI*t/m.pr);
  const Eigen::Matrix3d rotation=Eigen::AngleAxisd(angle,normal).toRotationMatrix();
  return {position_at(t),(position_at(t+h)-position_at(t-h))/(2*h),rotation};
}
GateState gate_center_state(const Gate& gate,double t) {
  const auto& m=gate.motion; const Eigen::Vector3d c0=gate.waypoint.shape->position;
  const auto polygon=std::dynamic_pointer_cast<drolib::Polygon>(gate.waypoint.shape);
  if(!polygon)throw std::runtime_error("dynamic native runner expects polygon gates");
  const Eigen::Matrix3d base=polygon->R_wb.toRotationMatrix();
  const Eigen::Vector3d tangent=base.col(0),normal=base.col(2);
  const double omega=2*M_PI/m.pt;
  const Eigen::Vector3d center=c0+tangent*m.at.y()*std::sin(m.phase+omega*t);
  const Eigen::Vector3d rate=tangent*m.at.y()*omega*std::cos(m.phase+omega*t);
  const double angle=m.ar.x()*std::sin(m.phase+.73+2*M_PI*t/m.pr);
  return {center,rate,Eigen::AngleAxisd(angle,normal).toRotationMatrix()};
}
Motion motion_for(int i) {
  Motion m;
  // All physical gates remain in the same vertical-plane family: translate
  // laterally inside that plane and rotate only about its normal.  Amplitude,
  // period and phase remain heterogeneous across the 50 gates.
  m.at=Eigen::Vector3d(0.,.45+.05*((3*i)%6),0.);
  m.ar=(M_PI/180.)*Eigen::Vector3d(15+4*(i%6),0.,0.);
  double speed_scale=1.;if(const char*raw=std::getenv("TOGT_MOTION_SPEED_SCALE"))speed_scale=std::stod(raw);
  if(!(speed_scale>0.)||!std::isfinite(speed_scale))throw std::runtime_error("TOGT_MOTION_SPEED_SCALE must be finite and positive");
  m.pt=(9.5+.47*(i%11))/speed_scale;m.pr=(8.0+.43*((3*i)%13))/speed_scale;m.phase=-1.3+.37*i;return m;
}
double forward_t(double k) { return k>0 ? (.5*k+1)*k+1 : 1/((.5*k-1)*k+1); }
double backward_t(double t) { return t>1 ? std::sqrt(2*t-1)-1 : 1-std::sqrt(2/t-1); }
double time_jac(double k) { double den=(.5*k-1)*k+1; return k>0?k+1:(1-k)/(den*den); }

struct Objective {
  PVAJ head=PVAJ::Zero(), tail_fixed=PVAJ::Zero(); std::vector<Gate> gates;
  double absolute_time=0,initial_speed=10.,tail_prior_weight=0.; bool free_tail=false,tail_point_only=false,has_tail_initial=false,fixed_centers=false; std::vector<char> fixed_gate; PVAJ tail_initial=PVAJ::Zero(); int evals=0; double last_penalty=NAN;
  static constexpr double tail_scale[4]={1.,8.,20.,60.};
  bool is_fixed(size_t i) const {return fixed_centers||(!fixed_gate.empty()&&fixed_gate[i]);}
  int temporal() const { return free_tail||tail_point_only ? int(gates.size()) : int(gates.size()+1); }
  int spatial() const {int n=0;for(size_t i=0;i<gates.size();++i)if(!is_fixed(i))n+=gates[i].waypoint.shape->dimension();return n;}
  int dimension() const { return temporal()+spatial()+(free_tail?9:0); }
  Eigen::VectorXd initial(double speed=1.) const {
    const int nt=temporal(), ns=spatial(); Eigen::VectorXd x=Eigen::VectorXd::Zero(dimension());
    int off=nt; Eigen::Vector3d prev=head.col(0);
    for(size_t i=0;i<gates.size();++i) {
      auto& g=gates[i]; Eigen::Vector3d p;
      if(is_fixed(i))p=gate_center_state(g,absolute_time+i+1).p;
      else{int dim=g.waypoint.shape->dimension();Eigen::VectorXd d=g.waypoint.shape->toD(g.waypoint.shape->position);x.segment(off,dim)=d;off+=dim;p=gate_state(g,d,absolute_time+i+1).p;}
      double duration=std::max(.2,(p-prev).norm()/speed);
      x(i)=backward_t(duration); prev=p;
    }
    if(!free_tail&&!tail_point_only) x(nt-1)=backward_t(std::max(.2,(tail_fixed.col(0)-prev).norm()/speed));
    else if(has_tail_initial)for(int c=1;c<4;c++)for(int r=0;r<3;r++)x(off++)=tail_initial(r,c)/tail_scale[c];
    return x;
  }
  double evaluate(const Eigen::VectorXd& x,Eigen::VectorXd& grad) {
    ++evals; const int nt=temporal(); std::vector<double>T(nt),cross(gates.size());
    double clock=absolute_time; for(int i=0;i<nt;i++){T[i]=forward_t(x(i));clock+=T[i];if(i<(int)gates.size())cross[i]=clock;}
    std::vector<Eigen::VectorXd> ds; std::vector<GateState> states; int off=nt;
    for(size_t i=0;i<gates.size();++i){if(is_fixed(i)){ds.emplace_back();states.push_back(gate_center_state(gates[i],cross[i]));}else{int n=gates[i].waypoint.shape->dimension();ds.push_back(x.segment(off,n));off+=n;states.push_back(gate_state(gates[i],ds.back(),cross[i]));}}
    PVAJ tail=tail_fixed; const bool mapped_tail=free_tail||tail_point_only;const int inner=mapped_tail?int(gates.size()-1):int(gates.size());
    if(mapped_tail)tail.col(0)=states.back().p;if(free_tail)for(int c=1;c<4;c++)for(int r=0;r<3;r++)tail(r,c)=x(off++)*tail_scale[c];
    std::vector<double> points(3*inner),gp(3*inner),gt(nt),gtail(12); for(int i=0;i<inner;i++)for(int a=0;a<3;a++)points[3*i+a]=states[i].p(a);
    double packed_head[12],packed_tail[12];for(int r=0;r<3;r++)for(int c=0;c<4;c++){packed_head[4*r+c]=head(r,c);packed_tail[4*r+c]=tail(r,c);}
    double cost=0; char error[512]={}; int status=mapped_tail?
      togt_objective_gradient_tail(nt,packed_head,packed_tail,points.data(),T.data(),&cost,gp.data(),gt.data(),gtail.data(),error,512):
      togt_objective_gradient(nt,packed_head,packed_tail,points.data(),T.data(),&cost,gp.data(),gt.data(),error,512);
    if(status||!std::isfinite(cost)){std::cerr<<"native objective failure: "<<error<<" status="<<status<<" cost="<<cost<<"\n";grad.setZero();return 1e30;}
    grad=Eigen::VectorXd::Zero(dimension()); std::vector<double> crossing_grad(gates.size()); off=nt;
    for(size_t i=0;i<gates.size();++i){
      Eigen::Vector3d world_grad; if(mapped_tail&&i+1==gates.size())for(int a=0;a<3;a++)world_grad(a)=gtail[4*a]; else for(int a=0;a<3;a++)world_grad(a)=gp[3*i+a];
      crossing_grad[i]=world_grad.dot(states[i].rate);if(!is_fixed(i)){Eigen::Vector3d static_grad=states[i].rotation.transpose()*world_grad;
      Eigen::Matrix3Xd one(3,1);one.col(0)=static_grad;int n=ds[i].size();Eigen::Map<Eigen::VectorXd> gd(grad.data()+off,n);
      gates[i].waypoint.shape->getGradD(ds[i],one,0,0,gd);off+=n;}
    }
    for(int j=0;j<nt;j++){double chain=gt[j];for(size_t i=j;i<crossing_grad.size();++i)chain+=crossing_grad[i];grad(j)=chain*time_jac(x(j));}
    if(free_tail){for(int c=1;c<4;c++)for(int r=0;r<3;r++){double actual_gradient=gtail[4*r+c];if(has_tail_initial){double error_state=tail(r,c)-tail_initial(r,c);cost+=tail_prior_weight*error_state*error_state/(tail_scale[c]*tail_scale[c]);actual_gradient+=2*tail_prior_weight*error_state/(tail_scale[c]*tail_scale[c]);}grad(off++)=actual_gradient*tail_scale[c];}}
    double flight=std::accumulate(T.begin(),T.end(),0.);last_penalty=cost-flight;return cost;
  }
  static double callback(void* p,int n,const double*x,double*g){auto*o=static_cast<Objective*>(p);Eigen::Map<const Eigen::VectorXd>X(x,n);Eigen::VectorXd G(n);double value=o->evaluate(X,G);Eigen::Map<Eigen::VectorXd>(g,n)=G;return value;}
};
struct Solved { bool ok,converged; Eigen::VectorXd x,T; Eigen::Matrix3Xd P; PVAJ tail; drolib::PiecewisePolynomial<7> traj; double cost,penalty,cpu; int iterations,evals; };
Solved solve(Objective& o) {
  Eigen::VectorXd x=o.initial(o.initial_speed);double cost=0;int iterations=0;auto tic=std::chrono::steady_clock::now();
  int code=togt_released_lbfgs(x.size(),x.data(),&cost,Objective::callback,&o,256,32,1e-32,256,0,1e-5,1e-5,&iterations);
  double cpu=std::chrono::duration<double>(std::chrono::steady_clock::now()-tic).count();Eigen::VectorXd dummy(x.size());o.evaluate(x,dummy);
  int nt=o.temporal(),off=nt;Eigen::VectorXd T(nt);for(int i=0;i<nt;i++)T(i)=forward_t(x(i));Eigen::Matrix3Xd P(3,nt-1);double clock=o.absolute_time;
  for(int i=0;i<nt;i++){clock+=T(i);if(i<(int)o.gates.size()){if(o.is_fixed(i)){if(i<nt-1)P.col(i)=gate_center_state(o.gates[i],clock).p;}else{int d=o.gates[i].waypoint.shape->dimension();if(i<nt-1)P.col(i)=gate_state(o.gates[i],x.segment(off,d),clock).p;off+=d;}}}
  PVAJ tail=o.tail_fixed;if(o.free_tail||o.tail_point_only){int d=o.gates.back().waypoint.shape->dimension();tail.col(0)=gate_state(o.gates.back(),x.segment(off-d,d),clock).p;if(o.free_tail)for(int c=1;c<4;c++)for(int r=0;r<3;r++)tail(r,c)=x(off++)*Objective::tail_scale[c];}
  drolib::MincoSnap minco;minco.setConditions(o.head,tail,nt);minco.setParameters(P,T);drolib::PiecewisePolynomial<7> traj;minco.getTrajectory(traj);
  if(code<0)std::cerr<<"L-BFGS failure code="<<code<<" iterations="<<iterations<<" cost="<<cost<<"\n";
  return {std::isfinite(cost)&&x.allFinite(),code>=0,x,T,P,tail,traj,cost,o.last_penalty,cpu,iterations,o.evals};
}
std::vector<Gate> all_gates(drolib::RaceTrack& track){track.initCorridors(0);std::vector<Gate> out;for(size_t i=0;i<track.gates.size();++i)out.push_back({track.gates[i]->corridor.front(),motion_for(int(i)),int(i)});return out;}
struct ExecutedChunk { drolib::PiecewisePolynomial<7> trajectory; double duration; };
struct PlanCall {int index,first_gate,horizon;double planning,executed_duration;int iterations,evaluations;bool converged;};
struct Summary {bool ok=true,converged=true;double flight=0,planning=0,penalty=0;int evals=0,calls=0,failed_stops=0;std::vector<ExecutedChunk> chunks;std::vector<PlanCall> plan_calls;};
Summary global(const std::vector<Gate>& gates,const PVAJ& head,const PVAJ& tail){Objective o;o.head=head;o.tail_fixed=tail;o.gates=gates;auto s=solve(o);Summary z{s.ok,s.converged,s.T.sum(),s.cpu,s.penalty,s.evals,1,s.converged?0:1};z.chunks.push_back({s.traj,s.T.sum()});return z;}
Summary global_centered(const std::vector<Gate>& gates,const PVAJ& head,const PVAJ& tail){Objective o;o.head=head;o.tail_fixed=tail;o.gates=gates;o.fixed_centers=true;auto s=solve(o);Summary z{s.ok,s.converged,s.T.sum(),s.cpu,s.penalty,s.evals,1,s.converged?0:1};z.chunks.push_back({s.traj,s.T.sum()});return z;}
Summary rolling(const std::vector<Gate>& gates,const PVAJ& start,const PVAJ& goal,int H){Summary z;PVAJ head=start;double clock=0;size_t next=0;double prior_weight=0.;if(const char*raw=std::getenv("TOGT_TAIL_PRIOR_WEIGHT"))prior_weight=std::stod(raw);const bool lookahead=std::getenv("TOGT_ENABLE_LOOKAHEAD")!=nullptr;while(next<gates.size()){
  size_t count=std::min<size_t>(H,gates.size()-next);bool final=gates.size()-next<=size_t(H);double speed=std::clamp(head.col(1).norm(),6.,15.);if(next==0)speed=10.;PVAJ predicted=PVAJ::Zero();if(!final&&lookahead){Objective seed;seed.head=head;seed.absolute_time=clock;seed.initial_speed=speed;seed.free_tail=true;seed.gates.assign(gates.begin()+next,gates.begin()+next+count+1);auto warm=solve(seed);z.planning+=warm.cpu;z.evals+=warm.evals;if(!warm.ok){z.ok=false;break;}double cut=warm.T.head(count).sum();predicted=warm.traj.getPVAJ(cut);}Objective o;o.head=head;o.tail_fixed=goal;o.absolute_time=clock;o.initial_speed=speed;o.tail_prior_weight=prior_weight;o.gates.assign(gates.begin()+next,gates.begin()+next+count);o.free_tail=!final;if(!final&&lookahead){o.has_tail_initial=true;o.tail_initial=predicted;}auto s=solve(o);z.calls++;z.planning+=s.cpu;z.evals+=s.evals;z.converged&=s.converged;z.failed_stops+=s.converged?0:1;if(!s.ok){z.ok=false;break;}if(final){z.plan_calls.push_back({z.calls,int(next)+1,int(count),s.cpu,s.T.sum(),s.iterations,s.evals,s.converged});z.flight+=s.T.sum();z.penalty+=s.penalty;z.chunks.push_back({s.traj,s.T.sum()});break;}double dt=s.T(0);z.plan_calls.push_back({z.calls,int(next)+1,int(count),s.cpu,dt,s.iterations,s.evals,s.converged});z.flight+=dt;clock+=dt;z.chunks.push_back({s.traj,dt});PVAJ old=head;head=s.traj.getPVAJ(dt);double ph[12],pt[12];for(int r=0;r<3;r++)for(int c=0;c<4;c++){ph[4*r+c]=old(r,c);pt[4*r+c]=head(r,c);}double piece_cost=0;double gp[1],gt[1];char err[512]={};togt_objective_gradient(1,ph,pt,nullptr,&dt,&piece_cost,gp,gt,err,512);z.penalty+=piece_cost-dt;next++;}return z;}
Summary rolling_center_tail(const std::vector<Gate>& gates,const PVAJ& start,const PVAJ& goal,int H){Summary z;PVAJ head=start;double clock=0;size_t next=0;while(next<gates.size()){const size_t remaining=gates.size()-next;const bool final=remaining<=size_t(H);Objective o;o.head=head;o.tail_fixed=goal;o.absolute_time=clock;o.initial_speed=next==0?10.:std::clamp(head.col(1).norm(),6.,15.);o.gates.assign(gates.begin()+next,gates.end());o.fixed_gate.assign(remaining,0);for(size_t i=H;i<remaining;++i)o.fixed_gate[i]=1;auto s=solve(o);z.calls++;z.planning+=s.cpu;z.evals+=s.evals;z.converged&=s.converged;z.failed_stops+=s.converged?0:1;if(!s.ok){z.ok=false;break;}if(final){z.plan_calls.push_back({z.calls,int(next)+1,int(remaining),s.cpu,s.T.sum(),s.iterations,s.evals,s.converged});z.flight+=s.T.sum();z.penalty+=s.penalty;z.chunks.push_back({s.traj,s.T.sum()});break;}double dt=s.T(0);z.plan_calls.push_back({z.calls,int(next)+1,H,s.cpu,dt,s.iterations,s.evals,s.converged});z.flight+=dt;clock+=dt;z.chunks.push_back({s.traj,dt});PVAJ old=head;head=s.traj.getPVAJ(dt);double ph[12],pt[12];for(int r=0;r<3;r++)for(int c=0;c<4;c++){ph[4*r+c]=old(r,c);pt[4*r+c]=head(r,c);}double piece_cost=0;double gp[1],gt[1];char err[512]={};togt_objective_gradient(1,ph,pt,nullptr,&dt,&piece_cost,gp,gt,err,512);z.penalty+=piece_cost-dt;++next;}return z;}
Summary rolling_stop_tail(const std::vector<Gate>& gates,const PVAJ& start,const PVAJ& goal,int H){Summary z;PVAJ head=start;double clock=0;size_t next=0;while(next<gates.size()){const size_t remaining=gates.size()-next;const size_t count=std::min<size_t>(H,remaining);const bool final=remaining<=size_t(H);Objective o;o.head=head;o.tail_fixed=final?goal:PVAJ::Zero();o.absolute_time=clock;o.initial_speed=next==0?10.:std::clamp(head.col(1).norm(),6.,15.);o.gates.assign(gates.begin()+next,gates.begin()+next+count);o.tail_point_only=!final;auto s=solve(o);z.calls++;z.planning+=s.cpu;z.evals+=s.evals;z.converged&=s.converged;z.failed_stops+=s.converged?0:1;if(!s.ok){z.ok=false;break;}if(final){z.plan_calls.push_back({z.calls,int(next)+1,int(count),s.cpu,s.T.sum(),s.iterations,s.evals,s.converged});z.flight+=s.T.sum();z.penalty+=s.penalty;z.chunks.push_back({s.traj,s.T.sum()});break;}double dt=s.T(0);z.plan_calls.push_back({z.calls,int(next)+1,int(count),s.cpu,dt,s.iterations,s.evals,s.converged});z.flight+=dt;clock+=dt;z.chunks.push_back({s.traj,dt});PVAJ old=head;head=s.traj.getPVAJ(dt);double ph[12],pt[12];for(int r=0;r<3;r++)for(int c=0;c<4;c++){ph[4*r+c]=old(r,c);pt[4*r+c]=head(r,c);}double piece_cost=0;double gp[1],gt[1];char err[512]={};togt_objective_gradient(1,ph,pt,nullptr,&dt,&piece_cost,gp,gt,err,512);z.penalty+=piece_cost-dt;++next;}return z;}

void print(const char*name,const Summary&s){std::cout<<name<<"_candidate_valid="<<s.ok<<"\n"<<name<<"_optimizer_converged="<<s.converged<<"\n"<<name<<"_failed_stops="<<s.failed_stops<<"\n"<<name<<"_flight_time_s="<<s.flight<<"\n"<<name<<"_planning_s="<<s.planning<<"\n"<<name<<"_soft_penalty="<<s.penalty<<"\n"<<name<<"_evaluations="<<s.evals<<"\n"<<name<<"_calls="<<s.calls<<"\n";for(const auto&c:s.plan_calls)std::cout<<name<<"_call="<<c.index<<",first_gate="<<c.first_gate<<",horizon="<<c.horizon<<",planning_s="<<c.planning<<",executed_duration_s="<<c.executed_duration<<",iterations="<<c.iterations<<",evaluations="<<c.evaluations<<",converged="<<c.converged<<"\n";}
void export_csv(const Summary&s,const std::string&path){double step=.025;if(const char*raw=std::getenv("TOGT_EXPORT_STEP"))step=std::stod(raw);if(!(step>0.)||!std::isfinite(step))throw std::runtime_error("TOGT_EXPORT_STEP must be finite and positive");std::ofstream out(path);out<<"time,px,py,pz,vx,vy,vz,ax,ay,az\n";double global=0;for(const auto&chunk:s.chunks){int samples=std::max(2,int(std::ceil(chunk.duration/step)));for(int i=0;i<samples;i++){double t=chunk.duration*i/samples;auto p=chunk.trajectory.getPos(t),v=chunk.trajectory.getVel(t),a=chunk.trajectory.getAcc(t);out<<std::setprecision(12)<<global+t<<","<<p.x()<<","<<p.y()<<","<<p.z()<<","<<v.x()<<","<<v.y()<<","<<v.z()<<","<<a.x()<<","<<a.y()<<","<<a.z()<<"\n";}global+=chunk.duration;}const auto&last=s.chunks.back();auto p=last.trajectory.getPos(last.duration),v=last.trajectory.getVel(last.duration),a=last.trajectory.getAcc(last.duration);out<<global<<","<<p.x()<<","<<p.y()<<","<<p.z()<<","<<v.x()<<","<<v.y()<<","<<v.z()<<","<<a.x()<<","<<a.y()<<","<<a.z()<<"\n";}
}
int main(int argc,char**argv){if(argc!=5&&argc!=6){std::cerr<<"usage: togt_dynamic_native <parameter_dir> <setup_yaml> <track_yaml> <H> [rolling_csv]\n";return 64;}drolib::RaceParams params(std::filesystem::absolute(argv[1]).string(),argv[2]);drolib::RaceTrack track(std::filesystem::absolute(argv[3]));auto gates=all_gates(track);int H=std::stoi(argv[4]);std::cout<<std::fixed<<std::setprecision(9)<<"implementation=pure_cpp\n"<<"gate_count="<<gates.size()<<"\n"<<"horizon="<<H<<"\n";auto c=global_centered(gates,track.initState.toPVAJ(),track.endState.toPVAJ());print("dynamic_togt_centered",c);auto g=global(gates,track.initState.toPVAJ(),track.endState.toPVAJ());print("dynamic_togt",g);auto h=rolling_center_tail(gates,track.initState.toPVAJ(),track.endState.toPVAJ(),H);print("dynamic_rolling_center_tail",h);auto s=rolling_stop_tail(gates,track.initState.toPVAJ(),track.endState.toPVAJ(),H);print("dynamic_rolling_stop_tail",s);auto r=rolling(gates,track.initState.toPVAJ(),track.endState.toPVAJ(),H);print("dynamic_rolling_pvaj",r);if(argc==6){const char*mode=std::getenv("TOGT_EXPORT_METHOD");if(mode&&std::string(mode)=="stop_tail"&&s.ok)export_csv(s,argv[5]);else if(h.ok)export_csv(h,argv[5]);}return c.ok&&g.ok&&h.ok&&s.ok&&r.ok?0:2;}
