// Global dynamic-gate planner with an explicit approach/passage/departure
// funnel around every physical aperture. This is a fast first corridor
// prototype: the independent dense audit remains the acceptance authority.
#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "drolib/race/race_params.hpp"
#include "drolib/race/race_track.hpp"
#include "drolib/shape/polygon.h"
#include "drolib/solver/minco_snap.hpp"

extern "C" int togt_objective_gradient(int,const double*,const double*,const double*,const double*,double*,double*,double*,char*,int);
extern "C" int togt_objective_gradient_weighted(int,const double*,const double*,const double*,const double*,double*,double*,double*,double,char*,int);
using RawEval=double(*)(void*,int,const double*,double*);
extern "C" int togt_released_lbfgs(int,double*,double*,RawEval,void*,int,int,double,int,int,double,double,int*);

namespace {
using drolib::PVAJ;
struct Motion { double translation_amplitude, rotation_amplitude, translation_period, rotation_period, phase; };
struct PhysicalGate { drolib::Waypoint waypoint; Motion motion; int index; };
enum class Phase { Approach, Passage, Departure };
struct Anchor { const PhysicalGate* gate; Phase phase; double normal_offset; };
struct State { Eigen::Vector3d position, rate; Eigen::Matrix3d inplane_rotation; };
constexpr double kBodyRadius=0.37942273679894306;

void beta(double t,Eigen::Matrix<double,8,2>&basis){basis.setZero();double powers[8]={1.};for(int k=1;k<8;++k)powers[k]=powers[k-1]*t;for(int k=0;k<8;++k)basis(k,0)=powers[k];for(int k=1;k<8;++k)basis(k,1)=k*powers[k-1];}

Motion motion_for(int i) {
  double scale=1.;
  if(const char* raw=std::getenv("TOGT_MOTION_SPEED_SCALE")) scale=std::stod(raw);
  if(!(scale>0.)||!std::isfinite(scale)) throw std::runtime_error("invalid TOGT_MOTION_SPEED_SCALE");
  return {.45+.05*((3*i)%6), (M_PI/180.)*(15+4*(i%6)),
          (9.5+.47*(i%11))/scale, (8.+.43*((3*i)%13))/scale, -1.3+.37*i};
}

State anchor_state(const Anchor& anchor,const Eigen::VectorXd& d,double t) {
  const auto& gate=*anchor.gate; const auto& m=gate.motion;
  const auto polygon=std::dynamic_pointer_cast<drolib::Polygon>(gate.waypoint.shape);
  if(!polygon) throw std::runtime_error("corridor prototype expects polygon gates");
  const Eigen::Vector3d c0=gate.waypoint.shape->position;
  const Eigen::Matrix3d base=polygon->R_wb.toRotationMatrix();
  const Eigen::Vector3d tangent=base.col(0), normal=base.col(2);
  const double wt=2*M_PI/m.translation_period, wr=2*M_PI/m.rotation_period;
  const double translation_phase=m.phase+wt*t;
  const double rotation_phase=m.phase+.73+wr*t;
  const Eigen::Vector3d center=c0+tangent*m.translation_amplitude*std::sin(translation_phase);
  const Eigen::Vector3d center_rate=tangent*m.translation_amplitude*wt*std::cos(translation_phase);
  const double angle=m.rotation_amplitude*std::sin(rotation_phase);
  const double angle_rate=m.rotation_amplitude*wr*std::cos(rotation_phase);
  const Eigen::Matrix3d rotation=Eigen::AngleAxisd(angle,normal).toRotationMatrix();
  if(anchor.phase!=Phase::Passage || d.size()==0)
    return {center+normal*anchor.normal_offset,center_rate,rotation};
  const Eigen::Vector3d local=gate.waypoint.shape->toP(d)-c0;
  const Eigen::Vector3d rotated=rotation*local;
  return {center+rotated,center_rate+angle_rate*normal.cross(rotated),rotation};
}

std::vector<Eigen::Vector3d> moving_vertices(const PhysicalGate&gate,double t){
  const auto polygon=std::dynamic_pointer_cast<drolib::Polygon>(gate.waypoint.shape);const auto&m=gate.motion;
  const Eigen::Vector3d c0=gate.waypoint.shape->position;const Eigen::Matrix3d base=polygon->R_wb.toRotationMatrix();const Eigen::Vector3d tangent=base.col(0),normal=base.col(2);
  const Eigen::Vector3d center=c0+tangent*m.translation_amplitude*std::sin(m.phase+2*M_PI*t/m.translation_period);
  const double angle=m.rotation_amplitude*std::sin(m.phase+.73+2*M_PI*t/m.rotation_period);const Eigen::Matrix3d rotation=Eigen::AngleAxisd(angle,normal).toRotationMatrix();
  std::vector<Eigen::Vector3d>out;out.reserve(polygon->vertices.size());for(const auto&v:polygon->vertices)out.push_back(center+rotation*(v-c0));return out;
}

double frame_penalty(const PhysicalGate&gate,const Eigen::Vector3d&p,double t,Eigen::Vector3d*position_gradient){
  const auto vertices=moving_vertices(gate,t);std::vector<double>distances;std::vector<Eigen::Vector3d>distance_gradients;distances.reserve(vertices.size());distance_gradients.reserve(vertices.size());double minimum=std::numeric_limits<double>::infinity();
  for(size_t i=0;i<vertices.size();++i){const auto&a=vertices[i];const auto&b=vertices[(i+1)%vertices.size()];const Eigen::Vector3d edge=b-a;const double alpha=std::clamp((p-a).dot(edge)/edge.squaredNorm(),0.,1.);const Eigen::Vector3d q=a+alpha*edge;const double distance=std::max((p-q).norm(),1e-12);distances.push_back(distance);distance_gradients.push_back((p-q)/distance);minimum=std::min(minimum,distance);}
  constexpr double temperature=.01;double denominator=0.;Eigen::Vector3d smooth_gradient=Eigen::Vector3d::Zero();for(size_t i=0;i<distances.size();++i){const double weight=std::exp(-(distances[i]-minimum)/temperature);denominator+=weight;smooth_gradient+=weight*distance_gradients[i];}smooth_gradient/=denominator;const double smooth_distance=minimum-temperature*std::log(denominator);
  const double violation=kBodyRadius+.03-smooth_distance;if(!(violation>0.)){if(position_gradient)position_gradient->setZero();return 0.;}
  if(position_gradient)*position_gradient=-violation*smooth_gradient;return .5*violation*violation;
}

double forward_time(double k){return k>0?(.5*k+1)*k+1:1/((.5*k-1)*k+1);}
double backward_time(double t){return t>1?std::sqrt(2*t-1)-1:1-std::sqrt(2/t-1);}
double time_jacobian(double k){const double den=(.5*k-1)*k+1;return k>0?k+1:(1-k)/(den*den);}

struct Objective {
  PVAJ head=PVAJ::Zero(),tail=PVAJ::Zero();
  std::vector<PhysicalGate> gates; std::vector<Anchor> anchors;
  double initial_speed=10.,safety_weight=10000.,dynamics_weight=1.;int safety_nodes=16; int evaluations=0; double last_penalty=NAN,last_safety=NAN;
  int temporal_dimension()const{return int(anchors.size()+1);}
  int spatial_dimension()const{int n=0;for(const auto&a:anchors)if(a.phase==Phase::Passage)n+=a.gate->waypoint.shape->dimension();return n;}
  int dimension()const{return temporal_dimension()+spatial_dimension();}
  Eigen::VectorXd initial()const{
    Eigen::VectorXd x=Eigen::VectorXd::Zero(dimension());
    int spatial=temporal_dimension();double clock=0.; Eigen::Vector3d previous=head.col(0);
    for(size_t i=0;i<anchors.size();++i){
      Eigen::VectorXd d;
      if(anchors[i].phase==Phase::Passage){const int n=anchors[i].gate->waypoint.shape->dimension();d=anchors[i].gate->waypoint.shape->toD(anchors[i].gate->waypoint.shape->position);x.segment(spatial,n)=d;spatial+=n;}
      double trial=std::max(.12,(anchor_state(anchors[i],d,clock+.2).position-previous).norm()/initial_speed);
      x(int(i))=backward_time(trial);clock+=trial;previous=anchor_state(anchors[i],d,clock).position;
    }
    x(temporal_dimension()-1)=backward_time(std::max(.2,(tail.col(0)-previous).norm()/initial_speed));
    return x;
  }
  double evaluate(const Eigen::VectorXd& x,Eigen::VectorXd& gradient){
    ++evaluations;const int nt=temporal_dimension();std::vector<double>T(nt),cross(anchors.size());
    double clock=0.;for(int i=0;i<nt;++i){T[i]=forward_time(x(i));clock+=T[i];if(i<int(anchors.size()))cross[i]=clock;}
    std::vector<Eigen::VectorXd> ds(anchors.size());std::vector<State> states;states.reserve(anchors.size());int offset=nt;
    for(size_t i=0;i<anchors.size();++i){if(anchors[i].phase==Phase::Passage){int n=anchors[i].gate->waypoint.shape->dimension();ds[i]=x.segment(offset,n);offset+=n;}states.push_back(anchor_state(anchors[i],ds[i],cross[i]));}
    std::vector<double>points(3*anchors.size()),gp(3*anchors.size()),gt(nt);
    for(size_t i=0;i<anchors.size();++i)for(int a=0;a<3;++a)points[3*i+a]=states[i].position(a);
    double packed_head[12],packed_tail[12];for(int r=0;r<3;++r)for(int c=0;c<4;++c){packed_head[4*r+c]=head(r,c);packed_tail[4*r+c]=tail(r,c);}
    double cost=0.;char error[512]={};int status=togt_objective_gradient_weighted(nt,packed_head,packed_tail,points.data(),T.data(),&cost,gp.data(),gt.data(),dynamics_weight,error,512);
    if(status||!std::isfinite(cost)){gradient.setZero();return 1e30;}
    // Direct moving-frame safety layer.  It is evaluated in native C++ and
    // differentiated through the same MINCO adjoint as released TOGT.
    Eigen::Matrix3Xd P(3,nt-1);for(int i=0;i<nt-1;++i)P.col(i)=states[i].position;
    drolib::MincoSnap minco;minco.setConditions(head,tail,nt);minco.setParameters(P,Eigen::Map<Eigen::VectorXd>(T.data(),nt));
    const Eigen::MatrixX3d&C=minco.getCoeffs();Eigen::MatrixX3d gC=Eigen::MatrixX3d::Zero(8*nt,3);Eigen::VectorXd directT=Eigen::VectorXd::Zero(nt),startT=Eigen::VectorXd::Zero(nt);double safety=0.,elapsed=0.;Eigen::Matrix<double,8,2>basis;
    for(int segment=0;segment<nt;++segment){const double duration=T[segment],fraction=1./safety_nodes,step=duration*fraction;const auto coeff=C.block<8,3>(8*segment,0);
      for(int node=0;node<=safety_nodes;++node){const double alpha=node*fraction,local=alpha*duration,absolute=elapsed+local,w=(node==0||node==safety_nodes)?.5:1.;beta(local,basis);const Eigen::Vector3d position=coeff.transpose()*basis.col(0),velocity=coeff.transpose()*basis.col(1);
        for(const auto&gate:gates){Eigen::Vector3d pg;const double pen=frame_penalty(gate,position,absolute,&pg);if(pen==0.)continue;const double h=1e-5;const double explicit_t=(frame_penalty(gate,position,absolute+h,nullptr)-frame_penalty(gate,position,absolute-h,nullptr))/(2*h);safety+=w*step*pen;gC.block<8,3>(8*segment,0)+=safety_weight*w*step*basis.col(0)*pg.transpose();directT(segment)+=safety_weight*w*(fraction*pen+step*alpha*(pg.dot(velocity)+explicit_t));startT(segment)+=safety_weight*w*step*explicit_t;}}
      elapsed+=duration;
    }
    for(int segment=0;segment<nt;++segment)for(int prior=0;prior<segment;++prior)directT(prior)+=startT(segment);
    Eigen::Matrix3Xd safety_gp(3,nt-1);Eigen::VectorXd safety_gt(nt);minco.propagateGrad(gC,directT,safety_gp,safety_gt);cost+=safety_weight*safety;for(int i=0;i<nt-1;++i)for(int a=0;a<3;++a)gp[3*i+a]+=safety_gp(a,i);for(int i=0;i<nt;++i)gt[i]+=safety_gt(i);last_safety=safety;
    gradient=Eigen::VectorXd::Zero(dimension());std::vector<double> crossing_gradient(anchors.size());offset=nt;
    for(size_t i=0;i<anchors.size();++i){Eigen::Vector3d world(gp[3*i],gp[3*i+1],gp[3*i+2]);crossing_gradient[i]=world.dot(states[i].rate);
      if(anchors[i].phase==Phase::Passage){const int n=ds[i].size();Eigen::Matrix3Xd one(3,1);one.col(0)=states[i].inplane_rotation.transpose()*world;Eigen::Map<Eigen::VectorXd>gd(gradient.data()+offset,n);anchors[i].gate->waypoint.shape->getGradD(ds[i],one,0,0,gd);offset+=n;}}
    for(int j=0;j<nt;++j){double chain=gt[j];for(size_t i=j;i<crossing_gradient.size();++i)chain+=crossing_gradient[i];gradient(j)=chain*time_jacobian(x(j));}
    last_penalty=(cost-safety_weight*safety-std::accumulate(T.begin(),T.end(),0.))/dynamics_weight;return cost;
  }
  static double callback(void*p,int n,const double*x,double*g){auto*o=static_cast<Objective*>(p);Eigen::Map<const Eigen::VectorXd>X(x,n);Eigen::VectorXd G(n);double value=o->evaluate(X,G);Eigen::Map<Eigen::VectorXd>(g,n)=G;return value;}
};

void export_csv(const drolib::PiecewisePolynomial<7>& trajectory,double duration,const std::string& path){
  double step=.001;if(const char*raw=std::getenv("TOGT_EXPORT_STEP"))step=std::stod(raw);
  std::ofstream out(path);out<<"time,px,py,pz,vx,vy,vz,ax,ay,az,jx,jy,jz,sx,sy,sz\n";const int count=std::max(1,int(std::ceil(duration/step)));
  for(int i=0;i<=count;++i){double t=duration*i/count;auto p=trajectory.getPos(t),v=trajectory.getVel(t),a=trajectory.getAcc(t),j=trajectory.getJer(t),s=trajectory.getSna(t);out<<std::setprecision(12)<<t<<","<<p.x()<<","<<p.y()<<","<<p.z()<<","<<v.x()<<","<<v.y()<<","<<v.z()<<","<<a.x()<<","<<a.y()<<","<<a.z()<<","<<j.x()<<","<<j.y()<<","<<j.z()<<","<<s.x()<<","<<s.y()<<","<<s.z()<<"\n";}
}
}

int main(int argc,char**argv){
  if(argc!=5){std::cerr<<"usage: global_corridor_togt <parameter_dir> <setup_yaml> <track_yaml> <trajectory_csv>\n";return 64;}
  drolib::RaceParams params(std::filesystem::absolute(argv[1]).string(),argv[2]);
  drolib::RaceTrack track(std::filesystem::absolute(argv[3]));track.initCorridors(0);
  Objective objective;objective.head=track.initState.toPVAJ();objective.tail=track.endState.toPVAJ();
  objective.initial_speed=6.;if(const char*raw=std::getenv("TOGT_INITIAL_SPEED"))objective.initial_speed=std::stod(raw);
  if(const char*raw=std::getenv("TOGT_SAFETY_WEIGHT"))objective.safety_weight=std::stod(raw);
  if(const char*raw=std::getenv("TOGT_DYNAMICS_WEIGHT"))objective.dynamics_weight=std::stod(raw);
  if(const char*raw=std::getenv("TOGT_SAFETY_NODES"))objective.safety_nodes=std::stoi(raw);
  double depth=.70;if(const char*raw=std::getenv("TOGT_CORRIDOR_DEPTH"))depth=std::stod(raw);
  for(size_t i=0;i<track.gates.size();++i)objective.gates.push_back({track.gates[i]->corridor.front(),motion_for(int(i)),int(i)});
  for(auto&gate:objective.gates)objective.anchors.push_back({&gate,Phase::Passage,0.});
  int past=8;if(const char*raw=std::getenv("TOGT_LBFGS_PAST"))past=std::stoi(raw);
  Eigen::VectorXd x=objective.initial();double cost=0.;int iterations=0;auto start=std::chrono::steady_clock::now();
  int code=togt_released_lbfgs(x.size(),x.data(),&cost,Objective::callback,&objective,256,past,1e-32,256,0,1e-5,1e-5,&iterations);
  Eigen::VectorXd dummy(x.size());objective.evaluate(x,dummy);
  const int nt=objective.temporal_dimension();Eigen::VectorXd T(nt);for(int i=0;i<nt;++i)T(i)=forward_time(x(i));
  double output_time_scale=1.;if(const char*raw=std::getenv("TOGT_OUTPUT_TIME_SCALE"))output_time_scale=std::stod(raw);T*=output_time_scale;
  Eigen::Matrix3Xd P(3,nt-1);double clock=0.;int offset=nt;
  for(size_t i=0;i<objective.anchors.size();++i){clock+=T(int(i));Eigen::VectorXd d;int n=objective.anchors[i].gate->waypoint.shape->dimension();d=x.segment(offset,n);offset+=n;P.col(int(i))=anchor_state(objective.anchors[i],d,clock).position;}
  drolib::MincoSnap minco;minco.setConditions(objective.head,objective.tail,nt);minco.setParameters(P,T);drolib::PiecewisePolynomial<7> trajectory;minco.getTrajectory(trajectory);double solve_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();export_csv(trajectory,T.sum(),argv[4]);
  std::cout<<std::fixed<<std::setprecision(9)<<"implementation=pure_cpp_global_safe_togt\n"<<"gate_count="<<objective.gates.size()<<"\n"<<"anchor_count="<<objective.anchors.size()<<"\n"<<"safety_weight="<<objective.safety_weight<<"\n"<<"dynamics_weight="<<objective.dynamics_weight<<"\n"<<"safety_nodes_per_piece="<<objective.safety_nodes<<"\n"<<"lbfgs_past="<<past<<"\n"<<"relative_cost_threshold=0.000010000\n"<<"output_time_scale="<<output_time_scale<<"\n"<<"optimizer_code="<<code<<"\n"<<"optimizer_converged="<<(code==0||code==1)<<"\n"<<"iterations="<<iterations<<"\n"<<"evaluations="<<objective.evaluations<<"\n"<<"final_gradient_norm="<<dummy.norm()<<"\n"<<"planning_s="<<solve_seconds<<"\n"<<"flight_time_s="<<T.sum()<<"\n"<<"soft_dynamics_penalty_unweighted_at_optimizer_solution="<<objective.last_penalty<<"\n"<<"sampled_safety_integral_at_optimizer_solution="<<objective.last_safety<<"\n";
  return std::isfinite(cost)&&x.allFinite()?0:2;
}
