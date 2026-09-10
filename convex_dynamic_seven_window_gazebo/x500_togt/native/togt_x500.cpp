#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <stdexcept>
#include "drolib/planner/angle.hpp"
#include "drolib/planner/traj_params.hpp"
#include "drolib/solver/minco_snap.hpp"
#include "drolib/system/quadrotor_manifold.hpp"

namespace {
void standard(drolib::TrajParams &p) {
  p.piecesPerSegment=13; p.speedGuess=1; p.maxVelNorm=11.95; p.maxOmgXY=3.839724354;
  p.maxOmgZ=3.490658504; p.maxTiltedAngle=0.78; p.maxThr=8.54858; p.minThr=0;
  p.weightTime=1; p.weightEnergy=0; p.weightPos=0; p.weightVel=0;
  p.weightOmg=1000; p.weightRot=1000; p.weightThr=1000; p.smoothingEps=.01;
  p.numConstPena=16; p.dynamicConstCheck=true; p.minNumCheck=8;
  p.maxNumCheck=32; p.checkTimeSec=.05;
  p.maxVelSqr=142.8025; p.maxOmgXYSqr=14.74348422; p.maxOmgZSqr=12.18469679;
  p.thrMean=4.27429; p.thrRadi=4.27429; p.thrRadiSqr=p.thrRadi*p.thrRadi;
  p.collectivtThrMean=17.09716; p.collectivtThrRadi=17.09716;
  p.collectivtThrRadiSqr=p.collectivtThrRadi*p.collectivtThrRadi;
  p.boundX<<-10000,10000; p.boundY<<-10000,10000; p.boundZ<<-10000,10000;
}
drolib::QuadParams quad_params() {
  drolib::QuadParams p; p.name="PX4_Gazebo_x500"; p.mass=2.0643076923076924;
  p.inertia<<0.02383948,0.02394241,0.04399995;
  p.T_bm.resize(4,4);
  p.T_bm<<1,1,1,1,
          -.174,.174,.174,-.174,
          -.174,.174,-.174,.174,
          -.016,-.016,.016,.016;
  p.T_mb=p.T_bm.inverse(); return p;
}
void beta(double t, Eigen::Matrix<double,8,6> &b) {
  b.setZero(); double pw[8]={1}; for(int k=1;k<8;k++) pw[k]=pw[k-1]*t;
  for(int r=0;r<6;r++) for(int k=r;k<8;k++) { double m=1;
    for(int q=0;q<r;q++) m*=k-q; b(k,r)=m*pw[k-r]; }
}
double integrate(const Eigen::VectorXd &T,const Eigen::MatrixX3d &C,
                 const drolib::QuadManifold &quad,const drolib::TrajParams &p,
                 Eigen::MatrixX3d &gC,Eigen::VectorXd &gT) {
  double cost=0; drolib::ConstAngle yaw(0); Eigen::Matrix<double,8,6> b;
  for(int i=0;i<T.size();i++) { auto c=C.block<8,3>(8*i,0);
    int M=std::min(std::max(int(T(i)/p.checkTimeSec),p.minNumCheck),p.maxNumCheck);
    double frac=1.0/M, step=T(i)*frac;
    for(int j=0;j<=M;j++) { double t=j*step; beta(t,b);
      Eigen::Vector3d x[6]; for(int r=0;r<6;r++) x[r]=c.transpose()*b.col(r);
      drolib::PVAJS pv; pv<<x[0],x[1],x[2],x[3],x[4];
      Eigen::Vector3d g[5]; double pen=quad.computeRobustPenalityCost(
        pv,yaw.at(t),p,g[0],g[1],g[2],g[3],g[4]);
      double w=(j==0||j==M)?.5:1;
      for(int r=0;r<5;r++) gC.block<8,3>(8*i,0)+=b.col(r)*g[r].transpose()*step*w;
      double chain=0; for(int r=0;r<5;r++) chain+=g[r].dot(x[r+1]);
      gT(i)+=chain*step*w*(j*frac)+w*frac*pen; cost+=w*step*pen;
    }
  } return cost;
}
}

extern "C" int togt_objective_gradient(int n,const double *h0,const double *h1,
 const double *pr,const double *tr,double *cost,double *gpr,double *gtr,
 char *error,int capacity) {
 try {
  if(n<1) throw std::runtime_error("invalid piece count");
  Eigen::Matrix<double,3,4> head,tail;
  for(int r=0;r<3;r++) for(int c=0;c<4;c++) {head(r,c)=h0[4*r+c];tail(r,c)=h1[4*r+c];}
  Eigen::VectorXd T(n); for(int i=0;i<n;i++){T(i)=tr[i];if(!(T(i)>0))throw std::runtime_error("invalid duration");}
  Eigen::Matrix3Xd P(3,n-1); for(int i=0;i<n-1;i++)for(int a=0;a<3;a++)P(a,i)=pr[3*i+a];
  drolib::MincoSnap minco; minco.setConditions(head,tail,n); minco.setParameters(P,T);
  drolib::TrajParams params; standard(params); drolib::QuadManifold quad(quad_params());
  Eigen::MatrixX3d gC=Eigen::MatrixX3d::Zero(8*n,3); Eigen::VectorXd gt0=Eigen::VectorXd::Zero(n);
  *cost=integrate(T,minco.getCoeffs(),quad,params,gC,gt0)+T.sum();
  Eigen::Matrix3Xd gp(3,n-1); Eigen::VectorXd gt(n); minco.propagateGrad(gC,gt0,gp,gt); gt.array()+=1;
  for(int i=0;i<n-1;i++)for(int a=0;a<3;a++)gpr[3*i+a]=gp(a,i);
  for(int i=0;i<n;i++)gtr[i]=gt(i); return 0;
 } catch(const std::exception &e) {if(error&&capacity){std::strncpy(error,e.what(),capacity-1);error[capacity-1]=0;}return 1;}
}

extern "C" int minco_propagate_gradient(
 const int n,const double *h0,const double *h1,const double *pr,const double *tr,
 const double *partial_coeffs,const double *partial_times,
 double *gpr,double *gtr,char *error,const int capacity) {
 try {
  if(n<1) throw std::runtime_error("invalid piece count");
  Eigen::Matrix<double,3,4> head,tail;
  for(int r=0;r<3;r++) for(int c=0;c<4;c++) {
    head(r,c)=h0[4*r+c]; tail(r,c)=h1[4*r+c];
  }
  Eigen::VectorXd T(n);
  for(int i=0;i<n;i++) {
    T(i)=tr[i]; if(!(T(i)>0)) throw std::runtime_error("invalid duration");
  }
  Eigen::Matrix3Xd P(3,n-1);
  for(int i=0;i<n-1;i++) for(int a=0;a<3;a++) P(a,i)=pr[3*i+a];
  drolib::MincoSnap minco;
  minco.setConditions(head,tail,n); minco.setParameters(P,T);
  Eigen::MatrixX3d gC(8*n,3);
  for(int i=0;i<8*n;i++) for(int a=0;a<3;a++) gC(i,a)=partial_coeffs[3*i+a];
  Eigen::VectorXd gt0(n);
  for(int i=0;i<n;i++) gt0(i)=partial_times[i];
  Eigen::Matrix3Xd gp(3,n-1); Eigen::VectorXd gt(n);
  minco.propagateGrad(gC,gt0,gp,gt);
  for(int i=0;i<n-1;i++) for(int a=0;a<3;a++) gpr[3*i+a]=gp(a,i);
  for(int i=0;i<n;i++) gtr[i]=gt(i);
  return 0;
 } catch(const std::exception &e) {
  if(error&&capacity) {std::strncpy(error,e.what(),capacity-1);error[capacity-1]=0;}
  return 1;
 }
}

extern "C" int togt_sample_dynamics(
 const int count,const double *pvajs,double *output,char *error,const int capacity) {
 try {
  if(count<1) throw std::runtime_error("invalid sample count");
  drolib::TrajParams params; standard(params);
  drolib::QuadManifold quad(quad_params());
  drolib::ConstAngle yaw_profile(0.0);
  const Eigen::Vector3d yaw=yaw_profile.at(0.0);
  for(int sample=0;sample<count;sample++) {
    drolib::PVAJS state;
    for(int derivative=0;derivative<5;derivative++) {
      for(int axis=0;axis<3;axis++) {
        state(axis,derivative)=pvajs[15*sample+3*derivative+axis];
      }
    }
    const Eigen::Vector3d alpha=state.col(2)+Eigen::Vector3d(0.0,0.0,9.8066);
    const double alpha_norm=alpha.norm();
    if(!(alpha_norm>0.0) || !std::isfinite(alpha_norm)) {
      throw std::runtime_error("invalid specific force in dynamics audit");
    }
    const Eigen::Vector3d body_z=alpha/alpha_norm;
    const bool regular=std::abs(body_z.z()+1.0)>0.001;
    double *row=output+12*sample;
    row[0]=state.col(1).norm();
    row[1]=std::acos(std::clamp(body_z.z(),-1.0,1.0));
    row[2]=row[3]=row[4]=row[6]=row[7]=row[8]=row[9]=NAN;
    row[5]=alpha_norm;
    row[10]=regular?1.0:0.0;
    if(regular) {
      drolib::Setpoint setpoint;
      if(!quad.toStateWithTiltYaw(0.0,state,yaw,setpoint)) {
        throw std::runtime_error("released QuadManifold state conversion failed");
      }
      row[2]=setpoint.input.omega.x();
      row[3]=setpoint.input.omega.y();
      row[4]=setpoint.input.omega.z();
      row[5]=setpoint.input.collective_thrust;
      for(int rotor=0;rotor<4;rotor++) row[6+rotor]=setpoint.input.thrusts(rotor);
    } else {
      const double c=alpha_norm/quad_params().mass;
      row[2]=state(0,3)/c;
      row[3]=state(1,3)/c;
    }
    Eigen::Vector3d gradients[5];
    row[11]=quad.computeRobustPenalityCost(
      state,yaw,params,gradients[0],gradients[1],gradients[2],gradients[3],gradients[4]);
    const int always_finite[]={0,1,2,3,5,10,11};
    for(const int column:always_finite) if(!std::isfinite(row[column])) {
      throw std::runtime_error("non-finite dynamics audit output");
    }
    if(regular) for(int column=4;column<=9;column++) if(!std::isfinite(row[column])) {
      throw std::runtime_error("non-finite regular-branch dynamics output");
    }
  }
  return 0;
 } catch(const std::exception &e) {
  if(error&&capacity) {std::strncpy(error,e.what(),capacity-1);error[capacity-1]=0;}
  return 1;
 }
}
