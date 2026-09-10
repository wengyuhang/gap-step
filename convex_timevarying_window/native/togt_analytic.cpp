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
  p.piecesPerSegment=13; p.speedGuess=1; p.maxVelNorm=60; p.maxOmgXY=10;
  p.maxOmgZ=10; p.maxTiltedAngle=6.28; p.maxThr=5; p.minThr=.25;
  p.weightTime=1; p.weightEnergy=0; p.weightPos=0; p.weightVel=0;
  p.weightOmg=1; p.weightRot=1; p.weightThr=1; p.smoothingEps=.01;
  p.numConstPena=16; p.dynamicConstCheck=true; p.minNumCheck=8;
  p.maxNumCheck=32; p.checkTimeSec=.05;
  p.maxVelSqr=3600; p.maxOmgXYSqr=100; p.maxOmgZSqr=100;
  p.thrMean=2.625; p.thrRadi=2.375; p.thrRadiSqr=p.thrRadi*p.thrRadi;
  p.collectivtThrMean=10.5; p.collectivtThrRadi=9.5;
  p.collectivtThrRadiSqr=90.25;
  p.boundX<<-10000,10000; p.boundY<<-10000,10000; p.boundZ<<-10000,10000;
}
drolib::QuadParams quad_params() {
  drolib::QuadParams p; p.name="QuadA"; p.mass=1; p.inertia<<.005,.005,.01;
  p.T_bm.resize(4,4);
  p.T_bm<<1,1,1,1, .15,-.15,-.15,.15, -.15,-.15,.15,.15, .01,-.01,.01,-.01;
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
