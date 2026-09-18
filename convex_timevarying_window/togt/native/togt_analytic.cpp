#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <stdexcept>
#include "drolib/planner/angle.hpp"
#include "drolib/planner/traj_params.hpp"
#include "drolib/solver/lbfgs.hpp"
#include "drolib/solver/minco_snap.hpp"
#include "drolib/system/quadrotor_manifold.hpp"

namespace {
using raw_lbfgs_evaluate_t = double (*)(void *, int, const double *, double *);
struct RawLbfgsContext { raw_lbfgs_evaluate_t evaluate; void *instance; int iterations = 0; };
double rawLbfgsEvaluate(void *opaque, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
  auto *ctx = static_cast<RawLbfgsContext *>(opaque);
  return ctx->evaluate(ctx->instance, x.size(), x.data(), g.data());
}
int rawLbfgsProgress(void *opaque, const Eigen::VectorXd &, const Eigen::VectorXd &,
                     const double, const double, const int iteration, const int) {
  static_cast<RawLbfgsContext *>(opaque)->iterations = iteration;
  return 0;
}
void standard(drolib::TrajParams &p, const double penalty_scale = 1.0) {
  if (!(penalty_scale > 0.0) || !std::isfinite(penalty_scale)) {
    throw std::runtime_error("penalty scale must be finite and positive");
  }
  p.piecesPerSegment=13; p.speedGuess=1; p.maxVelNorm=60; p.maxOmgXY=10;
  p.maxOmgZ=10; p.maxTiltedAngle=6.28; p.maxThr=5; p.minThr=.25;
  p.weightTime=1; p.weightEnergy=0; p.weightPos=0; p.weightVel=0;
  p.weightOmg=penalty_scale; p.weightRot=penalty_scale;
  p.weightThr=penalty_scale; p.smoothingEps=.01;
  p.numConstPena=16; p.dynamicConstCheck=true; p.minNumCheck=8;
  p.maxNumCheck=32; p.checkTimeSec=.05;
  p.maxVelSqr=3600; p.maxOmgXYSqr=100; p.maxOmgZSqr=100;
  p.thrMean=2.625; p.thrRadi=2.375; p.thrRadiSqr=p.thrRadi*p.thrRadi;
  p.collectivtThrMean=10.5; p.collectivtThrRadi=9.5;
  p.collectivtThrRadiSqr=90.25;
  p.boundX<<-10000,10000; p.boundY<<-10000,10000; p.boundZ<<-10000,10000;
}

extern "C" int togt_released_lbfgs(
    int n, double *x, double *cost, raw_lbfgs_evaluate_t evaluate, void *instance,
    int memory, int past, double min_step, int max_linesearch, int max_iterations,
    double rel_cost_tolerance, double rel_grad_tolerance, int *iterations) {
  try {
    if (n <= 0 || !x || !cost || !evaluate || !iterations) return -1;
    Eigen::VectorXd values(n); for (int i=0;i<n;i++) values(i)=x[i];
    drolib::lbfgs_parameter_t params;
    params.mem_size=memory; params.past=past; params.min_step=min_step;
    params.max_linesearch=max_linesearch; params.max_iterations=max_iterations;
    params.delta=rel_cost_tolerance; params.g_epsilon=rel_grad_tolerance;
    RawLbfgsContext context{evaluate, instance, 0};
    double value=0.0;
    const int status=drolib::lbfgs_optimize(values, value, rawLbfgsEvaluate,
                                              nullptr, rawLbfgsProgress, &context, params);
    for (int i=0;i<n;i++) x[i]=values(i);
    *cost=value; *iterations=context.iterations;
    return status;
  } catch (...) { return -2; }
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
    // A duration-dependent integer quadrature count makes the dynamic-window
    // objective discontinuous whenever T crosses a sampling threshold.  Use
    // the released maximum density for every candidate in both compared
    // methods, keeping the objective differentiable while changing neither
    // the penalty formula nor the L-BFGS stopping conditions.
    int M=p.maxNumCheck;
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

int objective_gradient_impl(int n,const double *h0,const double *h1,
 const double *pr,const double *tr,double *cost,double *gpr,double *gtr,
 const double penalty_scale, char *error,int capacity) {
 try {
  if(n<1) throw std::runtime_error("invalid piece count");
  Eigen::Matrix<double,3,4> head,tail;
  for(int r=0;r<3;r++) for(int c=0;c<4;c++) {head(r,c)=h0[4*r+c];tail(r,c)=h1[4*r+c];}
  Eigen::VectorXd T(n); for(int i=0;i<n;i++){T(i)=tr[i];if(!(T(i)>0))throw std::runtime_error("invalid duration");}
  Eigen::Matrix3Xd P(3,n-1); for(int i=0;i<n-1;i++)for(int a=0;a<3;a++)P(a,i)=pr[3*i+a];
  drolib::MincoSnap minco; minco.setConditions(head,tail,n); minco.setParameters(P,T);
  drolib::TrajParams params; standard(params, penalty_scale); drolib::QuadManifold quad(quad_params());
  Eigen::MatrixX3d gC=Eigen::MatrixX3d::Zero(8*n,3); Eigen::VectorXd gt0=Eigen::VectorXd::Zero(n);
  *cost=integrate(T,minco.getCoeffs(),quad,params,gC,gt0)+T.sum();
  Eigen::Matrix3Xd gp(3,n-1); Eigen::VectorXd gt(n); minco.propagateGrad(gC,gt0,gp,gt); gt.array()+=1;
  for(int i=0;i<n-1;i++)for(int a=0;a<3;a++)gpr[3*i+a]=gp(a,i);
  for(int i=0;i<n;i++)gtr[i]=gt(i); return 0;
 } catch(const std::exception &e) {if(error&&capacity){std::strncpy(error,e.what(),capacity-1);error[capacity-1]=0;}return 1;}
}

extern "C" int togt_objective_gradient(int n,const double *h0,const double *h1,
 const double *pr,const double *tr,double *cost,double *gpr,double *gtr,
 char *error,int capacity) {
  return objective_gradient_impl(n,h0,h1,pr,tr,cost,gpr,gtr,1.0,error,capacity);
}

extern "C" int togt_objective_gradient_weighted(int n,const double *h0,const double *h1,
 const double *pr,const double *tr,double *cost,double *gpr,double *gtr,
 const double penalty_scale, char *error,int capacity) {
  return objective_gradient_impl(n,h0,h1,pr,tr,cost,gpr,gtr,penalty_scale,error,capacity);
}

// Same released MINCO/dynamics objective, additionally exposing the adjoint
// with respect to the terminal PVAJ.  This is required by the rolling
// right-boundary formulation; it does not alter the standard entry points.
extern "C" int togt_objective_gradient_tail(
 const int n,const double *h0,const double *h1,const double *pr,const double *tr,
 double *cost,double *gpr,double *gtr,double *gtail,char *error,const int capacity) {
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
  Eigen::Matrix3Xd gp(3,n-1); Eigen::VectorXd gt(n); Eigen::Matrix<double,3,4> gtail_mat;
  minco.propagateGrad(gC,gt0,gp,gt,&gtail_mat); gt.array()+=1;
  for(int i=0;i<n-1;i++)for(int a=0;a<3;a++)gpr[3*i+a]=gp(a,i);
  for(int i=0;i<n;i++)gtr[i]=gt(i);
  for(int r=0;r<3;r++)for(int c=0;c<4;c++)gtail[4*r+c]=gtail_mat(r,c);
  return 0;
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
