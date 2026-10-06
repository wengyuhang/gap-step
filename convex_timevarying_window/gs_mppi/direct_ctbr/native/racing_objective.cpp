// Private reference objective: never changes the historical TOGT library.
#include "../../../../convex_dynamic_seven_window_gazebo/x500_togt/native/togt_x500.cpp"

namespace {
double racing_integral(const Eigen::VectorXd &T,const Eigen::MatrixX3d &C,
                       const drolib::QuadManifold &quad,drolib::TrajParams &p,
                       Eigen::MatrixX3d &gC,Eigen::VectorXd &gT) {
  // Reference preferences include headroom; execution's hard limits stay
  // 12 m/s, 8 m/s^2 and 42 degrees. These penalties are NOT safety admission.
  p.weightVel=1000.;p.maxVelNorm=11.5;p.maxVelSqr=11.5*11.5;
  p.maxTiltedAngle=40.*M_PI/180.;
  double cost=integrate(T,C,quad,p,gC,gT);
  Eigen::Matrix<double,8,6> b;
  for(int i=0;i<T.size();i++) {
    auto c=C.block<8,3>(8*i,0);
    // Fixed count avoids a changing quadrature grid in analytic derivatives.
    constexpr int M=24;double step=T(i)/M;
    for(int j=0;j<=M;j++) {
      double fraction=double(j)/M,t=T(i)*fraction,w=(j==0||j==M)?.5:1.;beta(t,b);
      Eigen::Vector3d a=c.transpose()*b.col(2),jerk=c.transpose()*b.col(3),snap=c.transpose()*b.col(4);
      Eigen::Vector3d demand=a+.15*jerk,ga=Eigen::Vector3d::Zero(),gj=.0012*jerk;
      double penalty=.0006*jerk.squaredNorm();
      // Cubic normalized violations, including attitude/motor response lead.
      for(int kind=0;kind<2;kind++) {
        const Eigen::Vector3d y=kind?demand:a;
        double violation=y.squaredNorm()/(7.5*7.5)-1.;
        if(violation>0) {
          penalty+=1000.*violation*violation*violation;
          Eigen::Vector3d gradient=6000.*violation*violation*y/(7.5*7.5);
          ga+=gradient;if(kind)gj+=.15*gradient;
        }
      }
      gC.block<8,3>(8*i,0)+=step*w*(b.col(2)*ga.transpose()+b.col(3)*gj.transpose());
      gT(i)+=w*penalty/M+step*w*fraction*(ga.dot(jerk)+gj.dot(snap));
      cost+=step*w*penalty;
    }
  }
  return cost;
}
}

extern "C" int racing_objective(int n,const double *h0,const double *h1,
 const double *pr,const double *tr,double *cost,double *gpr,double *gtr,
 double *gtail,char *error,int capacity,int gate_count,const double *planes,int lead_piece) {
 try {
  if(n<1)throw std::runtime_error("invalid piece count");
  Eigen::Matrix<double,3,4> head,tail;
  for(int r=0;r<3;r++)for(int c=0;c<4;c++){head(r,c)=h0[4*r+c];tail(r,c)=h1[4*r+c];}
  Eigen::VectorXd T(n);for(int i=0;i<n;i++){T(i)=tr[i];if(!(T(i)>0))throw std::runtime_error("invalid duration");}
  Eigen::Matrix3Xd P(3,n-1);for(int i=0;i<n-1;i++)for(int a=0;a<3;a++)P(a,i)=pr[3*i+a];
  drolib::MincoSnap minco;minco.setConditions(head,tail,n);minco.setParameters(P,T);
  drolib::TrajParams params;standard(params);drolib::QuadManifold quad(quad_params());
  // Fixed base quadrature too, so cost and gradient use the same discretization.
  params.minNumCheck=params.maxNumCheck=24;
  Eigen::MatrixX3d gC=Eigen::MatrixX3d::Zero(8*n,3);Eigen::VectorXd gt0=Eigen::VectorXd::Zero(n);
  *cost=racing_integral(T,minco.getCoeffs(),quad,params,gC,gt0)+T.sum();
  // Direction at a free crossing point is not implied by point-on-plane.
  // Penalize reverse/tangential relative passage, with known plane motion.
  for(int i=lead_piece;i<std::min(n,gate_count+lead_piece);i++){
   const double *data=planes+19*(i-lead_piece);
   Eigen::Map<const Eigen::Vector3d> normal(data),ndot(data+3),nddot(data+6),center(data+9),dc(data+12),ddc(data+15);
   double sign=data[18];Eigen::Matrix<double,8,6> b;beta(T(i),b);
   auto c=minco.getCoeffs().block<8,3>(8*i,0);
   Eigen::Vector3d position=c.transpose()*b.col(0),velocity=c.transpose()*b.col(1),acceleration=c.transpose()*b.col(2);
   double crossing_rate=sign*(normal.dot(velocity-dc)+ndot.dot(position-center));
   double violation=(2.-crossing_rate)/2.;
   if(violation<=0)continue;
   *cost+=1000.*violation*violation*violation;
   double scale=-1500.*violation*violation*sign;
   Eigen::Vector3d gp=scale*ndot,gv=scale*normal;
   gC.block<8,3>(8*i,0)+=b.col(0)*gp.transpose()+b.col(1)*gv.transpose();
   double clock_partial=scale*(ndot.dot(velocity-2.*dc)-normal.dot(ddc)+nddot.dot(position-center));
   for(int j=0;j<=i;j++)gt0(j)+=clock_partial;
   gt0(i)+=gp.dot(velocity)+gv.dot(acceleration);
  }
  Eigen::Matrix3Xd gp(3,n-1);Eigen::VectorXd gt(n);Eigen::Matrix<double,3,4> tail_gradient;
  minco.propagateGrad(gC,gt0,gp,gt,&tail_gradient);gt.array()+=1.;
  for(int i=0;i<n-1;i++)for(int a=0;a<3;a++)gpr[3*i+a]=gp(a,i);
  for(int i=0;i<n;i++)gtr[i]=gt(i);
  for(int r=0;r<3;r++)for(int c=0;c<4;c++)gtail[4*r+c]=tail_gradient(r,c);
  return 0;
 }catch(const std::exception &e){if(error&&capacity){std::strncpy(error,e.what(),capacity-1);error[capacity-1]=0;}return 1;}
}
