// Reuse the x500 objective, adding its already-supported MINCO tail adjoint.
// No change to the original baseline library or parameters.
#include "../../../convex_dynamic_seven_window_gazebo/x500_togt/native/togt_x500.cpp"

extern "C" int v3_objective_tail(int n,const double *h0,const double *h1,
 const double *pr,const double *tr,double *cost,double *gpr,double *gtr,
 double *gtail,char *error,int capacity) {
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
  Eigen::Matrix3Xd gp(3,n-1); Eigen::VectorXd gt(n); Eigen::Matrix<double,3,4> gt_end;
  minco.propagateGrad(gC,gt0,gp,gt,&gt_end); gt.array()+=1;
  for(int i=0;i<n-1;i++)for(int a=0;a<3;a++)gpr[3*i+a]=gp(a,i);
  for(int i=0;i<n;i++)gtr[i]=gt(i);
  for(int r=0;r<3;r++)for(int c=0;c<4;c++)gtail[4*r+c]=gt_end(r,c);
  return 0;
 } catch(const std::exception &e) {
  if(error&&capacity){std::strncpy(error,e.what(),capacity-1);error[capacity-1]=0;}
  return 1;
 }
}
