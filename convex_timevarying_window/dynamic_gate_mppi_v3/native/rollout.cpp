// ENU / FLU; state p,v,q(wxyz),body rate, four actual motor speeds (rad/s).
// Inputs are desired rotor thrust (N), NOT thrust derivatives or body rates.
#include <algorithm>
#include <cmath>
#include <cstring>
#ifndef V3_CONTROLS
#define V3_CONTROLS 50
#endif
#ifndef V3_CTBR
#define V3_CTBR 0
#endif
namespace v3 {
constexpr double mass=2.0643076923076924,g=9.8066,kf=8.54858e-6,maxf=8.54858;
constexpr double I[3]={.02383948,.02394241,.04399995};
constexpr double mix[3][4]={{-.174,.174,.174,-.174},{-.174,.174,-.174,.174},{-.016,-.016,.016,.016}};
double sq(double x){return x*x;}
void rotation(const double *q,double *r){
 double w=q[0],x=q[1],y=q[2],z=q[3];
 r[0]=1-2*(y*y+z*z);r[1]=2*(x*y-w*z);r[2]=2*(x*z+w*y);
 r[3]=2*(x*y+w*z);r[4]=1-2*(x*x+z*z);r[5]=2*(y*z-w*x);
 r[6]=2*(x*z-w*y);r[7]=2*(y*z+w*x);r[8]=1-2*(x*x+y*y);
}
void derivative(const double *s,const double *u,double *d){
 double r[9];rotation(s+6,r);double thrust=0,tau[3]={},drag=0;
 for(int j=0;j<4;j++){
  double omega=std::clamp(s[13+j],0.,1000.),f=kf*omega*omega;
  double command=std::sqrt(std::clamp(V3_CTBR?u[0]/4.:u[j],kf*150.*150.,maxf)/kf);
  d[13+j]=(command-omega)/(command>omega?.0125:.025);
  thrust+=f;drag+=8.06428e-5*omega/mass;
  for(int a=0;a<3;a++)tau[a]+=mix[a][j]*f;
 }
 double axial=0;for(int a=0;a<3;a++)axial+=s[3+a]*r[3*a+2];
 for(int a=0;a<3;a++){
  d[a]=s[3+a];d[3+a]=r[3*a+2]*thrust/mass-(a==2?g:0.)
      -drag*(s[3+a]-axial*r[3*a+2]);
 }
 double w=s[6],x=s[7],y=s[8],z=s[9],p=s[10],q=s[11],rr=s[12];
 d[6]=-.5*(x*p+y*q+z*rr);d[7]=.5*(w*p+y*rr-z*q);
 d[8]=.5*(w*q+z*p-x*rr);d[9]=.5*(w*rr+x*q-y*p);
 d[10]=(tau[0]-(I[2]-I[1])*q*rr)/I[0];
 d[11]=(tau[1]-(I[0]-I[2])*rr*p)/I[1];
 d[12]=(tau[2]-(I[1]-I[0])*p*q)/I[2];
#if V3_CTBR
 // Empirical rate-loop response, NOT a claim to model PX4 allocation exactly.
 // Empirical CTBR response: probe + development trace identification,
 // separately checked on _08. Not a certified inner-loop model.
 for(int a=0;a<3;a++)d[10+a]=(u[1+a]-s[10+a])/(a==0?.09:(a==1?.06:.45));
#endif
}
void step(double *s,const double *u,double h){
 double a[17],b[17],c[17],d[17],tmp[17];
 derivative(s,u,a);for(int j=0;j<17;j++)tmp[j]=s[j]+h*.5*a[j];
 derivative(tmp,u,b);for(int j=0;j<17;j++)tmp[j]=s[j]+h*.5*b[j];
 derivative(tmp,u,c);for(int j=0;j<17;j++)tmp[j]=s[j]+h*c[j];
 derivative(tmp,u,d);for(int j=0;j<17;j++)s[j]+=h*(a[j]+2*b[j]+2*c[j]+d[j])/6;
 double n=0;for(int j=6;j<10;j++)n+=sq(s[j]);n=std::sqrt(n);
 for(int j=6;j<10;j++)s[j]/=n;
}
void local(const double *p,const double *pose,double *q){
 for(int j=0;j<3;j++){q[j]=0;for(int a=0;a<3;a++)q[j]+=pose[3+3*a+j]*(p[a]-pose[a]);}
}
// Exact distance to expanded centreline: circle or convex polygon.
double distance(const double *q,const double *geom){
 double ds=1e30;
 if(geom[0]==0)ds=sq(std::hypot(q[0],q[1])-geom[1]);
 else for(int j=0;j<int(geom[2]);j++){
  int n=(j+1)%int(geom[2]);double ax=geom[3+2*j],ay=geom[4+2*j];
  double ex=geom[3+2*n]-ax,ey=geom[4+2*n]-ay;
  double t=std::clamp(((q[0]-ax)*ex+(q[1]-ay)*ey)/(ex*ex+ey*ey),0.,1.);
  ds=std::min(ds,sq(q[0]-ax-t*ex)+sq(q[1]-ay-t*ey));
 }
 return std::sqrt(ds+q[2]*q[2]);
}
bool inside(const double *q,const double *geom,double margin){
 if(geom[0]==0)return std::hypot(q[0],q[1])+margin<geom[1];
 double area=0;for(int j=0;j<int(geom[2]);j++){
  int n=(j+1)%int(geom[2]);area+=geom[3+2*j]*geom[4+2*n]-geom[4+2*j]*geom[3+2*n];
 }
 for(int j=0;j<int(geom[2]);j++){
  int n=(j+1)%int(geom[2]);double ex=geom[3+2*n]-geom[3+2*j],ey=geom[4+2*n]-geom[4+2*j];
  double cross=ex*(q[1]-geom[4+2*j])-ey*(q[0]-geom[3+2*j]);
  if(cross*(area>=0?1:-1)<=margin*std::hypot(ex,ey))return false;
 }return true;
}
}

extern "C" void v3_integrate(double *s,const double *u,double dt,int substeps){
 for(int i=0;i<substeps;i++)v3::step(s,u,dt/substeps);
}

// Feedback only constructs candidates. No candidate bypasses hard admission.
namespace v3 {
void feedback(double *s,const double *pd,const double *vd,const double *ad,
              double gain,const double *cfg,double *prev,double *u){
 double acc[3],R[9];rotation(s+6,R);
 for(int a=0;a<3;a++)acc[a]=ad[a]+gain*(pd[a]-s[a])+2.*std::sqrt(gain)*(vd[a]-s[3+a]);
 double norm=std::sqrt(sq(acc[0])+sq(acc[1])+sq(acc[2]));
 if(norm>cfg[4])for(double &v:acc)v*=cfg[4]/norm;
 double drag=0.,axial=0.;
 for(int j=0;j<4;j++)drag+=8.06428e-5*s[13+j]/mass;
 for(int a=0;a<3;a++)axial+=s[3+a]*R[3*a+2];
 for(int a=0;a<3;a++)acc[a]+=drag*(s[3+a]-axial*R[3*a+2]);
 acc[2]+=g;
 double horizontal=std::hypot(acc[0],acc[1]);
 double horizontal_limit=.95*std::max(0.,acc[2])*std::sqrt(1.-cfg[5]*cfg[5])/cfg[5];
 if(horizontal>horizontal_limit)for(int a=0;a<2;a++)acc[a]*=horizontal_limit/std::max(horizontal,1e-9);
 norm=std::sqrt(sq(acc[0])+sq(acc[1])+sq(acc[2]));
 double zd[3];for(int a=0;a<3;a++)zd[a]=acc[a]/norm;
 double yd[3]={0.,zd[2],-zd[1]};double yn=std::hypot(yd[1],yd[2]);
#if V3_CTBR
 // Heading is not a gate constraint. Preserve measured heading rather
 // than commanding a 90-degree yaw transient at PX4 handover.
 double hn=std::max(1e-9,std::hypot(R[0],R[3]));
 double hx=R[0]/hn,hy=R[3]/hn;
 yd[0]=-zd[2]*hy;yd[1]=zd[2]*hx;yd[2]=zd[0]*hy-zd[1]*hx;
 yn=std::sqrt(sq(yd[0])+sq(yd[1])+sq(yd[2]));
#endif
 for(double &v:yd)v/=std::max(yn,1e-9);
 double xd[3]={yd[1]*zd[2]-yd[2]*zd[1],yd[2]*zd[0]-yd[0]*zd[2],yd[0]*zd[1]-yd[1]*zd[0]};
 double rd[9];for(int a=0;a<3;a++){rd[3*a]=xd[a];rd[3*a+1]=yd[a];rd[3*a+2]=zd[a];}
 double E[9]={};for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int a=0;a<3;a++)E[3*i+j]+=rd[3*a+i]*R[3*a+j];
 double err[3]={.5*(E[7]-E[5]),.5*(E[2]-E[6]),.5*(E[3]-E[1])};
 double torque[3];for(int a=0;a<3;a++)torque[a]=-.7*err[a]-2.*std::sqrt(.7*I[a])*s[10+a];
 torque[0]+=(I[2]-I[1])*s[11]*s[12];
 torque[1]+=(I[0]-I[2])*s[12]*s[10];
 torque[2]+=(I[1]-I[0])*s[10]*s[11];
 double thrust=0;for(int a=0;a<3;a++)thrust+=mass*acc[a]*R[3*a+2];
#if V3_CTBR
 u[0]=std::clamp(thrust,4*kf*150.*150.,4*maxf);
 for(int a=0;a<3;a++)u[1+a]=std::clamp(std::clamp(-5.*err[a],-3.,3.),prev[1+a]-.16,prev[1+a]+.16);
 for(int a=0;a<4;a++)prev[a]=u[a];
#else
 for(int j=0;j<4;j++){
  double f=thrust/4.;for(int a=0;a<3;a++)f+=mix[a][j]*torque[a]/(4*sq(mix[a][j]));
  u[j]=std::clamp(std::clamp(f,prev[j]-1.6,prev[j]+1.6),kf*150.*150.,maxf);prev[j]=u[j];
 }
#endif
}
}
extern "C" void v3_feedback(double *state,const double *pva,const double *cfg,double *out){
 double prev[4]={};
#if V3_CTBR
 for(int a=0;a<3;a++)prev[1+a]=state[10+a];
#endif
 v3::feedback(state,pva,pva+3,pva+6,4.,cfg,prev,out);
}
extern "C" void v3_tracking(int batch,const double *initial,const double *reference,
 const double *cfg,const double *previous,double *out){
 using namespace v3;
 for(int b=0;b<batch;b++){
  double s[17],prev[4];std::memcpy(s,initial,17*sizeof(double));std::memcpy(prev,previous,4*sizeof(double));
  double gain=2.+2.*(b%5);
  for(int k=0;k<V3_CONTROLS;k++){
   const double *r=reference+2*(k+1)*9;double u[4];
   feedback(s,r,r+3,r+6,gain,cfg,prev,u);
   for(int j=0;j<4;j++)out[(b*V3_CONTROLS+k)*4+j]=u[j];
   for(int i=0;i<4;i++)step(s,u,.005);
  }
 }
}
// Autonomous passage shooting: sample crossing speeds, transverse points and
// durations, then follow a local quintic. This is NOT a second TOGT solve.
extern "C" void v3_guided(int batch,int gates,const double *initial,const double *poses,
 const double *signs,const double *cfg,int first,int phase,const double *previous,double *out){
 using namespace v3;
 if(first>=gates)return;
 for(int b=0;b<batch;b++){
  double s[17],prev[4],d0[17];std::memcpy(s,initial,17*sizeof(double));std::memcpy(prev,previous,4*sizeof(double));
  derivative(s,prev,d0);
  const double cruise=2.+2.*(b%5);
  const double qx=((b/5)%3-1)*.25,qy=((b/15)%3-1)*.25;
  const bool retreat=phase && b>=batch/2;
  const double *p0=poses+first*12;double q[3];local(initial,p0,q);
  double exit_z=(retreat?-1.:1.)*(cfg[0]+cfg[1]+cfg[2]+.3);
  double delta0[3],distance0=0.,approach=0.;
  for(int a=0;a<3;a++){
   delta0[a]=p0[a]+p0[3+3*a]*qx+p0[4+3*a]*qy+p0[5+3*a]*signs[first]*exit_z-initial[a];
   distance0+=sq(delta0[a]);
  }
  distance0=std::sqrt(distance0);
  for(int a=0;a<3;a++)approach+=initial[3+a]*delta0[a]/std::max(distance0,1e-9);
  // Time to the FULL exit, with a kinematic acceleration scale. Distance to
  // the centre alone collapses near the gate and creates saturated shots.
  double nominal=std::max(2.*distance0/(std::max(0.,approach)+cruise),
                          std::sqrt(2.*distance0/(.65*cfg[4])));
  double tau=std::max(.35,nominal)*(.75+.25*((b/45)%4));
  tau=std::max(.35,tau);
  int at=std::min(2*V3_CONTROLS-1,int(tau/.01));
  const double *pose=poses+(at*gates+first)*12,*future=pose+gates*12;
  double slab=cfg[0]+cfg[1]+cfg[2],coeff[3][6];
  bool staged=false;double T=tau,entry_depth=slab+.3;
#if V3_CTBR
  staged=cfg[16]>0 && phase==0 && !retreat;
  if(staged){
   entry_depth+=.3*((b/180)%2);
   T=std::max(.15,tau-(entry_depth+slab+.3)/cruise);
   at=std::min(2*V3_CONTROLS-1,int(T/.01));
   pose=poses+(at*gates+first)*12;future=pose+gates*12;
  }
#endif
  double tangent[3]={},tn=0.;
  if(first+1<gates && !retreat){
   const double *next=pose+12;double dot=0.;
   for(int a=0;a<3;a++)dot+=(next[a]-pose[a])*pose[5+3*a]*signs[first];
   for(int a=0;a<3;a++){tangent[a]=next[a]-pose[a]-dot*pose[5+3*a]*signs[first];tn+=sq(tangent[a]);}
   tn=std::sqrt(tn);
  }
  for(int a=0;a<3;a++){
   double n=pose[5+3*a]*signs[first];
   double target=pose[a]+pose[3+3*a]*qx+pose[4+3*a]*qy+n*(staged?-entry_depth:(retreat?-1.:1.)*(slab+.3));
   double speed=(future[a]-pose[a])/.01+n*(retreat?-cruise:cruise)
       +(staged?0.:.5*((b/180)%2)*cruise*tangent[a]/std::max(tn,1e-9));
   if(staged) speed+=((future[3+3*a]-pose[3+3*a])*qx+(future[4+3*a]-pose[4+3*a])*qy
                           -(future[5+3*a]-pose[5+3*a])*signs[first]*entry_depth)/.01;
   double c0=initial[a],c1=initial[3+a],c2=.5*d0[3+a];
   double dp=target-c0-c1*T-c2*T*T,dv=speed-c1-2*c2*T,da=-2*c2;
   coeff[a][0]=c0;coeff[a][1]=c1;coeff[a][2]=c2;
   coeff[a][3]=(10*dp-4*dv*T+.5*da*T*T)/(T*T*T);
   coeff[a][4]=(-15*dp+7*dv*T-da*T*T)/(T*T*T*T);
   coeff[a][5]=(6*dp-3*dv*T+.5*da*T*T)/(T*T*T*T*T);
  }
  bool exited=false;
  for(int k=0;k<V3_CONTROLS;k++){
   double t=std::min((k+1)*.02,T),pd[3],vd[3],ad[3],u[4];
   for(int a=0;a<3;a++){
    double *c=coeff[a];
    pd[a]=c[0]+t*(c[1]+t*(c[2]+t*(c[3]+t*(c[4]+t*c[5]))));
    vd[a]=c[1]+t*(2*c[2]+t*(3*c[3]+t*(4*c[4]+t*5*c[5])));
    ad[a]=2*c[2]+t*(6*c[3]+t*(12*c[4]+t*20*c[5]));
    if((k+1)*.02>T){pd[a]+=((k+1)*.02-T)*vd[a];ad[a]=0.;}
   }
   if(staged && (k+1)*.02>T){
    // Follow a straight line in the known MOVING frame, not a frozen
    // world-space exit point. The entry and exit remain one rollout.
    int node=2*(k+1),last=2*V3_CONTROLS;
    int center=std::min(node,last-1);double points[3][3];
    for(int j=0;j<3;j++){
     int index=center+j-1;const double *moving=poses+(index*gates+first)*12;
     double z=signs[first]*(-entry_depth+cruise*(index*.01-T));
     for(int a=0;a<3;a++)points[j][a]=moving[a]+moving[3+3*a]*qx+moving[4+3*a]*qy+moving[5+3*a]*z;
    }
    for(int a=0;a<3;a++){
     pd[a]=points[node==last?2:1][a];
     vd[a]=node==last?(3*points[2][a]-4*points[1][a]+points[0][a])/.02:(points[2][a]-points[0][a])/.02;
     ad[a]=(points[2][a]-2*points[1][a]+points[0][a])/.0001;
    }
   }
   double current_q[3];local(s,poses+(2*k*gates+first)*12,current_q);
   if(!retreat && signs[first]*current_q[2]>slab)exited=true;
   if(exited){
    // Complete the current gate first, then steer toward the next task in
    // the SAME rollout, rather than blindly extending the exit velocity.
    double distance=0.;
    for(int a=0;a<3;a++){
     if(first+1<gates){
      const double *next=poses+(2*k*gates+first+1)*12;
      pd[a]=next[a]+next[5+3*a]*signs[first+1]*(slab+.3);
     }else pd[a]=cfg[12+a];
     distance+=sq(pd[a]-s[a]);
    }
    distance=std::sqrt(distance);
    double speed=first+1<gates?cruise:std::min(cruise,std::sqrt(2.*.65*cfg[4]*distance));
    for(int a=0;a<3;a++){vd[a]=speed*(pd[a]-s[a])/std::max(distance,1e-9);ad[a]=0.;}
   }
   feedback(s,pd,vd,ad,4.,cfg,prev,u);
   for(int j=0;j<4;j++)out[(b*V3_CONTROLS+k)*4+j]=u[j];
   for(int i=0;i<4;i++)step(s,u,.005);
  }
 }
}

extern "C" void v3_rollout(int batch,int steps,int gates,const double *initial,
 const double *controls,const double *poses,const double *geom,const double *frame_speeds,
 const double *signs,const double *guides,const double *cfg,int first,int crossing,
 double *states,double *clearances,int *valid,int *indices,double *race,int *phases,int *reasons){
 using namespace v3;
 // geometry step .01, control step .02, integration .005; configuration wrapper checks these.
 const double h=.01,acc_bound=4*maxf/mass+g+4*8.06428e-5*1000/mass*cfg[3];
 #pragma omp parallel for num_threads(4) if(batch >= 4)
 for(int b=0;b<batch;b++){
  double s[17];std::memcpy(s,initial,17*sizeof(double));int gate=first,phase=crossing;
  bool newly_entered=false;
  double *path=states+b*(steps+1)*17;std::memcpy(path,s,17*sizeof(double));
  valid[b]=1;reasons[b]=0;race[b]=0;indices[b*(steps+1)]=gate;
  phases[b*(steps+1)]=phase;
  for(int a=0;a<17;a++)if(!std::isfinite(s[a])){valid[b]=0;reasons[b]|=256;}
  for(int a=13;a<17;a++)if(s[a]<0||s[a]>1000.){valid[b]=0;reasons[b]|=64;}
  for(int k=0;k<steps;k++){
   double old[17];std::memcpy(old,s,17*sizeof(double));
   const double *u=controls+(b*(steps/2)+k/2)*4;
#if V3_CTBR
   if(u[0]<4*kf*150.*150.||u[0]>4*maxf||!std::isfinite(u[0])){valid[b]=0;reasons[b]|=64;}
   for(int j=1;j<4;j++)if(std::abs(u[j])>3.839724354||!std::isfinite(u[j])){valid[b]=0;reasons[b]|=64;}
   if(k>=2)for(int j=1;j<4;j++)if(std::abs(u[j]-controls[(b*(steps/2)+k/2-1)*4+j])>.16001){valid[b]=0;reasons[b]|=64;}
#else
   for(int j=0;j<4;j++)if(u[j]<kf*150.*150.||u[j]>maxf||!std::isfinite(u[j])){valid[b]=0;reasons[b]|=64;}
#endif
   for(int sub=0;sub<2;sub++){
    step(s,u,.005);double r[9],d[17];rotation(s+6,r);derivative(s,u,d);
    double speed=std::sqrt(sq(s[3])+sq(s[4])+sq(s[5]));
    double acceleration=std::sqrt(sq(d[3])+sq(d[4])+sq(d[5]));
    if(!std::isfinite(speed))reasons[b]|=256;
    if(speed>cfg[3])reasons[b]|=4;
    if(acceleration>cfg[4])reasons[b]|=8;
    if(r[8]<cfg[5])reasons[b]|=16;
    if(std::hypot(s[10],s[11])>3.839724354||std::abs(s[12])>3.490658504)reasons[b]|=32;
    if(reasons[b])valid[b]=0;
   }
   double speed_bound=std::sqrt(sq(old[3])+sq(old[4])+sq(old[5]))+acc_bound*h;
   double remaining=1e30;
   for(int j=0;j<gates;j++){
    double a[3],c[3];local(old,poses+(k*gates+j)*12,a);local(s,poses+((k+1)*gates+j)*12,c);
    double clearance=std::min(distance(a,geom+j*32),distance(c,geom+j*32))
       -cfg[0]-cfg[1]-cfg[2]-cfg[15]-.5*(speed_bound+frame_speeds[j])*h;
    remaining=std::min(remaining,clearance);
   }
   for(int a=0;a<3;a++){
    double room=std::min(std::min(old[a]-cfg[6+a],s[a]-cfg[6+a]),
                         std::min(cfg[9+a]-old[a],cfg[9+a]-s[a]));
    remaining=std::min(remaining,room-cfg[1]-.5*speed_bound*h);
   }
   clearances[b*steps+k]=remaining;
   if(!(remaining>0)){valid[b]=0;reasons[b]|=1;}
   // Same-time guide: its motion cannot generate progress by itself.
   const double *raw_guide=guides+((k+1)*(gates+1)+gate)*3;
   double guide[3];std::memcpy(guide,raw_guide,3*sizeof(double));
   // No artificial pre-gate holding target in the objective. Entry-side
   // alignment remains a proposal only; legal full passage is hard-filtered.
   double before=0,after=0;for(int a=0;a<3;a++){before+=sq(old[a]-guide[a]);after+=sq(s[a]-guide[a]);}
   race[b]+=std::sqrt(after)-std::sqrt(before);
   if(gate<gates){
    double a[3],c[3];local(old,poses+(k*gates+gate)*12,a);local(s,poses+((k+1)*gates+gate)*12,c);
    const double slab=cfg[0]+cfg[1]+cfg[2];
    const double za=signs[gate]*a[2],zc=signs[gate]*c[2];
#if V3_CTBR
    if(crossing==2 && gate==first && zc<0){valid[b]=0;reasons[b]|=2;}
#endif
    const double margin=cfg[0]+cfg[1]+cfg[15]+.065+.5*(speed_bound+frame_speeds[gate])*h;
#if V3_CTBR
    // Tighten the ENTRY task boundary too, not just frame distance. A
    // nominal path skimming just outside the slab off-aperture can become
    // an illegal entry under tiny tracking error, despite wide clearance.
    // This guard does not change the actual gate-event surface or count.
    if(cfg[15]>0 && phase==0 && zc>-slab-cfg[15] && !inside(c,geom+gate*32,margin)){
      valid[b]=0;reasons[b]|=2;
    }
#endif
    // Guard only the finite approach neighbourhood. A rotating infinite
    // plane must not become a wall elsewhere on a folded course.
    double extent=geom[gate*32]==0?geom[gate*32+1]:0.;
    if(geom[gate*32]!=0)for(int j=0;j<int(geom[gate*32+2]);j++)
     extent=std::max(extent,std::hypot(geom[gate*32+3+2*j],geom[gate*32+4+2*j]));
    double reach=extent+slab+(speed_bound+frame_speeds[gate])*h;
    bool near=std::min(std::sqrt(sq(a[0])+sq(a[1])+sq(a[2])),
                       std::sqrt(sq(c[0])+sq(c[1])+sq(c[2])))<=reach;
    // Cannot join the local passage from the back or side without entry.
    if(phase==0 && za>-slab+1e-9){valid[b]=0;reasons[b]|=2;}
    if(phase==0 && za<=-slab && zc>-slab){phase=1;newly_entered=true;}
    if(phase){
     if(std::min(za,zc)<=slab && std::max(za,zc)>=-slab){
      double lo=0.,hi=1.;
      if(std::abs(zc-za)>1e-12){
       double x=(-slab-za)/(zc-za),y=(slab-za)/(zc-za);
       lo=std::max(0.,std::min(x,y));hi=std::min(1.,std::max(x,y));
      }
      double q0[3],q1[3];for(int j=0;j<3;j++){
       q0[j]=a[j]+lo*(c[j]-a[j]);q1[j]=a[j]+hi*(c[j]-a[j]);
      }
      if(!inside(q0,geom+gate*32,margin)||!inside(q1,geom+gate*32,margin)){valid[b]=0;reasons[b]|=2;}
     }
     if(za<0 && zc>=0)phase=2;
     if(zc<0 && phase==2)phase=1;
     if(zc<=-slab){
      // A flight already inside may retreat. A new admission must have a
      // forward-exit witness, not enter then back out within the horizon.
      if(newly_entered){valid[b]=0;reasons[b]|=2;}
      phase=0;newly_entered=false;
     }else if(phase==2 && zc>slab){
      gate++;phase=0;newly_entered=false;race[b]-=5.*(1.-(k+1.)/steps);
     }
    }
   }
   std::memcpy(path+(k+1)*17,s,17*sizeof(double));indices[b*(steps+1)+k+1]=gate;
   phases[b*(steps+1)+k+1]=phase;
  }
  // Admission needs a COMPLETE exit witness. A still-in-gate prediction is
  // not executable merely because its prefix has positive clearance.
  if(phase!=0){valid[b]=0;reasons[b]|=2;}
  for(int a=0;a<3;a++){
   double lower_speed=std::max(0.,-s[3+a]),upper_speed=std::max(0.,s[3+a]);
   double lower_stop=sq(lower_speed)/(2.*.65*cfg[4])+.15*lower_speed;
   double upper_stop=sq(upper_speed)/(2.*.65*cfg[4])+.15*upper_speed;
   if(s[a]-cfg[6+a]-cfg[1]<lower_stop || cfg[9+a]-s[a]-cfg[1]<upper_stop){valid[b]=0;reasons[b]|=128;}
  }
  if(gate<gates && phase==0){
   // Terminal entry-side viability proxy: preserve enough normal braking
   // distance when a full passage has NOT yet been demonstrated. This is
   // not a command to stop, nor a formal quadrotor invariant-set proof.
   const double *pose=poses+(steps*gates+gate)*12;
   const double *before_pose=pose-gates*12;
   double q[3],before_q[3],previous_p[3];local(s,pose,q);
   for(int a=0;a<3;a++)previous_p[a]=s[a]-.01*s[3+a];
   local(previous_p,before_pose,before_q);
   double closing=std::max(0.,signs[gate]*(q[2]-before_q[2])/.01);
   double available=-signs[gate]*q[2]-(cfg[0]+cfg[1]+cfg[2]);
   double stopping=closing*closing/(2.*.65*cfg[4])+.15*closing;
   if(available<stopping){valid[b]=0;reasons[b]|=128;}
  }
  // Terminal cost-to-go proxy: transverse kinetic energy is distance that
  // must be spent braking before passage. Do not reward fast orbiting.
  if(gate<gates){
   const double *target=guides+(steps*(gates+1)+gate)*3;
   double delta[3],length=0.,along=0.,speed2=0.;
   for(int a=0;a<3;a++){delta[a]=target[a]-s[a];length+=sq(delta[a]);speed2+=sq(s[3+a]);}
   length=std::sqrt(length);
   for(int a=0;a<3;a++)along+=s[3+a]*delta[a]/std::max(length,1e-9);
   race[b]+=(speed2-sq(std::max(0.,along)))/(2.*cfg[4]);
  }else{
   // Evaluate return-to-goal ONCE at the fixed horizon endpoint. A running
   // goal penalty activated only after passage rewards delaying that event.
   // Position progress is already counted above. Only the remaining braking
   // distance is added; no branch-dependent absolute goal-distance cost.
   double terminal=0.;for(int a=0;a<3;a++)terminal+=sq(s[3+a]);
   race[b]+=terminal/(2.*cfg[4]);
  }
 }
}
