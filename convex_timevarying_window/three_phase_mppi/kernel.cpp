// Appended to shared physical primitives only; no legacy controller/FSM.
namespace phase3 {
using namespace v3;
void target(const double *policy,double t,const double *reference,double *pva){
 if(policy[20]<.5){for(int j=0;j<9;j++)pva[j]=reference[j];
  double at=t+policy[19];
  for(int j=0;j<3;j++){pva[j]+=.5*at*at*policy[21+j];pva[3+j]+=at*policy[21+j];pva[6+j]+=policy[21+j];}return;}
 double at=t+policy[19],q=std::min(at,policy[18]);
 for(int a=0;a<3;a++){
  const double *c=policy+6*a;
  double p=c[5],v=5*c[5],acc=20*c[5];
  for(int j=4;j>=0;j--)p=p*q+c[j];
  for(int j=4;j>=1;j--)v=v*q+j*c[j];
  for(int j=4;j>=2;j--)acc=acc*q+j*(j-1)*c[j];
  pva[a]=p+std::max(0.,at-policy[18])*v;pva[3+a]=v;
  pva[6+a]=at<=policy[18]?acc:0.;
 }
}
void command(double *x,const double *pva,const double *cfg,double *prev,double *u){
 double old[4];for(int j=0;j<4;j++)old[j]=prev[j];
 // Control-authority reserve belongs to the FEEDBACK command, not a lower
 // physical rejection threshold that can make recovery impossible.
 double feedback_cfg[12];for(int j=0;j<12;j++)feedback_cfg[j]=cfg[j];
 feedback_cfg[4]=cfg[4]-1.5;feedback_cfg[5]=std::cos(38.*3.141592653589793/180.);
 double desired[3],speed=std::sqrt(sq(x[3])+sq(x[4])+sq(x[5])),radial=0.;
 for(int a=0;a<3;a++){
  desired[a]=pva[6+a]+4*(pva[a]-x[a])+4*(pva[3+a]-x[3+a]);
  radial+=desired[a]*x[3+a]/std::max(speed,1e-9);
 }
 // A velocity-aware feedback governor prevents sampling against the exact
 // 12 m/s boundary from exhausting correction authority in the real plant.
 double allowed=2.*(cfg[3]-.5-speed);
 if(radial>allowed)for(int a=0;a<3;a++)desired[a]-=(radial-allowed)*x[3+a]/std::max(speed,1e-9);
 feedback(x,x,x+3,desired,4.,feedback_cfg,prev,u);
 for(int a=0;a<3;a++)u[1+a]=std::clamp(u[1+a]-.35*x[10+a],-3.,3.); // measured-rate damping
 // At the CURRENT attitude, acceleration is affine in collective thrust.
 // Project onto its scalar feasible interval; motor lag and future attitude
 // still undergo rollout checks, so this is not advertised as a CBF proof.
 double R[9],drag=0.,axial=0.,base[3],bn=0.,bz=0.;rotation(x+6,R);
 for(int j=0;j<4;j++)drag+=8.06428e-5*x[13+j]/mass;
 for(int a=0;a<3;a++)axial+=x[3+a]*R[3*a+2];
 for(int a=0;a<3;a++){
  base[a]=-(a==2?g:0.)-drag*(x[3+a]-axial*R[3*a+2]);
  bn+=sq(base[a]);bz+=base[a]*R[3*a+2];
 }
 double half=std::sqrt(std::max(0.,sq(cfg[4]-1.)-bn+bz*bz));
 u[0]=std::clamp(u[0],mass*(-bz-half),mass*(-bz+half));
 u[0]=std::clamp(u[0],4*kf*150.*150.,4*maxf);
 for(int j=0;j<4;j++)u[j]=prev[j]=std::clamp(u[j],old[j]-(j?.08:3.2),old[j]+(j?.08:3.2));
}
}
extern "C" void phase3_command(double *x,const double *pva,const double *cfg,double *prev,double *u){
 phase3::command(x,pva,cfg,prev,u);
}
extern "C" void phase3_integrate(double *x,const double *u,double dt){
 int n=int(std::ceil(dt/.005));for(int j=0;j<n;j++)v3::step(x,u,dt/n);
}
extern "C" void phase3_batch(int batch,int steps,int gates,const double *initial,const double *previous,
 const double *policies,const double *reference,const double *poses,const double *geom,
 const double *speeds,const double *signs,const double *cfg,int first,int initial_phase,int mode,
 double *metrics,double *paths,double *commands){
 using namespace v3;
 for(int b=0;b<batch;b++){
  double s[17],prev[4];for(int j=0;j<17;j++)s[j]=initial[j];for(int j=0;j<4;j++)prev[j]=previous[j];
  const double *pol=policies+25*b;double *m=metrics+10*b;
  for(int j=0;j<10;j++)m[j]=0.;m[2]=1e9;m[3]=-1.;m[4]=first;
  int phase=initial_phase,gate=first,reason=0,exit_step=-1;double cost=0.;
  const double slab=cfg[0]+cfg[1]+cfg[2],h=.01;
  const double bound=4*maxf/mass+g+4*8.06428e-5*1000/mass*cfg[3];
  if(paths)for(int j=0;j<17;j++)paths[j]=s[j];
  for(int k=0;k<steps;k++){
   double old[17],pva[9],u[4];for(int j=0;j<17;j++)old[j]=s[j];
   phase3::target(pol,k*h,reference+9*k,pva);phase3::command(s,pva,cfg,prev,u);
   if(commands)for(int j=0;j<4;j++)commands[4*k+j]=u[j];
   for(int sub=0;sub<2;sub++){
    step(s,u,.005);double d[17],r[9];derivative(s,u,d);rotation(s+6,r);
    if(!std::isfinite(s[0]))reason|=256;
    if(sq(s[3])+sq(s[4])+sq(s[5])>sq(cfg[3]))reason|=4;
    if(sq(d[3])+sq(d[4])+sq(d[5])>sq(cfg[4]))reason|=8;
    if(r[8]<cfg[5])reason|=16;
    if(std::hypot(s[10],s[11])>3.839724354||std::abs(s[12])>3.490658504)reason|=32;
   }
   double speed=std::sqrt(sq(old[3])+sq(old[4])+sq(old[5]))+bound*h,clear=1e9;
   for(int j=0;j<gates;j++){
    double a[3],c[3];local(old,poses+(k*gates+j)*12,a);local(s,poses+((k+1)*gates+j)*12,c);
    clear=std::min(clear,std::min(distance(a,geom+32*j),distance(c,geom+32*j))
       -slab-.5*(speed+speeds[j])*h);
   }
   for(int a=0;a<3;a++)clear=std::min(clear,
    std::min(std::min(old[a]-cfg[6+a],s[a]-cfg[6+a]),std::min(cfg[9+a]-old[a],cfg[9+a]-s[a]))
     -cfg[1]-.5*(std::abs(old[3+a])+bound*h)*h);
   if(steps>10)clear-=cfg[12]*std::min(1.,(k+1)*h/.3);
   // Extra prediction-error reserve grows from the measured initial state.
   // A uniform extra margin at t=0 would exclude safe recovery states merely
   // because they are already outside the tightened planning set.
   m[2]=std::min(m[2],clear);if(!(clear>0))reason|=1;
   if(gate<gates){
    double qa[3],qb[3];local(old,poses+(k*gates+gate)*12,qa);local(s,poses+((k+1)*gates+gate)*12,qb);
    double a=qa[2]*signs[gate],z=qb[2]*signs[gate];
    if(phase==0 && a<=-slab && z>-slab && inside(qb,geom+32*gate,-slab)){
     phase=1;if(mode==0)reason|=64;
    }
    if(phase){
     // Task topology: centre remains within physical opening; the separate
     // 3D frame distance checks the whole spherical vehicle, not just its centre.
     if(!inside(qb,geom+32*gate,.065))reason|=2;
     if(z<=-slab || (phase==2&&z<0))reason|=128;
     if(a<=0 && z>0)phase=2;
     if(phase==2&&z>slab){gate++;phase=0;if(exit_step<0){exit_step=k+1;m[3]=(k+1)*h;}}
    }
   }
   if(mode==0)for(int a=0;a<3;a++)cost+=h*(sq(s[a]-reference[9*(k+1)+a])+ .15*sq(s[3+a]-reference[9*(k+1)+3+a]));
   if(paths)for(int j=0;j<17;j++)paths[(k+1)*17+j]=s[j];
   m[5]=k+1;m[4]=gate;m[7]=phase;
   if(k==9)m[6]=reason==0 && m[2]>.05;
   if(reason)break;
   if(exit_step>=0 && k+1>=exit_step+100)break;
  }
  if(mode>0 && steps>10 && exit_step<0)reason|=512;
  if(mode>0 && steps>10 && exit_step>=0 && exit_step+100>steps)reason|=1024;
  if(steps>10)for(int a=0;a<3;a++){
   // A short clear exit is insufficient if it leaves no room to turn/brake.
   // Apply the same axis-wise room condition to every gate and transit plan.
   // This is an engineering reachability proxy, not a proven invariant set.
   double lo=std::max(0.,-s[3+a]),hi=std::max(0.,s[3+a]);
   double lower_need=.5*lo+sq(lo)/(2*.6*(cfg[4]-1.5));
   double upper_need=.5*hi+sq(hi)/(2*.6*(cfg[4]-1.5));
   if(s[a]-cfg[6+a]-cfg[1]-cfg[12]<lower_need || cfg[9+a]-s[a]-cfg[1]-cfg[12]<upper_need)reason|=2048;
  }
  if(mode>0)cost=m[3]<0?1e6:m[3];
  // Small continuation preference only; exit time remains dominant.
  if(mode>0 && gate>first && gate<gates){
   const double *p=poses+(int(m[5])*gates+gate)*12;double dist=0.,dot=0.;
   for(int a=0;a<3;a++){dist+=sq(p[a]-s[a]);dot+=(p[a]-s[a])*s[3+a];}
   cost+=.02*(std::sqrt(dist)-.3*dot/std::max(1.,std::sqrt(dist)));
  }
  m[0]=cost;m[1]=reason;m[8]=s[3];m[9]=s[4];
 }
}
