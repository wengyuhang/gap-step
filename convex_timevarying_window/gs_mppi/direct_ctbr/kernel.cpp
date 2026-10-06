// Appended to the physical primitives only, compiled with V3_CTBR=1.
namespace direct {
using namespace v3;
void clip(double *u){u[0]=std::clamp(u[0],4*kf*150.*150.,4*maxf);for(int j=1;j<4;j++)u[j]=std::clamp(u[j],-3.,3.);}
double norm3(const double *v){return std::sqrt(sq(v[0])+sq(v[1])+sq(v[2]));}
double aperture_radius(const double *geom){
 if(geom[0]<.5)return geom[1];
 double radius=0.;for(int i=0;i<int(geom[2]);i++)radius=std::max(radius,std::hypot(geom[3+2*i],geom[4+2*i]));
 return radius;
}
double vehicle_yaw(const double *x){
 double r[9];rotation(x+6,r);return std::atan2(r[3],r[0]);
}
double wrap_angle(double angle){return std::atan2(std::sin(angle),std::cos(angle));}
// Unwrap the horizontal PATH tangent, keeping its last value when a path has
// no meaningful horizontal motion. This is not an observed-velocity target.
double tangent_heading(const double *tangent,double previous){
 if(std::hypot(tangent[0],tangent[1])<.2)return previous;
 return previous+wrap_angle(std::atan2(tangent[1],tangent[0])-previous);
}
double heading_error(const double *x,double heading){
 return wrap_angle(heading-vehicle_yaw(x));
}
// A hairpin should not leave a gate with a large velocity pointing away
// from the following gate. The slow endpoint follows v^2=2*a*distance;
// straight-ahead exits keep the vehicle-scaled cruise preference.
double exit_speed(const double *poses,const double *cfg,const double *signs,int gates,int gate,int k,int steps,double dt){
 if(cfg[16]<.5)return 9.;
 const double *pose=poses+12*(4*k*gates+gate);
 int look=std::min(steps,k+int(.6/dt));double direction[3],normal[3];
 for(int j=0;j<3;j++){
  double target=gate+1<gates?poses[12*(4*look*gates+gate+1)+j]:cfg[13+j];
  direction[j]=target-pose[j];normal[j]=signs[gate]*pose[3+3*j+2];
 }
 double length=norm3(direction),cosine=0.;
 for(int j=0;j<3;j++)cosine+=normal[j]*direction[j]/std::max(.1,length);
 double fast=.9*cfg[3],slow=std::sqrt(2.*.9*cfg[4]*(cfg[0]+cfg[1]+cfg[2]));
 return fast+(std::min(fast,slow)-fast)*std::clamp(-cosine,0.,1.);
}
// A soft continuation value, not a prescribed crossing point. Once a
// candidate has fully exited, retain useful velocity toward the NEXT gate
// (or the return goal). All geometric/physical checks still apply.
void continuation(const double *x,const double *poses,const double *cfg,int gates,int gate,int k,int steps,double dt,double *desired,double *direction){
 double target[3];int look=std::min(steps,k+int(.6/dt));
 for(int j=0;j<3;j++)target[j]=gate<gates?poses[12*(4*look*gates+gate)+j]:cfg[13+j];
 double d[3];for(int j=0;j<3;j++)d[j]=target[j]-x[j];
 double length=norm3(d),speed=std::min(cfg[16]>.5?.9*cfg[3]:9.,std::sqrt(8.*length));
 for(int j=0;j<3;j++){direction[j]=d[j]/std::max(.1,length);desired[j]=speed*direction[j];}
}
// Warm-start only: produces a CTBR tape in the SAME 17-state plant.
// No feedback controller is called after a tape has been selected.
void seed_command(const double *x,const double *ref,const double *jerk,bool feedforward,double heading,double heading_rate,double accel_limit,double *u){
 double r[9];rotation(x+6,r);double a[3],jdes[3]={},der[17],hover[4]={mass*g,0.,0.,0.};
 if(feedforward)derivative(x,hover,der);
 for(int j=0;j<3;j++)a[j]=ref[6+j]+2.*(ref[j]-x[j])+2.8*(ref[3+j]-x[3+j]);
 if(feedforward)for(int j=0;j<3;j++)jdes[j]=jerk[j]+2.*(ref[3+j]-x[3+j])+2.8*(ref[6+j]-der[3+j]);
 double length=norm3(a),scale=std::min(1.,accel_limit/std::max(1e-9,length));
 if(feedforward && scale<1.){
  double radial=0.;for(int j=0;j<3;j++)radial+=a[j]*jdes[j]/(length*length);
  for(int j=0;j<3;j++)jdes[j]=scale*(jdes[j]-radial*a[j]);
 }
 for(int j=0;j<3;j++)a[j]*=scale;
 double drag=0.,axial=0.;for(int j=0;j<4;j++)drag+=8.06428e-5*x[13+j]/mass;
 for(int j=0;j<3;j++)axial+=x[3+j]*r[3*j+2];
 if(feedforward){
  double dz[3],axial_dot=0.;for(int j=0;j<3;j++)dz[j]=r[3*j]*x[11]-r[3*j+1]*x[10];
  for(int j=0;j<3;j++)axial_dot+=der[3+j]*r[3*j+2]+x[3+j]*dz[j];
  for(int j=0;j<3;j++)jdes[j]+=drag*(der[3+j]-axial_dot*r[3*j+2]-axial*dz[j]);
 }
 for(int j=0;j<3;j++)a[j]+=drag*(x[3+j]-axial*r[3*j+2]);a[2]+=g;
 double n=norm3(a),zd[3];for(int j=0;j<3;j++)zd[j]=a[j]/std::max(1e-9,n);
 // Tilt toward acceleration; yaw follows the planned path tangent with its
 // curvature feedforward. Convert Euler yaw rate to the body-z CTBR input.
 double zb[3]={};for(int j=0;j<3;j++)for(int k=0;k<3;k++)zb[j]+=r[3*k+j]*zd[k];
 u[0]=mass*(a[0]*r[2]+a[1]*r[5]+a[2]*r[8]);
 double world_rate[3]={a[1]*jdes[2]-a[2]*jdes[1],a[2]*jdes[0]-a[0]*jdes[2],a[0]*jdes[1]-a[1]*jdes[0]},ff[3]={};
 if(feedforward)for(int j=0;j<3;j++)for(int k=0;k<3;k++)ff[j]+=r[3*k+j]*world_rate[k]/std::max(1e-9,n*n);
 u[1]=-5.*zb[1]+1.25*ff[0]-.25*x[10];u[2]=5.*zb[0]+1.25*ff[1]-.25*x[11];
 double yaw_rate=std::clamp(heading_rate+1.5*heading_error(x,heading),-1.2,1.2);
 double cos_pitch=std::hypot(r[0],r[3]);
 double sin_roll=r[7]/std::max(.1,cos_pitch),cos_roll=r[8]/std::max(.1,cos_pitch);
 u[3]=std::clamp((yaw_rate*cos_pitch-u[2]*sin_roll)/std::max(.5,cos_roll)-.25*x[12],-1.2,1.2);clip(u);
}
// One CONTROL step, with independent geometry/physical checks at each 5 ms.
double check(const double *x,const double *u,const double *poses,int gates,const double *geom,const double *speeds,const double *signs,const double *cfg,int first,int phase,double dt,double *end,int *reason,int *next,int *nextphase){
 double h=1e20;*reason=0;*next=first;*nextphase=phase;
 for(int j=0;j<17;j++)end[j]=x[j];
 const double slab=cfg[0]+cfg[1]+cfg[2],bound=4*maxf/mass+g+4*8.06428e-5*1000/mass*cfg[3];
 double sub=dt/4.;
 for(int k=0;k<4;k++){
  double old[17];for(int j=0;j<17;j++)old[j]=end[j];step(end,u,sub);
  if(!std::isfinite(end[0])){*reason|=128;return -1e9;}
  double der[17],r[9];derivative(end,u,der);rotation(end+6,r);
  double vs=(cfg[3]-norm3(end+3))*.2,as=(cfg[4]-norm3(der+3))*.05;
  double ts=(r[8]-cfg[5])*2.,rs=std::min(3.839724354-std::hypot(end[10],end[11]),3.490658504-std::abs(end[12]))*.3;
  if(vs<0)*reason|=2;if(as<0)*reason|=4;if(ts<0)*reason|=8;if(rs<0)*reason|=16;
  h=std::min(h,std::min(std::min(vs,as),std::min(ts,rs)));
  double speed=norm3(old+3)+bound*sub;
  for(int gate=0;gate<gates;gate++){
   double qa[3],qb[3];local(old,poses+12*(k*gates+gate),qa);local(end,poses+12*((k+1)*gates+gate),qb);
   double clear=std::min(distance(qa,geom+32*gate),distance(qb,geom+32*gate))-slab-.5*(speed+speeds[gate])*sub;
   h=std::min(h,clear);if(clear<0)*reason|=1;
  }
  for(int j=0;j<3;j++){
   double clear=std::min(std::min(old[j],end[j])-cfg[6+j],cfg[9+j]-std::max(old[j],end[j]))-cfg[1]-.5*(std::abs(old[3+j])+bound*sub)*sub;
   h=std::min(h,clear);if(clear<0)*reason|=1;
  }
  if(*next<gates){
   int gate=*next;double qa[3],qb[3];local(old,poses+12*(k*gates+gate),qa);local(end,poses+12*((k+1)*gates+gate),qb);
   double a=signs[gate]*qa[2],b=signs[gate]*qb[2];
   // Crossing the infinite supporting plane far outside the finite opening
   // is not entry into the gate.  Only a body aligned with the actual aperture
   // can start the committed passage phase.
   if(!*nextphase && a<=-slab && b>-slab){
    if(inside(qb,geom+32*gate,.065))*nextphase=1;
    else{
     double frozen[3];local(end,poses+12*(k*gates+gate),frozen);
     if(std::hypot(qb[0],qb[1])<=direct::aperture_radius(geom+32*gate)+slab &&
        signs[gate]*frozen[2]>-slab){*reason|=32;h=std::min(h,-.001);}
    }
   }
   if(*nextphase){
    if(!inside(qb,geom+32*gate,.065)){*reason|=32;h=std::min(h,-.001);}
    if(b<=-slab || (*nextphase==2&&b<0.)){*reason|=64;h=std::min(h,-.001);}
    if(a<=0.&&b>0.)*nextphase=2;
    if(*nextphase==2&&b>slab){(*next)++;*nextphase=0;}
   }
  }
 }
 // A discrete time-varying geometric barrier for the ACTUAL next command.
 // Both endpoints use the corresponding known gate pose.  The 5 ms checks
 // above still protect the interval; this condition discourages spending all
 // remaining clearance in one 20 ms command.  It is not a viability proof for
 // the delayed 17-state CTBR plant.
 for(int gate=0;gate<gates;gate++){
  double qa[3],qb[3];local(x,poses+12*gate,qa);
  local(end,poses+12*(4*gates+gate),qb);
  double before=distance(qa,geom+32*gate)-slab;
  double after=distance(qb,geom+32*gate)-slab;
  double cbf=after-.4*std::max(0.,before);
  h=std::min(h,cbf);
  if(cbf < -1e-6)*reason|=256;
 }
 // Rotor thrust is a state, not an instantaneous CTBR input.  Preserve a
 // fraction of the acceleration headroom each command so the rate-limited
 // motors are corrected before the next measurement is already infeasible.
 double d0[17],d1[17];derivative(x,u,d0);derivative(end,u,d1);
 double accel_headroom0=cfg[4]-norm3(d0+3);
 double accel_headroom1=cfg[4]-norm3(d1+3);
 double accel_barrier=accel_headroom1-.4*std::max(0.,accel_headroom0);
 h=std::min(h,.05*accel_barrier);
 if(accel_barrier < -1e-6)*reason|=512;
 return h;
}
double filter(const double *x,const double *proposal,const double *poses,int gates,const double *geom,const double *speeds,const double *signs,const double *cfg,int first,int phase,double dt,double *u,double *end,int *reason,int *next,int *ph,int *changed){
 for(int j=0;j<4;j++)u[j]=proposal[j];clip(u);
 double h=check(x,u,poses,gates,geom,speeds,signs,cfg,first,phase,dt,end,reason,next,ph);
 *changed=0;if(h>=0)return h;
 // Finite-difference projection in normalized CTBR coordinates. This is a
 // local approximation only: a nonlinear check is mandatory after correction.
 for(int iter=0;iter<int(cfg[12]);iter++){
  double grad[4],norm=0.;
  for(int j=0;j<4;j++){
   double test[4],state[17];for(int k=0;k<4;k++)test[k]=u[k];
   double scale=j?1.:mass*g;test[j]+=.002*scale;
   int rr,nn,pp;double hp=check(x,test,poses,gates,geom,speeds,signs,cfg,first,phase,dt,state,&rr,&nn,&pp);
   grad[j]=(hp-h)/.002;norm+=sq(grad[j]);
  }
  if(norm<1e-12)break;
  for(int j=0;j<4;j++)u[j]+=(-h+1e-5)*grad[j]/norm*(j?1.:mass*g);
  clip(u);*changed=1;h=check(x,u,poses,gates,geom,speeds,signs,cfg,first,phase,dt,end,reason,next,ph);
  if(h>=0)break;
 }
 return h;
}
}
#ifndef DIRECT_GPU
extern "C" void direct_integrate(double *x,const double *u,double dt){int n=int(std::ceil(dt/.005));for(int i=0;i<n;i++)v3::step(x,u,dt/n);}
extern "C" void direct_seed(int steps,int gates,const double *initial,const double *reference,const double *poses,const double *signs,const double *geom,const double *cfg,int first,int mode,double dt,double *controls){
 double x[17];for(int j=0;j<17;j++)x[j]=initial[j];int gate=first;
 bool entered=mode==2;
 bool centered=false;
 if(entered && first<gates){double q[3];v3::local(x,poses+12*first,q);centered=signs[first]*q[2]>=0.;}
 double heading=direct::vehicle_yaw(x);
 for(int k=0;k<steps;k++){
  double ref[9];for(int j=0;j<9;j++)ref[j]=reference[9*k+j];
  if(mode && gate!=first){
   double desired[3],direction[3];direct::continuation(x,poses,cfg,gates,gate,k,steps,dt,desired,direction);
   for(int j=0;j<3;j++){ref[j]=x[j];ref[3+j]=desired[j];ref[6+j]=0.;}
  }else if(mode==1 && !entered && gate<gates){
   // Prepare: preserve/accelerate lateral closure while limiting forward
   // speed to the time needed to reach the inner aperture. Gate motion is
   // queried at this step and the next one, including rotation.
   const double *pose=poses+12*(4*k*gates+gate);
   const double *nextpose=poses+12*(std::min(4*steps,4*k+4)*gates+gate);
   double q[3],qnext[3];v3::local(x,pose,q);
   double ahead[17];for(int j=0;j<17;j++)ahead[j]=x[j];
   for(int j=0;j<3;j++)ahead[j]+=dt*x[3+j]-(nextpose[j]-pose[j]);
   v3::local(ahead,pose,qnext);
   double lateral=std::hypot(q[0],q[1]),closing=-(q[0]*(qnext[0]-q[0])+q[1]*(qnext[1]-q[1]))/(dt*std::max(.01,lateral));
   double centre[3]={0.,0.,0.};double inner=std::max(.05,v3::distance(centre,geom+32*gate)-cfg[0]-cfg[1]-cfg[2]-.1);
   double remaining=std::max(0.,lateral-inner),a=2.5;
   double reach=(std::sqrt(closing*closing+2*a*remaining)-closing)/a;
   reach=std::clamp(reach,.35,3.5);
   double entry=std::max(0.,-cfg[0]-cfg[1]-cfg[2]-signs[gate]*q[2]);
   double desired_exit=direct::exit_speed(poses,cfg,signs,gates,gate,k,steps,dt);
   double approach=cfg[16]>.5?std::min(.9*cfg[3],std::sqrt(desired_exit*desired_exit+2.*.9*cfg[4]*entry)):9.;
   double forward=std::clamp(entry/(reach+.35),.5,approach);
   // Aligned flight need not brake to a near-stop at the entry slab. Blend
   // out that capture-speed limit only when both current and predicted
   // arrival lie in the inner aperture of the KNOWN moving gate.
   double vn=signs[gate]*(qnext[2]-q[2])/dt;
   double eta=std::clamp((entry+cfg[0]+cfg[1]+cfg[2])/std::max(3.,vn),0.,.8);
   int at=std::min(4*steps,4*k+int(eta/(dt/4.)));
   double projected[3],future[3];for(int j=0;j<3;j++)projected[j]=x[j]+eta*x[3+j];
   v3::local(projected,poses+12*(at*gates+gate),future);
   double outside=std::max(0.,std::hypot(future[0],future[1])-inner);
   double aligned=std::exp(-v3::sq(remaining/.35)-v3::sq(outside/.35));
   forward+=aligned*(approach-forward);
   double tangent[2]={-q[0]/std::max(.7,reach),-q[1]/std::max(.7,reach)};
   double tangentspeed=std::hypot(tangent[0],tangent[1]);
   if(tangentspeed>10.5)for(double &v:tangent)v*=10.5/tangentspeed;
   double desired[3]={tangent[0],tangent[1],signs[gate]*forward};
   double turn[3]={};
   for(int j=0;j<3;j++)for(int h=0;h<3;h++)turn[j]+=(nextpose[3+3*j+h]-pose[3+3*j+h])*q[h]/dt;
   double turn_norm=direct::norm3(turn),turn_scale=std::min(1.,(.25*cfg[3])/std::max(1e-9,turn_norm));
   for(int j=0;j<3;j++){
    // The gate centre can be followed. A rapidly rotating frame far from
    // its opening cannot dictate arbitrarily large world-frame velocity.
    ref[3+j]=(nextpose[j]-pose[j])/dt+turn_scale*turn[j];
    for(int h=0;h<3;h++)ref[3+j]+=pose[3+3*j+h]*desired[h];
    ref[j]=x[j];ref[6+j]=0.;
   }
  }else if(mode && entered && gate==first && gate<gates){
   const double *pose=poses+12*(4*k*gates+gate);double q[3];v3::local(x,pose,q);
   double delta[3];for(int j=0;j<3;j++)delta[j]=pose[j]-x[j];
   double eta=std::min(2.,direct::norm3(delta)/8.);int at=std::min(4*steps,4*k+int(eta/(dt/4)));
   const double *future=poses+12*(at*gates+gate);double d[3];
   for(int j=0;j<3;j++)d[j]=future[j]+signs[gate]*future[3+3*j+2]*1.8-x[j];
   double norm=direct::norm3(d);
   double speed=direct::exit_speed(poses,cfg,signs,gates,gate,k,steps,dt);
   for(int j=0;j<3;j++){ref[j]=x[j];ref[3+j]=speed*d[j]/std::max(1.,norm);ref[6+j]=0.;}
  }
  double desired=direct::tangent_heading(ref+3,heading),rate=0.;
  if(k)rate=(desired-heading)/dt;
  else if(!mode && std::hypot(ref[3],ref[4])>=.2)
   rate=(ref[3]*ref[7]-ref[4]*ref[6])/(ref[3]*ref[3]+ref[4]*ref[4]);
  heading=desired;
  double jerk[3];for(int j=0;j<3;j++)jerk[j]=(reference[9*(k+1)+6+j]-reference[9*k+6+j])/dt;
  direct::seed_command(x,ref,jerk,cfg[17]>.5 && !mode,heading,std::clamp(rate,-1.2,1.2),cfg[17]>.5?.9*cfg[4]:6.5,controls+4*k);
  double old[17];for(int j=0;j<17;j++)old[j]=x[j];
  for(int sub=0;sub<4;sub++)v3::step(x,controls+4*k,dt/4.);
  if(mode && gate==first && gate<gates){
   double qa[3],qb[3];v3::local(old,poses+12*(4*k*gates+gate),qa);
   v3::local(x,poses+12*(4*(k+1)*gates+gate),qb);
   double a=signs[gate]*qa[2],b=signs[gate]*qb[2],slab=cfg[0]+cfg[1]+cfg[2];
   if(!entered && a<=-slab && b>-slab && v3::inside(qb,geom+32*gate,.065))entered=true;
   if(entered && a<=0. && b>0.)centered=true;
   if(entered && centered && b>slab)gate++;
  }
 }
}
#endif
extern "C" void direct_rollout(int batch,int steps,int gates,const double *initial,const double *controls,const double *previous_command,const double *reference,const double *poses,const double *geom,const double *speeds,const double *signs,const double *cfg,int first,int phase,int mode,int soft_future,double dt,double *metrics,double *paths,double *actions){
 for(int b=0;b<batch;b++){
  double x[17],cost=0.,minimum=1e20,prev[4];
  for(int j=0;j<4;j++)prev[j]=previous_command[j];
  for(int j=0;j<17;j++)x[j]=initial[j];if(paths)for(int j=0;j<17;j++)paths[j]=x[j];
  double heading=direct::vehicle_yaw(x);
  int gate=first,ph=phase,reason=0,changed=0,k=0;
  for(;k<steps;k++){
   double next[17],prior[17],u[4];int ng,np,mod,step_reason;
   for(int j=0;j<17;j++)prior[j]=x[j];
   // Sample within the same per-publication control slew envelope used by
   // the PX4 sender. A 20 ms future step gets twice its 10 ms budget.
   // The physical safety projection may still override this preference.
   double proposal[4];
   for(int j=0;j<4;j++){
    double limit=(j==0?2.:(j==3?.2:.3))*(k==0?1.:2.);
    proposal[j]=std::clamp(controls[4*(steps*b+k)+j],prev[j]-limit,prev[j]+limit);
   }
   double h=direct::filter(x,proposal,poses+12*gates*4*k,gates,geom,speeds,signs,cfg,gate,ph,dt,u,next,&step_reason,&ng,&np,&mod);
   minimum=std::min(minimum,h);changed+=mod;
   // A delayed CTBR plant need not be able to satisfy the proposed barrier
   // from every measured state.  The geometric/acceleration barriers are
   // therefore prioritized and penalized, but only actual interval collision
   // and physical-limit violations are hard rejections.
   if(step_reason&768)cost+=dt*20.*v3::sq(std::max(0.,-20.*h));
   reason=step_reason&~768;
   if(reason)break;
   for(int j=0;j<17;j++)x[j]=next[j];gate=ng;ph=np;
   if(paths)for(int j=0;j<17;j++)paths[17*(k+1)+j]=x[j];
   if(actions)for(int j=0;j<4;j++)actions[4*k+j]=u[j];
   if(!mode){for(int j=0;j<3;j++)cost+=dt*(.6*v3::sq(x[j]-reference[9*(k+1)+j])+.25*v3::sq(x[3+j]-reference[9*(k+1)+3+j]));}
   else if(!ph && gate==first && first<gates){
    double q[3],before[3];v3::local(x,poses+12*((k+1)*4*gates+first),q);
    // Measure commanded motion in one instantaneous gate frame. Angular
    // motion of a distant plane creates a spurious, arbitrarily large local
    // closing rate; actual future gate poses enter the lookahead term below.
    v3::local(prior,poses+12*((k+1)*4*gates+first),before);
    double lateral=std::hypot(q[0],q[1]),prior=std::hypot(before[0],before[1]);
    double closing=(prior-lateral)/dt;
    double centre[3]={0.,0.,0.};double inner=std::max(.05,v3::distance(centre,geom+32*first)-cfg[0]-cfg[1]-cfg[2]-.1);
    double rem=std::max(0.,lateral-inner),acc=2.5;
    double reach=(std::sqrt(closing*closing+2*acc*rem)-closing)/acc;
    double entry=std::max(0.,-cfg[0]-cfg[1]-cfg[2]-signs[first]*q[2]);
    double vn=signs[first]*(q[2]-before[2])/dt;
    double shortage=std::clamp(rem/.3,0.,1.)*std::max(0.,vn*(reach+.35)-entry);
    // Ballistic arrival is a soft warning, not a safety certificate: query
    // the known aperture pose 0.8 s ahead and penalize arriving near its
    // plane before reaching its inner opening. This couples lateral capture
    // to the future moving aperture instead of rewarding plane approach alone.
    int look=std::min(steps,k+1+int(.8/dt));
    double tau=(look-k-1)*dt,projected[3],future[3];
    for(int j=0;j<3;j++)projected[j]=x[j]+tau*x[3+j];
    v3::local(projected,poses+12*(look*4*gates+first),future);
    double near=std::clamp((signs[first]*future[2]+cfg[0]+cfg[1]+cfg[2]+2.)/2.,0.,1.);
    double outside=std::max(0.,std::hypot(future[0],future[1])-inner);
    cost+=dt*(12.+.035*lateral*lateral+.12*entry*entry+2.*shortage*shortage+2.*near*outside*outside);
   }
   else if(ph && gate==first && first<gates){
    double q[3];v3::local(x,poses+12*((k+1)*4*gates+first),q);
    cost+=dt*(.1*(q[0]*q[0]+q[1]*q[1])+.5*v3::sq(signs[first]*q[2]-1.3)+14.);
   }
   // A tilt close to its limit with outward body rate has little remaining
   // braking authority in the delayed attitude loop. Penalize that state
   // before the next 20 ms hard check becomes impossible to satisfy.
   double r[9];v3::rotation(x+6,r);
   double outward=std::min(0.,r[6]*x[11]-r[7]*x[10]);
   // Leave 2 degrees of soft attitude reserve and anticipate 0.18 s of
   // outward motion (rate-loop response plus online command latency).
   // This changes preference, never the hard 42 degree physical limit.
   double soft_cos=std::cos(std::max(0.,std::acos(cfg[5])-.03490658504));
   double tilt_reserve=r[8]-soft_cos+.18*outward;
   cost+=dt*2000.*v3::sq(std::max(0.,-tilt_reserve));
   if(mode && (ph || cfg[17]>.5) && gate==first && first<gates){
    double q[3];v3::local(x,poses+12*((k+1)*4*gates+first),q);
    double clearance=v3::distance(q,geom+32*first)-(cfg[0]+cfg[1]+cfg[2]);
    // A soft reserve near the moving frame leaves steering authority during
    // the committed passage. It does not impose a centre crossing point.
    cost+=dt*200.*v3::sq(std::max(0.,cfg[1]-clearance));
   }
   // The first step is compared with the command actually sent to PX4, not
   // with an invented hover input. This discourages optimizer-to-optimizer
   // chattering; the sender separately enforces a safety-checked slew bound.
   for(int j=0;j<4;j++){
    double scale=j?1.:(k==0?4.:v3::mass*v3::g);
    cost+=(k==0?.4:(j?.0008/dt:dt*.05))*v3::sq((u[j]-prev[j])/scale);prev[j]=u[j];
   }
   // Penalize angular acceleration, not the sustained banking needed for
   // racing. This suppresses body-rate reversals without preferring hover.
   for(int j=0;j<3;j++)cost+=.0015/dt*v3::sq(x[10+j]-prior[10+j]);
   // Ordinary tracking follows TOGT's tangent, even when the measured drone
   // is drifting sideways. During takeover, use this candidate's own path
   // chord, never the abandoned TOGT reference or raw measured velocity.
   double tangent[3];
   for(int j=0;j<3;j++)tangent[j]=mode?(x[j]-prior[j])/dt:reference[9*(k+1)+3+j];
   heading=direct::tangent_heading(tangent,heading);
   cost+=dt*.2*v3::sq(direct::heading_error(x,heading));
  }
  // For receding-horizon MPPI, a later failed step is a finite warning, not
  // grounds to throw away a safe first command.  Penalize early failures more
  // strongly and compare terminal progress at the ACTUAL last valid time.
  if(soft_future&&reason){double remaining=double(steps-k)/steps;
    cost+=20.+80.*remaining*remaining;
    if(!mode)for(int j=0;j<3;j++)cost+=2*v3::sq(x[j]-reference[9*k+j])+.4*v3::sq(x[3+j]-reference[9*k+3+j]);
    else if(gate==first&&first<gates){double q[3];v3::local(x,poses+12*(k*4*gates+first),q);
      cost+=3.*(q[0]*q[0]+q[1]*q[1]+v3::sq(signs[first]*q[2]-1.3));}
  }else{
    if(!mode)for(int j=0;j<3;j++)cost+=2*v3::sq(x[j]-reference[9*steps+j])+.4*v3::sq(x[3+j]-reference[9*steps+3+j]);
    else if(gate==first && first<gates){double q[3];v3::local(x,poses+12*(steps*4*gates+first),q);
      cost+=12.+(ph?3.:1.)*(q[0]*q[0]+q[1]*q[1]+v3::sq(signs[first]*q[2]-1.3));}
  }
  if(mode && first<gates){
   // The SAME downstream value applies on both sides of the exit event.
   // Adding it only to successful passages made crossing more expensive
   // than hovering just before the final gate when return required a turn.
   int nextgate=std::min(first+1,gates),look=std::min(steps,k+int(.6/dt));
   double desired[3],direction[3];direct::continuation(x,poses,cfg,gates,nextgate,k,steps,dt,desired,direction);
   double distance2=0.,forward=0.,speed2=0.;
   for(int j=0;j<3;j++){
    double target=nextgate<gates?poses[12*(4*look*gates+nextgate)+j]:cfg[13+j];
    distance2+=v3::sq(target-x[j]);forward+=x[3+j]*direction[j];speed2+=v3::sq(x[3+j]);
   }
   cost+=.6*std::sqrt(distance2)-.5*forward+.04*std::max(0.,speed2-forward*forward);
  }
  double *out=metrics+8*b;out[0]=cost;out[1]=reason;out[2]=minimum;out[3]=gate;out[4]=ph;out[5]=changed;out[6]=k;out[7]=k>0||!reason?1.:0.;
 }
}
