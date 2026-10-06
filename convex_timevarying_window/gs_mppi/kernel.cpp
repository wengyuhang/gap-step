#include <cmath>
#include <algorithm>
namespace gs {
double square(double x){return x*x;}
double dot(const double *a,const double *b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
void ball(const double *v,double limit,double *a){
 double scale=std::min(1.,limit/std::max(1e-12,std::sqrt(dot(v,v))));
 for(int j=0;j<3;j++)a[j]=v[j]*scale;
}
void local_state(const double *x,const double *pose,double *q,double *v,double *bias){
 for(int j=0;j<3;j++){
  q[j]=v[j]=bias[j]=0.;
  for(int a=0;a<3;a++){
   q[j]+=pose[3+3*a+j]*(x[a]-pose[a]);
   v[j]+=pose[3+3*a+j]*(x[3+a]-pose[12+a])+pose[15+3*a+j]*(x[a]-pose[a]);
   bias[j]+=-pose[3+3*a+j]*pose[24+a]+2*pose[15+3*a+j]*(x[3+a]-pose[12+a])+pose[27+3*a+j]*(x[a]-pose[a]);
  }
 }
}
// Stable online log-sum-exp AND its weighted affine derivative.
void accumulate(double psi,const double *a,double c,double rho,double *m){
 if(psi<m[0]){
  double w=std::exp(-rho*(m[0]-psi));m[1]=m[1]*w+1.;
  for(int j=0;j<3;j++)m[2+j]=m[2+j]*w+a[j];m[5]=m[5]*w+c;m[0]=psi;
 }else{
  double w=std::exp(-rho*(psi-m[0]));m[1]+=w;
  for(int j=0;j<3;j++)m[2+j]+=w*a[j];m[5]+=w*c;
 }
}
// cfg: inflated radius,k1,k2,rho,gamma,amax,vmax,lower3,upper3,frame_offset,slab,padding
void barrier(const double *x,const double *poses,int gates,const double *features,int nf,const double *cfg,double *out){
 double agg[6]={1e30,0.,0.,0.,0.,0.},minimum=1e30;
 for(int gate=0;gate<gates;gate++){
  const double *pose=poses+36*gate;double q[3],v[3],bias[3];local_state(x,pose,q,v,bias);
  for(int i=0;i<nf;i++){
   const double *f=features+8*i;if(int(f[0])!=gate)continue;
   double e[3],r[3],grad[3],world[3],len=0.,proj=0.;
   for(int a=0;a<3;a++){e[a]=f[4+a]-f[1+a];r[a]=q[a]-f[1+a];len+=e[a]*e[a];proj+=r[a]*e[a];}
   double fraction=std::clamp(proj/len,0.,1.);
   for(int a=0;a<3;a++){r[a]-=fraction*e[a];grad[a]=2*r[a];}
   double rad=cfg[0]+f[7],h=dot(r,r)-rad*rad,hdot=dot(grad,v);
   double vv=dot(v,v);if(fraction>0. && fraction<1.)vv-=dot(e,v)*dot(e,v)/len;
   double c=2*vv+dot(grad,bias)+cfg[1]*hdot;
   for(int a=0;a<3;a++){world[a]=0.;for(int j=0;j<3;j++)world[a]+=pose[3+3*a+j]*grad[j];}
   accumulate(hdot+cfg[1]*h,world,c,cfg[3],agg);minimum=std::min(minimum,h);
  }
 }
 for(int axis=0;axis<3;axis++)for(int side=0;side<2;side++){
  double sign=side?-1.:1.,h=side?cfg[10+axis]-x[axis]:x[axis]-cfg[7+axis];
  h-=.35+cfg[15];double a[3]={0.,0.,0.};a[axis]=sign;
  accumulate(sign*x[3+axis]+cfg[1]*h,a,cfg[1]*sign*x[3+axis],cfg[3],agg);minimum=std::min(minimum,h);
 }
 // Relative-degree-one speed barrier; no fictitious hdot term.
 double hs=cfg[6]*cfg[6]-dot(x+3,x+3),as[3];
 for(int a=0;a<3;a++)as[a]=-2*x[3+a];
 accumulate(hs,as,0.,cfg[3],agg);minimum=std::min(minimum,hs);
 out[0]=agg[0]-std::log(agg[1])/cfg[3];
 for(int a=0;a<3;a++)out[1+a]=agg[2+a]/agg[1];out[4]=agg[5]/agg[1];out[5]=minimum;
}
double dual(double lambda,const double *desired,const double *b,const double *cfg,double *a){
 double v[3];for(int j=0;j<3;j++)v[j]=desired[j]+lambda*b[1+j];ball(v,cfg[5],a);
 return dot(b+1,a)+b[4]+cfg[2]*b[0]+lambda*b[0]*b[0]/cfg[4];
}
int filter(const double *desired,const double *b,const double *cfg,double *a,double *info){
 double residual=dual(0.,desired,b,cfg,a),lambda=0.;
 if(residual<0.){
  double hi=1.;int i=0;for(;i<30;i++){if(dual(hi,desired,b,cfg,a)>=0.)break;hi*=2.;}
  if(i==30){info[0]=residual;info[1]=0.;return 1;}
  double lo=0.;for(int j=0;j<24;j++){double mid=.5*(lo+hi);if(dual(mid,desired,b,cfg,a)>=0.)hi=mid;else lo=mid;}
  lambda=hi;residual=dual(lambda,desired,b,cfg,a);
 }
 info[0]=residual;info[1]=lambda*b[0]/cfg[4];
 return !std::isfinite(residual)||residual < -1e-5;
}
void advance(double *x,const double *a,double dt){for(int j=0;j<3;j++){x[j]+=dt*x[3+j]+.5*dt*dt*a[j];x[3+j]+=dt*a[j];}}
bool inside(const double *q,int gate,const double *features,int nf,const double *cfg){
 for(int i=0;i<nf;i++){
  const double *f=features+8*i;if(int(f[0])!=gate)continue;
  if(f[7]>0.)return std::sqrt(q[0]*q[0]+q[1]*q[1])<=std::sqrt(f[1]*f[1]+f[2]*f[2])-cfg[13];
  double ex=f[4]-f[1],ey=f[5]-f[2];
  if(ex*(q[1]-f[2])-ey*(q[0]-f[1])<cfg[13]*std::sqrt(ex*ex+ey*ey)-1e-7)return false;
 }
 return true;
}
}
#ifndef GS_GPU
extern "C" void gs_barrier(const double *x,const double *poses,int gates,const double *features,int nf,const double *cfg,double *out){gs::barrier(x,poses,gates,features,nf,cfg,out);}
extern "C" int gs_filter(const double *desired,const double *b,const double *cfg,double *a,double *info){return gs::filter(desired,b,cfg,a,info);}
#endif
extern "C" void gs_rollout(int batch,int steps,int gates,int nf,const double *initial,const double *controls,const double *reference,const double *poses,const double *features,const double *cfg,const double *signs,int first,int phase,int mode,double dt,double *metrics,double *paths,double *actions){
 for(int b=0;b<batch;b++){
  double x[6];for(int j=0;j<6;j++)x[j]=initial[j];int gate=first,ph=phase,reason=0;double cost=0.,clear=1e30,minB=1e30;
  if(paths)for(int j=0;j<6;j++)paths[j]=x[j];
  for(int k=0;k<steps;k++){
   double barrier[6],a[3],info[2];const double *desired=controls+3*(steps*b+k);
   gs::barrier(x,poses+36*gates*k,gates,features,nf,cfg,barrier);clear=std::min(clear,barrier[5]);minB=std::min(minB,barrier[0]);
   if(gs::filter(desired,barrier,cfg,a,info))reason|=1;
   if(barrier[5]<-1e-5)reason|=2;
   // A negative B is an explicit certificate-domain failure, not a collision.
   if(barrier[0]<-1e-4)reason|=4;
   if(actions)for(int j=0;j<3;j++)actions[3*k+j]=a[j];
   double old[6];for(int j=0;j<6;j++)old[j]=x[j];gs::advance(x,a,dt);
   if(gate<gates){
    double qa[3],qb[3],vv[3],bb[3];gs::local_state(old,poses+36*(k*gates+gate),qa,vv,bb);gs::local_state(x,poses+36*((k+1)*gates+gate),qb,vv,bb);
    double za=signs[gate]*qa[2],zb=signs[gate]*qb[2];
    if(!ph && za<=-cfg[14] && zb>-cfg[14])ph=1;
    if(ph){
     if(zb<=-cfg[14] || (ph==2&&zb<0.))reason|=8;
     if(!gs::inside(qb,gate,features,nf,cfg))reason|=16;
     if(za<=0.&&zb>0.)ph=2;
     if(ph==2&&zb>cfg[14]){gate++;ph=0;}
    }
   }
   if(mode==0){for(int j=0;j<3;j++)cost+=dt*(.4*gs::square(x[j]-reference[9*(k+1)+j])+.15*gs::square(x[3+j]-reference[9*(k+1)+3+j]));}
   else if(gate==first && first<gates){
    double q[3],v[3],bias[3];gs::local_state(x,poses+36*((k+1)*gates+first),q,v,bias);
    cost+=dt*(.3*(q[0]*q[0]+q[1]*q[1])+.3*gs::square(signs[first]*q[2]-cfg[14]-.5)+12.);
   }
   cost+=dt*.02*gs::dot(a,a);
   if(paths)for(int j=0;j<6;j++)paths[6*(k+1)+j]=x[j];
   if(reason)break;
  }
  double last[6];gs::barrier(x,poses+36*gates*steps,gates,features,nf,cfg,last);
  // Early failures already have a reason; do not use their mismatched-time terminal value.
  if(!reason){clear=std::min(clear,last[5]);minB=std::min(minB,last[0]);if(last[5]<-1e-5)reason|=2;if(last[0]<-1e-4)reason|=4;}
  if(mode==0)for(int j=0;j<3;j++)cost+=2*gs::square(x[j]-reference[9*steps+j])+.4*gs::square(x[3+j]-reference[9*steps+3+j]);
  // Each autonomous solve completes the CURRENT gate. Do not suddenly charge
  // the distant next gate's squared distance immediately after this exit:
  // that makes not crossing cheaper than crossing on widely separated tracks.
  if(mode && gate==first && first<gates){
   double q[3],v[3],bias[3];gs::local_state(x,poses+36*(steps*gates+first),q,v,bias);
   cost+=12.+3.*(q[0]*q[0]+q[1]*q[1]+gs::square(signs[first]*q[2]-cfg[14]-.5));
  }
  double *out=metrics+6*b;out[0]=cost;out[1]=reason;out[2]=clear;out[3]=minB;out[4]=gate;out[5]=ph;
 }
}
