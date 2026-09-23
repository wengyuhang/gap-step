// Independent implementation of arXiv:2509.14726 Eq.2-7, NOT author code.
// ENU world, FLU body. x500 rotor order/moments are from the local SDF model.
#include <cmath>
#include <algorithm>
#include <cstring>
#include <omp.h>

constexpr double mass=2.0643076923076924, gravity=9.8066, maxT=8.54858;
constexpr double inertia[3]={.02383948,.02394241,.04399995};
constexpr double mixer[3][4]={{-.174,.174,.174,-.174},{-.174,.174,-.174,.174},{-.016,-.016,.016,.016}};
inline double sq(double x){return x*x;}
inline void rotation(const double *q,double *r){
    double w=q[0],x=q[1],y=q[2],z=q[3];
    r[0]=1-2*(y*y+z*z); r[1]=2*(x*y-w*z); r[2]=2*(x*z+w*y);
    r[3]=2*(x*y+w*z); r[4]=1-2*(x*x+z*z); r[5]=2*(y*z-w*x);
    r[6]=2*(x*z-w*y); r[7]=2*(y*z+w*x); r[8]=1-2*(x*x+y*y);
}
inline void derivative(const double *s,const double *u,double *d){
    double r[9]; rotation(s+6,r); double thrust=0,tau[3]={};
    for(int j=0;j<4;j++){
        double f=std::clamp(s[13+j],0.,maxT); thrust+=f;
        for(int a=0;a<3;a++) tau[a]+=mixer[a][j]*f;
        d[13+j]=u[j];
    }
    for(int a=0;a<3;a++){d[a]=s[3+a];d[3+a]=r[3*a+2]*thrust/mass-(a==2?gravity:0.);}
    double w=s[6],x=s[7],y=s[8],z=s[9],p=s[10],q=s[11],rr=s[12];
    d[6]=-.5*(x*p+y*q+z*rr); d[7]=.5*(w*p+y*rr-z*q);
    d[8]=.5*(w*q+z*p-x*rr); d[9]=.5*(w*rr+x*q-y*p);
    d[10]=(tau[0]-(inertia[2]-inertia[1])*q*rr)/inertia[0];
    d[11]=(tau[1]-(inertia[0]-inertia[2])*rr*p)/inertia[1];
    d[12]=(tau[2]-(inertia[1]-inertia[0])*p*q)/inertia[2];
}
inline void step(double *s,const double *u,double dt){
    double a[17],b[17],c[17],d[17],tmp[17];
    derivative(s,u,a); for(int j=0;j<17;j++)tmp[j]=s[j]+dt*.5*a[j];
    derivative(tmp,u,b); for(int j=0;j<17;j++)tmp[j]=s[j]+dt*.5*b[j];
    derivative(tmp,u,c); for(int j=0;j<17;j++)tmp[j]=s[j]+dt*c[j];
    derivative(tmp,u,d); for(int j=0;j<17;j++)s[j]+=dt*(a[j]+2*b[j]+2*c[j]+d[j])/6.;
    double norm=std::sqrt(sq(s[6])+sq(s[7])+sq(s[8])+sq(s[9]));
    for(int j=6;j<10;j++)s[j]/=std::max(norm,1e-12);
    for(int j=13;j<17;j++)s[j]=std::clamp(s[j],0.,maxT);
}
inline void ctbr_derivative(const double *s,const double *u,double *d){
    double zeros[4]={};derivative(s,zeros,d);
    for(int a=0;a<3;a++)d[10+a]=std::clamp((u[1+a]-s[10+a])/.05,-50.,50.);
    for(int j=0;j<4;j++)d[13+j]=(u[0]/4-s[13+j])/.025;
    // Rotor in-plane drag from x500 SDF, converted to acceleration.
    double r[9];rotation(s+6,r);double axial=0,drag=0;
    for(int a=0;a<3;a++)axial+=r[3*a+2]*s[3+a];
    for(int j=0;j<4;j++)drag+=8.06428e-5*std::sqrt(std::max(0.,s[13+j])/8.54858e-6)/mass;
    for(int a=0;a<3;a++)d[3+a]-=drag*(s[3+a]-axial*r[3*a+2]);
}
inline void ctbr_step(double *s,const double *u,double dt){
    // Two RK4 substeps resolve the 25 ms collective response at dt=30 ms.
    for(int sub=0;sub<2;sub++){
        double h=dt/2,a[17],b[17],c[17],d[17],tmp[17];
        ctbr_derivative(s,u,a);for(int j=0;j<17;j++)tmp[j]=s[j]+h*.5*a[j];
        ctbr_derivative(tmp,u,b);for(int j=0;j<17;j++)tmp[j]=s[j]+h*.5*b[j];
        ctbr_derivative(tmp,u,c);for(int j=0;j<17;j++)tmp[j]=s[j]+h*c[j];
        ctbr_derivative(tmp,u,d);for(int j=0;j<17;j++)s[j]+=h*(a[j]+2*b[j]+2*c[j]+d[j])/6;
        double norm=std::sqrt(sq(s[6])+sq(s[7])+sq(s[8])+sq(s[9]));for(int j=6;j<10;j++)s[j]/=norm;
    }
}
inline void local(const double *s,const double *pose,double *q){
    for(int j=0;j<3;j++){q[j]=0;for(int a=0;a<3;a++)q[j]+=pose[3+3*a+j]*(s[a]-pose[a]);}
}
// geom[gate, 32]: type(0 circle), radius, count, offset polygon vertices.
inline double frame_distance(const double *q,const double *g){
    double ds=1e20;
    if(g[0]==0) ds=sq(std::hypot(q[0],q[1])-g[1]);
    else for(int j=0;j<(int)g[2];j++){
        int next=(j+1)%(int)g[2]; double ax=g[3+2*j],ay=g[4+2*j];
        double ex=g[3+2*next]-ax,ey=g[4+2*next]-ay;
        double t=std::clamp(((q[0]-ax)*ex+(q[1]-ay)*ey)/(ex*ex+ey*ey),0.,1.);
        ds=std::min(ds,sq(q[0]-ax-t*ex)+sq(q[1]-ay-t*ey));
    }
    return std::sqrt(ds+q[2]*q[2]);
}
inline bool contains(const double *q,const double *g){
    if(g[0]==0)return std::hypot(q[0],q[1])<=g[1]-.065;
    double area=0;
    for(int j=0;j<(int)g[2];j++){int n=(j+1)%(int)g[2];area+=g[3+2*j]*g[4+2*n]-g[4+2*j]*g[3+2*n];}
    for(int j=0;j<(int)g[2];j++){
        int n=(j+1)%(int)g[2]; double ax=g[3+2*j],ay=g[4+2*j];
        double ex=g[3+2*n]-ax,ey=g[4+2*n]-ay;
        double cross=ex*(q[1]-ay)-ey*(q[0]-ax);
        if(cross*(area>=0?1.:-1.)<.065*std::hypot(ex,ey))return false;
    }
    return true;
}
extern "C" void rollout(int M,int K,double dt,const double *initial,const double *controls,
    const double *poses,const double *geometry,const double *signs,const double *previous,
    const int *previous_gates,int first,int mode,double near_weight,double near_range,double penalty,
    double *costs,double *states,int *indices){
    #pragma omp parallel for num_threads(4)
    for(int m=0;m<M;m++){
        double s[17];std::memcpy(s,initial,17*sizeof(double));double cost=0;int gate=first;
        if(states)std::memcpy(states+m*(K+1)*17,s,17*sizeof(double));
        if(indices)indices[m*(K+1)]=gate;
        for(int k=0;k<K;k++){
            const double *pose=poses+(k*8+gate)*12;
            const double *next_pose=poses+((k+1)*8+gate)*12;
            double before[3],after[3],old[3];local(s,pose,before);
            // Compare progress against the SAME time-indexed waypoint so gate
            // motion alone is not counted as vehicle progress.
            double distance_before=0;for(int a=0;a<3;a++)distance_before+=sq(s[a]-next_pose[a]);
            const double *u=controls+(m*K+k)*4;
            if(mode)ctbr_step(s,u,dt);else step(s,u,dt);local(s,next_pose,after);
            double distance_after=sq(after[0])+sq(after[1])+sq(after[2]);
            cost+=distance_after-distance_before;
            if(gate<7 && std::abs(after[2])<near_range)cost+=near_weight*distance_after;
            if(gate==7)cost+=.6*distance_after+.8*(sq(s[3])+sq(s[4])+sq(s[5]));
            if(gate<7){
                local(previous+(k+1)*17,next_pose,old);
                bool temporal=(signs[gate]*old[2]>=0 || previous_gates[k+1]>gate);
                if(signs[gate]*before[2]>=0 && signs[gate]*after[2]<0 && contains(after,geometry+gate*32) && temporal)gate++;
            }
            double speed=std::sqrt(sq(s[3])+sq(s[4])+sq(s[5]));
            cost+=penalty*sq(std::max(0.,speed-12.));
            double r[9];rotation(s+6,r); double thrust=s[13]+s[14]+s[15]+s[16];
            cost+=penalty*sq(std::max(0.,thrust-4*maxT*.9));
            double acceleration=std::sqrt(sq(r[2]*thrust/mass)+sq(r[5]*thrust/mass)+sq(r[8]*thrust/mass-gravity));
            cost+=penalty*sq(std::max(0.,acceleration-8.));
            cost+=penalty*sq(std::max(0.,.743144825-r[8])); // 42 degree tilt
            for(int a=0;a<3;a++)cost+=penalty*sq(std::max(0.,std::abs(s[10+a])-3.49));
            double lower[3]={-28,-30,.65},upper[3]={28,22,12};
            for(int a=0;a<3;a++)cost+=penalty*(sq(std::max(0.,lower[a]-s[a]))+sq(std::max(0.,s[a]-upper[a])));
            for(int j=0;j<7;j++){
                double q[3];local(s,poses+((k+1)*8+j)*12,q);
                if(std::abs(q[2])<.82094472)cost+=penalty*sq(std::max(0.,.82094472-frame_distance(q,geometry+j*32)));
            }
            if(states)std::memcpy(states+(m*(K+1)+k+1)*17,s,17*sizeof(double));
            if(indices)indices[m*(K+1)+k+1]=gate;
        }
        costs[m]=std::isfinite(cost)?cost:1e30;
    }
}
extern "C" void integrate(double *state,const double *control,double dt){step(state,control,dt);}
extern "C" void integrate_ctbr(double *state,const double *control,double dt){ctbr_step(state,control,dt);}
