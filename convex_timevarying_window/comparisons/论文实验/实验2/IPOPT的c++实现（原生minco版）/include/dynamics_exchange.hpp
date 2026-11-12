#pragma once

#include "numpy_passage.hpp"
#include <array>
#include <chrono>
#include <queue>

namespace dynamics_exchange {

// Numerical interval enclosures. Polynomial conversion uses floating-point
// coefficients, so outcomes are numerical evidence rather than Arb certificates.
struct Interval {
    double lo,hi;
    Interval(double value=0.):lo(value),hi(value) {}
    Interval(double lower,double upper):lo(lower),hi(upper) {}
};
inline Interval outward(double lo,double hi) {
    if(std::isnan(lo)||std::isnan(hi))return {-INFINITY,INFINITY};
    return {std::nextafter(lo,-INFINITY),std::nextafter(hi,INFINITY)};
}
inline Interval operator+(Interval a,Interval b){return outward(a.lo+b.lo,a.hi+b.hi);}
inline Interval operator-(Interval a,Interval b){return outward(a.lo-b.hi,a.hi-b.lo);}
inline Interval operator-(Interval a){return {-a.hi,-a.lo};}
inline Interval operator*(Interval a,Interval b) {
    const std::array<double,4> v{a.lo*b.lo,a.lo*b.hi,a.hi*b.lo,a.hi*b.hi};
    for(double x:v)if(std::isnan(x))return {-INFINITY,INFINITY};
    return outward(*std::min_element(v.begin(),v.end()),*std::max_element(v.begin(),v.end()));
}
inline Interval operator/(Interval a,Interval b) {
    if(b.lo<=0. && b.hi>=0.)return {-INFINITY,INFINITY};
    return a*outward(1./b.hi,1./b.lo);
}
inline Interval squared(Interval a) {
    const double lower=a.lo<=0.&&a.hi>=0.?0.:std::min(a.lo*a.lo,a.hi*a.hi);
    return outward(lower,std::max(a.lo*a.lo,a.hi*a.hi));
}
inline Interval root(Interval a) {
    if(a.hi<0.)return {-INFINITY,INFINITY};
    auto value=outward(std::sqrt(std::max(0.,a.lo)),std::sqrt(a.hi));
    value.lo=std::max(0.,value.lo);return value;
}
inline double squared(double a){return a*a;}
inline double root(double a){return std::sqrt(a);}
inline double lower(double a){return a;}
inline double upper(double a){return a;}

struct Jet {
    Interval value,derivative;
    Jet(double value=0.):value(value),derivative(0.) {}
    Jet(Interval value,Interval derivative):value(value),derivative(derivative) {}
};
inline Jet operator+(Jet a,Jet b){return {a.value+b.value,a.derivative+b.derivative};}
inline Jet operator-(Jet a,Jet b){return {a.value-b.value,a.derivative-b.derivative};}
inline Jet operator-(Jet a){return {-a.value,-a.derivative};}
inline Jet operator*(Jet a,Jet b){return {a.value*b.value,a.derivative*b.value+a.value*b.derivative};}
inline Jet operator/(Jet a,Jet b){return {a.value/b.value,(a.derivative*b.value-a.value*b.derivative)/squared(b.value)};}
inline Jet squared(Jet a){return {squared(a.value),Interval(2.)*a.value*a.derivative};}
inline Jet root(Jet a){const auto r=root(a.value);return {r,a.derivative/(Interval(2.)*r)};}
inline double lower(Jet a){return a.value.lo;}
inline double upper(Jet a){return a.value.hi;}

template<class Scalar> using Vec=std::array<Scalar,3>;
template<class Scalar> Scalar dot(const Vec<Scalar>& a,const Vec<Scalar>& b) {
    Scalar out=0.;for(int k=0;k<3;++k)out=out+a[k]*b[k];return out;
}
template<class Scalar> Scalar norm_squared(const Vec<Scalar>& a) {
    Scalar out=0.;for(auto x:a)out=out+squared(x);return out;
}
template<class Scalar> Vec<Scalar> cross(const Vec<Scalar>& a,const Vec<Scalar>& b) {
    return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
}

struct Model {
    double mass,gravity,midpoint;
    std::array<double,3> inertia;
    std::array<double,2> rate_limits;
    std::array<double,6> scale;
    std::array<std::array<double,4>,4> mixer;
    explicit Model(const Json& metadata):mass(metadata.at("mass")),gravity(metadata.at("gravity")),
        midpoint(metadata.at("rotor_midpoint")),inertia(metadata.at("inertia").get<decltype(inertia)>()),
        rate_limits(metadata.at("rate_squared_limits").get<decltype(rate_limits)>()),
        scale(metadata.at("residual_scale").get<decltype(scale)>()),
        mixer(metadata.at("inverse_mixer").get<decltype(mixer)>()) {}

    template<class Scalar> std::array<Scalar,6> residuals(
            Vec<Scalar> a,const Vec<Scalar>& j,const Vec<Scalar>& snap) const {
        a[2]=a[2]+Scalar(gravity);
        const Scalar magnitude=root(norm_squared(a));
        const Scalar r2=squared(magnitude),r3=r2*magnitude,r5=r3*r2;
        const Scalar projection=dot(a,j);
        Vec<Scalar> direction,rate,acceleration;
        for(int k=0;k<3;++k) {
            direction[k]=a[k]/magnitude;
            rate[k]=j[k]/magnitude-a[k]*projection/r3;
        }
        const Scalar projected_snap=dot(direction,snap),j2=norm_squared(j);
        for(int k=0;k<3;++k)acceleration[k]=(snap[k]-direction[k]*projected_snap)/magnitude
            -(Scalar(2.)*j[k]*projection+a[k]*j2)/r3
            +Scalar(3.)*a[k]*squared(projection)/r5;
        const Scalar denominator=Scalar(1.)+direction[2];
        std::array<Scalar,6> out;
        if(lower(denominator)>.001) {
            const auto& x=direction[0];const auto& y=direction[1];
            const auto& dx=rate[0];const auto& dy=rate[1];const auto& dz=rate[2];
            const auto& ddx=acceleration[0];const auto& ddy=acceleration[1];const auto& ddz=acceleration[2];
            const Scalar twist=y*dx-x*dy,d2=squared(denominator);
            const Vec<Scalar> omega{-dy+y*dz/denominator,dx-x*dz/denominator,twist/denominator};
            const Vec<Scalar> alpha{
                -ddy+y*ddz/denominator+dy*dz/denominator-y*squared(dz)/d2,
                ddx-x*ddz/denominator-dx*dz/denominator+x*squared(dz)/d2,
                (y*ddx-x*ddy)/denominator-twist*dz/d2};
            Vec<Scalar> angular_momentum;for(int k=0;k<3;++k)angular_momentum[k]=Scalar(inertia[k])*omega[k];
            const auto gyroscopic=cross(omega,angular_momentum);
            std::array<Scalar,4> wrench;wrench[0]=Scalar(mass)*magnitude;
            for(int k=0;k<3;++k)wrench[k+1]=Scalar(inertia[k])*alpha[k]+gyroscopic[k];
            out[0]=squared(omega[0])+squared(omega[1])-Scalar(rate_limits[0]);
            out[1]=squared(omega[2])-Scalar(rate_limits[1]);
            for(int rotor=0;rotor<4;++rotor) {
                Scalar thrust=0.;for(int k=0;k<4;++k)thrust=thrust+Scalar(mixer[rotor][k])*wrench[k];
                out[rotor+2]=squared(thrust-Scalar(midpoint))-Scalar(midpoint*midpoint);
            }
        } else if(upper(denominator)<=.001) {
            // Preserve the existing TOGT singular-chart fallback exactly.
            out={ (squared(j[0])+squared(j[1]))/r2-Scalar(rate_limits[0]),Scalar(-1.),
                squared(Scalar(mass)*magnitude-Scalar(4.*midpoint))-Scalar(16.*midpoint*midpoint),
                Scalar(-1.),Scalar(-1.),Scalar(-1.)};
        } else {
            for(auto& value:out)value=Scalar(INFINITY);
        }
        for(int k=0;k<6;++k)out[k]=out[k]/Scalar(scale[k]);
        return out;
    }
};

struct Segment {
    const Model& model;
    Curve coefficients;
    double duration;
    std::array<Curve,4> controls;
    int evaluations=0;
    Segment(const Model& model,const Curve& curve,double duration):model(model),coefficients(curve),duration(duration) {
        for(int order=2;order<=5;++order) {
            Curve power=Curve::Zero();
            for(int k=order;k<8;++k) {
                double factor=1.;for(int d=0;d<order;++d)factor*=k-d;
                power.row(k-order)=curve.row(k)*factor/std::pow(duration,order);
            }
            controls[order-2]=numpy_bernstein(power);
        }
    }
    std::array<double,6> at(double u) {
        ++evaluations;std::array<Vec<double>,3> states;
        for(int order=2;order<=4;++order)for(int axis=0;axis<3;++axis) {
            double value=0.;for(int k=7;k>=order;--k) {
                double factor=1.;for(int d=0;d<order;++d)factor*=k-d;
                value=value*u+coefficients(k,axis)*factor;
            }
            states[order-2][axis]=value/std::pow(duration,order);
        }
        return model.residuals(states[0],states[1],states[2]);
    }
    std::array<double,6> upper_bounds(double left,double right,const std::array<double,6>& midpoint) const {
        std::array<Vec<Interval>,4> ranges;
        for(int order=0;order<4;++order) {
            const Curve local=restrict_interval(controls[order],left,right);
            for(int axis=0;axis<3;++axis) {
                const double guard=2e-12*(1.+local.col(axis).cwiseAbs().maxCoeff());
                ranges[order][axis]=outward(local.col(axis).minCoeff()-guard,local.col(axis).maxCoeff()+guard);
            }
        }
        std::array<Vec<Jet>,3> states;
        for(int order=0;order<3;++order)for(int axis=0;axis<3;++axis)
            states[order][axis]=Jet(ranges[order][axis],ranges[order+1][axis]*Interval(duration));
        const auto values=model.residuals(states[0],states[1],states[2]);
        std::array<double,6> result;
        for(int k=0;k<6;++k) {
            const auto& d=values[k].derivative;
            const double slope=std::max(std::abs(d.lo),std::abs(d.hi));
            const double centered=midpoint[k]+slope*(right-left)/2.+1e-11;
            result[k]=std::min(values[k].value.hi,std::isnan(centered)?INFINITY:centered);
            if(std::isnan(result[k]))result[k]=INFINITY;
        }
        return result;
    }
};

inline Json check_segment(const Model& model,const Curve& curve,double duration,double tolerance=1e-6) {
    const auto began=std::chrono::steady_clock::now();Segment segment(model,curve,duration);
    std::array<double,6> best,where;best.fill(-INFINITY);where.fill(0.);
    std::array<std::vector<double>,6> local_peaks;
    auto observe=[&](double u) {
        const auto values=segment.at(u);
        for(int k=0;k<6;++k)if(values[k]>best[k]){best[k]=values[k];where[k]=u;}
        return values;
    };
    // Locate the visible peaks first. Interval subdivision subsequently checks
    // the whole segment, including narrow peaks missed by these seed brackets.
    std::array<std::array<double,6>,33> grid;
    for(int k=0;k<=32;++k)grid[k]=observe(k/32.);
    for(int component=0;component<6;++component)for(int k=1;k<32;++k)
        if(grid[k][component]>grid[k-1][component] && grid[k][component]>=grid[k+1][component]) {
            double a=(k-1)/32.,b=(k+1)/32.;
            constexpr double ratio=.6180339887498949;
            double c=b-ratio*(b-a),d=a+ratio*(b-a);
            double fc=observe(c)[component],fd=observe(d)[component];
            for(int iter=0;iter<40 && b-a>1e-9;++iter) {
                if(fc>fd){b=d;d=c;fd=fc;c=b-ratio*(b-a);fc=observe(c)[component];}
                else {a=c;c=d;fc=fd;d=a+ratio*(b-a);fd=observe(d)[component];}
            }
            local_peaks[component].push_back((c+d)/2.);
        }
    struct Box {double a,b,priority;std::array<double,6> bounds;};
    struct Compare {bool operator()(const Box& a,const Box& b)const{return a.priority<b.priority;}};
    std::priority_queue<Box,std::vector<Box>,Compare> queue;
    std::array<double,6> verified_upper;verified_upper.fill(-INFINITY);
    auto enqueue=[&](double a,double b) {
        const auto bounds=segment.upper_bounds(a,b,observe((a+b)/2.));
        const double maximum=*std::max_element(bounds.begin(),bounds.end());
        if(maximum<=tolerance)for(int k=0;k<6;++k)verified_upper[k]=std::max(verified_upper[k],bounds[k]);
        else queue.push(Box{a,b,maximum,bounds});
    };
    const bool visible_violation=*std::max_element(best.begin(),best.end())>tolerance;
    if(!visible_violation)enqueue(0.,1.);
    int boxes=0;std::string outcome=visible_violation?"violated":"numerically_enclosed";
    while(!queue.empty() && boxes<65536) {
        const Box box=queue.top();queue.pop();++boxes;
        if(*std::max_element(best.begin(),best.end())>tolerance){outcome="violated";break;}
        if((box.b-box.a)*duration<1e-8){outcome="unresolved";break;}
        const double middle=(box.a+box.b)/2.;enqueue(box.a,middle);enqueue(middle,box.b);
    }
    if(outcome=="numerically_enclosed" && !queue.empty())outcome="unresolved";
    if(*std::max_element(best.begin(),best.end())>tolerance)outcome="violated";
    return Json{{"passed",outcome=="numerically_enclosed"},{"outcome",outcome},
        {"peak_residuals",best},{"peak_fractions",where},{"evaluations",segment.evaluations},{"boxes",boxes},
        {"local_peak_fractions",local_peaks},
        {"upper_bounds",outcome=="numerically_enclosed"?Json(verified_upper):Json(nullptr)},
        {"check_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()},
        {"evidence","floating-point interval and centered-derivative enclosures"}};
}

} // namespace dynamics_exchange
