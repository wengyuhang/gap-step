#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include "json.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

using Json=nlohmann::json;
using Vector=Eigen::Vector3d;
using Curve=Eigen::Matrix<double,8,3>;
constexpr double pi=3.1415926535897932384626433832795;

Vector evaluate(const Curve& coefficients,double u) {
    Vector value=coefficients.row(7).transpose();
    for (int k=6;k>=0;--k) value=u*value+coefficients.row(k).transpose();
    return value;
}

std::vector<double> roots(const Eigen::VectorXd& polynomial) {
    int degree=static_cast<int>(polynomial.size())-1;
    while (degree>0 && polynomial[degree]==0.) --degree;
    if (degree==0) return {};
    Eigen::MatrixXd companion=Eigen::MatrixXd::Zero(degree,degree);
    for (int k=1;k<degree;++k) companion(k,k-1)=1.;
    companion.col(degree-1)=-polynomial.head(degree)/polynomial[degree];
    companion=companion.reverse().eval();
    Eigen::EigenSolver<Eigen::MatrixXd> solver(companion,false);
    if (solver.info()!=Eigen::Success) throw std::runtime_error("Polynomial eigenproblem failed");
    std::vector<double> values;
    for (const auto& value:solver.eigenvalues())
        if (std::abs(value.imag())<1e-7) values.push_back(value.real());
    std::sort(values.begin(),values.end());
    return values;
}

double binomial(int n,int k) {
    double value=1.;
    for (int i=1;i<=k;++i) value*=static_cast<double>(n-i+1)/i;
    return value;
}

Curve bernstein(const Curve& power) {
    Curve control=Curve::Zero();
    for (int i=0;i<8;++i) for (int k=0;k<=i;++k)
        control.row(i)+=binomial(i,k)/binomial(7,k)*power.row(k);
    return control;
}

std::pair<Curve,Curve> split(Curve work,double fraction) {
    Curve left,right;
    left.row(0)=work.row(0);right.row(7)=work.row(7);
    for (int count=7;count>0;--count) {
        for (int i=0;i<count;++i) work.row(i)=(1.-fraction)*work.row(i)+fraction*work.row(i+1);
        left.row(8-count)=work.row(0);right.row(count-1)=work.row(count-1);
    }
    return {left,right};
}

Curve restrict_interval(Curve control,double lower,double upper) {
    if (lower>0.) control=split(control,lower).second;
    if (upper<1.) control=split(control,(upper-lower)/(1.-lower)).first;
    return control;
}

std::pair<double,double> sine_range(double lower,double upper) {
    if (lower>upper) std::swap(lower,upper);
    double low=std::min(std::sin(lower),std::sin(upper));
    double high=std::max(std::sin(lower),std::sin(upper));
    for (long k=static_cast<long>(std::ceil((lower-pi/2)/pi));k<=std::floor((upper-pi/2)/pi);++k) {
        const double value=k%2==0?1.:-1.;
        low=std::min(low,value);high=std::max(high,value);
    }
    return {low,high};
}

struct Window {
    Vector center;
    Eigen::Matrix3d rotation;
    bool circle,swing;
    double opening_radius,frequency,amplitude,length,phase;
    std::vector<Eigen::Vector2d> physical,frame;
    Vector motion(double at) const {
        const double sine=std::sin(2*pi*frequency*at+phase);
        if (!swing) return Vector(0.,-amplitude*sine,0.);
        const double angle=std::asin(amplitude/length)*sine;
        return Vector(length*(1.-std::cos(angle)),-length*std::sin(angle),0.);
    }
    Vector motion_velocity(double t) const {
        const double phase_at=2.*pi*frequency*t+phase;
        const double rate=2.*pi*frequency;
        if(!swing)return Vector(0.,-amplitude*rate*std::cos(phase_at),0.);
        const double angle=std::asin(amplitude/length)*std::sin(phase_at);
        const double angle_rate=std::asin(amplitude/length)*rate*std::cos(phase_at);
        return Vector(length*std::sin(angle)*angle_rate,-length*std::cos(angle)*angle_rate,0.);
    }
    std::pair<Vector,Vector> bounds(double a,double b) const {
        auto range=sine_range(2*pi*frequency*a+phase,2*pi*frequency*b+phase);
        if (!swing) return {Vector(0.,-amplitude*range.second,0.),Vector(0.,-amplitude*range.first,0.)};
        const double angle=std::asin(amplitude/length);
        auto horizontal=sine_range(angle*range.first,angle*range.second);
        auto vertical=sine_range(angle*range.first+pi/2,angle*range.second+pi/2);
        return {Vector(length*(1.-vertical.second),-length*horizontal.second,0.),
                Vector(length*(1.-vertical.first),-length*horizontal.first,0.)};
    }
    bool contains(const Eigen::Vector2d& q) const {
        if (circle) return q.norm()<=opening_radius+1e-10;
        bool positive=false,negative=false;
        for (size_t i=0;i<physical.size();++i) {
            auto edge=physical[(i+1)%physical.size()]-physical[i];auto delta=q-physical[i];
            const double cross=edge.x()*delta.y()-edge.y()*delta.x();
            positive=positive || cross>1e-10;negative=negative || cross < -1e-10;
        }
        return !(positive&&negative);
    }
    double distance_squared(const Vector& q) const {
        if (circle) return std::pow(q.head<2>().norm()-opening_radius-.085,2)+q.z()*q.z();
        double best=std::numeric_limits<double>::infinity();
        for (size_t i=0;i<frame.size();++i) {
            auto edge=frame[(i+1)%frame.size()]-frame[i];auto delta=q.head<2>()-frame[i];
            double fraction=std::clamp(delta.dot(edge)/edge.squaredNorm(),0.,1.);
            best=std::min(best,(delta-fraction*edge).squaredNorm());
        }
        return best+q.z()*q.z();
    }
};

Window load_window(const Json& row) {
    Window window;
    for(int i=0;i<3;++i) {
        window.center[i]=row.at("center")[i];
        for(int j=0;j<3;++j) window.rotation(i,j)=row.at("rotation")[i][j];
    }
    window.circle=row.at("circle");window.swing=row.at("swing");
    window.frequency=row.at("frequency");window.amplitude=row.at("amplitude");
    window.length=row.at("length");window.phase=row.at("phase");
    window.opening_radius=window.circle?row.at("opening_radius").get<double>():0.;
    if (!window.circle) for(const auto& vertex:row.at("vertices")) {
        Eigen::Vector2d value(vertex[0],vertex[1]);window.physical.push_back(value);
        for(int i=0;i<2;++i) value[i]+=(value[i]>0.?1.:value[i]<0.?-1.:0.)*.085;
        window.frame.push_back(value);
    }
    return window;
}

inline std::vector<Eigen::Vector2d> inset_frame_vertices(const Window& w,double radius) {
    const int count=w.frame.size();
    std::vector<Eigen::Vector2d> normals(count),result(count);
    std::vector<double> limits(count);
    for(int k=0;k<count;++k) {
        const auto edge=(w.frame[(k+1)%count]-w.frame[k]).eval();
        normals[k]=Eigen::Vector2d(edge[1],-edge[0]).normalized();
        limits[k]=normals[k].dot(w.frame[k])-radius;
    }
    for(int k=0;k<count;++k) {
        const int previous=(k+count-1)%count;
        Eigen::Matrix2d pair;pair.row(0)=normals[previous];pair.row(1)=normals[k];
        result[k]=pair.inverse()*Eigen::Vector2d(limits[previous],limits[k]);
    }
    return result;
}

struct Box {double bound,a,b;size_t serial;Curve control;};
struct Compare {bool operator()(const Box& a,const Box& b) const {
    return a.bound==b.bound?a.serial>b.serial:a.bound>b.bound;
}};

Json passage(const Curve& world,double duration,double start,const Window& window,double radius) {
    Curve local=world*window.rotation;
    local.row(0)-=window.center.transpose()*window.rotation;
    auto position=[&](double u)->Vector{return evaluate(local,u)-window.motion(start+u*duration);};
    for(double u:roots(local.col(2))) if(u>1e-6 && u<1.-1e-6 && window.contains(position(u).head<2>()))
        return Json{{"passed",false},{"outcome","repeat_aperture_crossing"},{"witness_u",nullptr},{"boxes",0},{"minimum_sample_distance_m",0.}};
    std::vector<double> cuts{0.,1.};
    for(double level:{-radius,radius}) {
        Eigen::VectorXd polynomial=local.col(2);polynomial[0]-=level;
        for(double u:roots(polynomial)) if(u>0. && u<1.) cuts.push_back(u);
    }
    std::sort(cuts.begin(),cuts.end());cuts.erase(std::unique(cuts.begin(),cuts.end()),cuts.end());
    std::priority_queue<Box,std::vector<Box>,Compare> queue;
    size_t serial=0;double best=std::numeric_limits<double>::infinity(),witness=0.;bool have_witness=false;
    auto enqueue=[&](double a,double b,const Curve& control) {
        auto motion=window.bounds(start+a*duration,start+b*duration);
        Vector low=control.colwise().minCoeff().transpose()-motion.second-Vector::Constant(1e-10);
        Vector high=control.colwise().maxCoeff().transpose()-motion.first+Vector::Constant(1e-10);
        double planar=std::numeric_limits<double>::infinity();
        if(window.circle) {
            Eigen::Vector2d near,far;
            for(int i=0;i<2;++i) {near[i]=std::max({low[i],-high[i],0.});far[i]=std::max(std::abs(low[i]),std::abs(high[i]));}
            const double frame_radius=window.opening_radius+.085;
            planar=std::pow(std::max({near.norm()-frame_radius,frame_radius-far.norm(),0.}),2);
        } else {
            const Eigen::Vector2d center=(low.head<2>()+high.head<2>())/2.;
            const Eigen::Vector2d half=(high.head<2>()-low.head<2>())/2.;
            for(size_t i=0;i<window.frame.size();++i) {
                Eigen::Vector2d edge=window.frame[(i+1)%window.frame.size()]-window.frame[i];
                const double length=edge.norm();Eigen::Vector2d tangent=edge/length,normal(tangent.y(),-tangent.x());
                const double along=tangent.dot(center-window.frame[i]),along_radius=tangent.cwiseAbs().dot(half);
                const double across=normal.dot(center-window.frame[i]),across_radius=normal.cwiseAbs().dot(half);
                const double ag=std::max({-along-along_radius,along-along_radius-length,0.});
                const double ng=std::max(std::abs(across)-across_radius,0.);
                planar=std::min(planar,ag*ag+ng*ng);
            }
        }
        for(double u:{a,(a+b)/2.,b}) {
            const double distance=window.distance_squared(position(u));
            if(distance<best) {best=distance;witness=u;have_witness=true;}
        }
        const double normal=std::max({low.z(),-high.z(),0.});
        queue.push(Box{planar+normal*normal,a,b,++serial,control});
    };
    for(size_t k=1;k<cuts.size();++k) if(std::abs(evaluate(local,(cuts[k-1]+cuts[k])/2.).z())<=radius+1e-9)
        enqueue(cuts[k-1],cuts[k],restrict_interval(bernstein(local),cuts[k-1],cuts[k]));
    int boxes=0;std::string outcome="floating_local_bound";
    while(!queue.empty() && boxes<512) {
        Box item=queue.top();queue.pop();
        if(item.bound>=radius*radius) continue;
        if(best<std::pow(radius-2e-4,2)) {outcome="violated";break;}
        if((item.b-item.a)*duration<1e-6) {outcome="unresolved";break;}
        auto halves=split(item.control,.5);const double middle=(item.a+item.b)/2.;
        enqueue(item.a,middle,halves.first);enqueue(middle,item.b,halves.second);++boxes;
    }
    if(!queue.empty() && outcome=="floating_local_bound") outcome="budget_unresolved";
    return Json{{"passed",outcome=="floating_local_bound"},{"outcome",outcome},
        {"witness_u",have_witness?Json(witness):Json(nullptr)},{"boxes",boxes},
        {"minimum_sample_distance_m",std::isfinite(best)?Json(std::sqrt(best)):Json(nullptr)}};
}

