#pragma once
#include "geometry.hpp"
#include <numeric>

struct Trajectory {
    double start=0.;
    std::vector<double> durations;
    std::vector<Curve> coefficients;
    double end()const{return start+std::accumulate(durations.begin(),durations.end(),0.);}
    Vector sample(double at,int derivative=0)const {
        if(at<start-1e-9 || at>end()+1e-9)throw std::runtime_error("Trajectory extrapolation");
        size_t piece=0;double begin=start;
        while(piece+1<durations.size() && at>=begin+durations[piece])begin+=durations[piece++];
        const double u=(at-begin)/durations[piece];Vector value=Vector::Zero();
        for(int power=7;power>=derivative;--power) {
            double multiplier=1.;for(int k=0;k<derivative;++k)multiplier*=power-k;
            value=u*value+multiplier*coefficients[piece].row(power).transpose();
        }
        return value/std::pow(durations[piece],derivative);
    }
    static Trajectory load(const Json& input,double start=0.) {
        Trajectory result;result.start=start;result.durations=input.at("durations").get<std::vector<double>>();
        for(const auto& rows:input.at("coefficients")) {
            Curve curve;for(int i=0;i<8;++i)for(int j=0;j<3;++j)curve(i,j)=rows[i][j];result.coefficients.push_back(curve);
        }
        return result;
    }
};
inline Json curve_json(const Curve& curve) {
    Json result=Json::array();for(int i=0;i<8;++i)result.push_back({curve(i,0),curve(i,1),curve(i,2)});return result;
}
inline Json state_json(const Eigen::Matrix<double,3,4>& state) {
    Json result=Json::array();for(int i=0;i<3;++i)result.push_back({state(i,0),state(i,1),state(i,2),state(i,3)});return result;
}
