#pragma once

#include <casadi/casadi.hpp>
#include "dynamics_exchange.hpp"
#include "minimum_thrust.hpp"

namespace stable_dynamics {

// C2 extension on x >= 0. It equals x for x >= floor and stays >= floor/2.
inline casadi::SX positive_extension(const casadi::SX& x,double floor) {
    const casadi::SX t=x/floor;
    return casadi::SX::if_else(x<=0.,.5*floor,casadi::SX::if_else(x<floor,
        floor*(.5+t*t*t-.5*t*t*t*t),x));
}

inline casadi::Function instantaneous(const dynamics_exchange::Model& model,bool norm_form=false,bool lifted=false) {
    using casadi::SX;
    const SX state=SX::sym("flatness_derivatives",9);
    const SX a=state(casadi::Slice(0,3))+SX(casadi::DM(std::vector<double>{0.,0.,model.gravity}));
    const SX j=state(casadi::Slice(3,6)),snap=state(casadi::Slice(6,9));
    const double minimum_acceleration=minimum_thrust::collective_floor/model.mass;
    const double minimum_squared=minimum_acceleration*minimum_acceleration;
    const SX actual_squared=SX::sumsqr(a);
    const SX lift=SX::sym("positive_thrust_acceleration");
    const SX r=lifted ? lift : SX::sqrt(positive_extension(actual_squared,minimum_squared));
    const SX r2=r*r,r3=r2*r,r5=r3*r2,projection=SX::dot(a,j);
    const SX direction=a/r,rate=j/r-a*projection/r3;
    const SX acceleration=(snap-direction*SX::dot(direction,snap))/r
        -(2.*j*projection+a*SX::sumsqr(j))/r3+3.*a*projection*projection/r5;
    const SX x=direction(0),y=direction(1),z=direction(2);
    const SX dx=rate(0),dy=rate(1),dz=rate(2);
    const SX ddx=acceleration(0),ddy=acceleration(1),ddz=acceleration(2);
    const SX actual_denominator=1.+z;
    const SX denominator=positive_extension(actual_denominator,.001);
    const SX twist=y*dx-x*dy,d2=denominator*denominator;
    const SX omega=SX::vertcat({-dy+y*dz/denominator,dx-x*dz/denominator,twist/denominator});
    const SX alpha=SX::vertcat({
        -ddy+y*ddz/denominator+dy*dz/denominator-y*dz*dz/d2,
        ddx-x*ddz/denominator-dx*dz/denominator+x*dz*dz/d2,
        (y*ddx-x*ddy)/denominator-twist*dz/d2});
    const SX inertia(casadi::DM(std::vector<double>(model.inertia.begin(),model.inertia.end())));
    const SX torque=inertia*alpha+SX::cross(omega,inertia*omega);
    const SX wrench=SX::vertcat({model.mass*r,torque});
    std::vector<SX> regular{omega(0)*omega(0)+omega(1)*omega(1)-model.rate_limits[0],
        omega(2)*omega(2)-model.rate_limits[1]};
    for(int rotor=0;rotor<4;++rotor) {
        SX force=0.;for(int k=0;k<4;++k)force+=model.mixer[rotor][k]*wrench(k);
        regular.push_back((force-model.midpoint)*(force-model.midpoint)-model.midpoint*model.midpoint);
    }
    // Match the historical chart branch on the original regular thrust domain.
    const SX total=model.mass*r-4.*model.midpoint;
    const SX fallback=SX::vertcat({(j(0)*j(0)+j(1)*j(1))/r2-model.rate_limits[0],SX(-1.),
        total*total-16.*model.midpoint*model.midpoint,SX(-1.),SX(-1.),SX(-1.)});
    const SX selected=norm_form ? SX::vertcat(regular) :
        SX::if_else(SX::abs(actual_denominator)>.001,SX::vertcat(regular),fallback);
    std::vector<SX> residuals;
    for(int k=0;k<6;++k) {
        const SX value=selected(k)/model.scale[k];
        residuals.push_back(norm_form ? SX::sqrt(1.+value+.01)-std::sqrt(1.01) : value);
    }
    residuals.push_back(lifted ? (r*r-actual_squared)/(model.gravity*model.gravity) :
        (minimum_squared-actual_squared)/(model.gravity*model.gravity));
    const SX output=SX::vertcat(residuals);
    return casadi::Function("stable_flatness_residuals",lifted ? std::vector<SX>{state,lift} : std::vector<SX>{state},
        {output},{{"cse",true}});
}

inline casadi::Function segment_node(const dynamics_exchange::Model& model,bool norm_form=false,bool lifted=false) {
    using casadi::SX;
    const SX state=SX::sym("stable_segment",25),u=SX::sym("stable_fraction");
    const SX coefficients=SX::reshape(state(casadi::Slice(0,24)),8,3);
    std::vector<SX> derivatives;
    for(int order=2;order<=4;++order)for(int axis=0;axis<3;++axis) {
        SX value=0.;
        for(int power=7;power>=order;--power) {
            double factor=1.;for(int d=0;d<order;++d)factor*=power-d;
            value=value*u+factor*coefficients(power,axis);
        }
        derivatives.push_back(value/SX::pow(state(24),order));
    }
    const auto kernel=instantaneous(model,norm_form,lifted);
    const SX force=SX::sym("scaled_thrust_acceleration");
    const auto values=lifted ? kernel(std::vector<SX>{SX::vertcat(derivatives),model.gravity*force}) :
        kernel(std::vector<SX>{SX::vertcat(derivatives)});
    return casadi::Function("stable_exchange_node",lifted ? std::vector<SX>{state,u,force} : std::vector<SX>{state,u},
        {values[0]},
        {{"cse",true},{"never_inline",true}});
}

} // namespace stable_dynamics
