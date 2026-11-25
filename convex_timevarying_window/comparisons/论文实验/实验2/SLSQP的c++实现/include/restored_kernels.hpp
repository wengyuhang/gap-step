#pragma once

#include "native_minco_pvaj.hpp"
#include "dynamics_exchange.hpp"

namespace restored_kernels {

inline casadi::Function dynamic_node(const Json& metadata) {
    using casadi::SX;
    const dynamics_exchange::Model model(metadata);
    const SX state=SX::sym("restored_segment_state",25);
    const SX u=SX::sym("restored_fraction");
    const SX curve=SX::reshape(state(casadi::Slice(0,24)),8,3);
    std::vector<SX> derivatives;
    for(int order=2;order<=4;++order) {
        std::vector<SX> components;
        for(int axis=0;axis<3;++axis) {
            SX value=0.;
            for(int power=7;power>=order;--power) {
                double factor=1.;
                for(int k=0;k<order;++k)factor*=power-k;
                value=value*u+factor*curve(power,axis);
            }
            components.push_back(value/SX::pow(state(24),order));
        }
        derivatives.push_back(SX::vertcat(components));
    }
    const SX a=derivatives[0]+SX(casadi::DM(std::vector<double>{0.,0.,model.gravity}));
    const SX j=derivatives[1],snap=derivatives[2];
    const SX magnitude=SX::sqrt(SX::sumsqr(a));
    const SX projection=SX::dot(a,j);
    const SX direction=a/magnitude;
    const SX rate=j/magnitude-a*projection/SX::pow(magnitude,3);
    const SX acceleration=(snap-direction*SX::dot(direction,snap))/magnitude
        -(2.*j*projection+a*SX::sumsqr(j))/SX::pow(magnitude,3)
        +3.*a*projection*projection/SX::pow(magnitude,5);
    const SX x=direction(0),y=direction(1),z=direction(2);
    const SX dx=rate(0),dy=rate(1),dz=rate(2);
    const SX ddx=acceleration(0),ddy=acceleration(1),ddz=acceleration(2);
    const SX denominator=1.+z,twist=y*dx-x*dy;
    const SX omega=SX::vertcat({-dy+y*dz/denominator,dx-x*dz/denominator,twist/denominator});
    const SX alpha=SX::vertcat({
        -ddy+y*ddz/denominator+dy*dz/denominator-y*dz*dz/(denominator*denominator),
        ddx-x*ddz/denominator-dx*dz/denominator+x*dz*dz/(denominator*denominator),
        (y*ddx-x*ddy)/denominator-twist*dz/(denominator*denominator)});
    const SX inertia(casadi::DM(std::vector<double>(model.inertia.begin(),model.inertia.end())));
    const SX torque=inertia*alpha+SX::cross(omega,inertia*omega);
    const SX wrench=SX::vertcat({model.mass*magnitude,torque});
    std::vector<SX> regular{SX::sumsqr(omega(casadi::Slice(0,2)))-model.rate_limits[0],
                            omega(2)*omega(2)-model.rate_limits[1]};
    for(int rotor=0;rotor<4;++rotor) {
        SX thrust=0.;
        for(int component=0;component<4;++component)thrust+=model.mixer[rotor][component]*wrench(component);
        regular.push_back(SX::pow(thrust-model.midpoint,2)-model.midpoint*model.midpoint);
    }
    const SX collective=model.mass*magnitude-4.*model.midpoint;
    const SX fallback=SX::vertcat({SX::sumsqr(j(casadi::Slice(0,2)))/(magnitude*magnitude)-model.rate_limits[0],
        SX(-1.),collective*collective-16.*model.midpoint*model.midpoint,SX(-1.),SX(-1.),SX(-1.)});
    const SX physical=SX::if_else(SX::abs(denominator)>.001,SX::vertcat(regular),fallback);
    return casadi::Function("restored_dynamic_node",{state,u},
        {physical/SX(casadi::DM(std::vector<double>(model.scale.begin(),model.scale.end())))},
        {{"cse",true},{"never_inline",true}});
}

inline casadi::Function frame_node(const Window& window,const std::string& kind,double radius) {
    using casadi::MX;
    const MX state=MX::sym("restored_frame_state",26),offset=MX::sym("restored_offset");
    const MX duration=state(24),start=state(25);
    const MX clipped=MX::fmin(offset,duration)/duration;
    const MX u=kind=="before" ? 1.-clipped : kind=="after" ? clipped : offset;
    const MX coefficients=MX::reshape(state(casadi::Slice(0,24)),8,3);
    std::vector<MX> components;
    for(int axis=0;axis<3;++axis) {
        MX value=coefficients(7,axis);
        for(int power=6;power>=0;--power)value=value*u+coefficients(power,axis);
        components.push_back(value);
    }
    const MX phase=2.*pi*window.frequency*(start+u*duration)+window.phase;
    MX motion;
    if(window.swing) {
        const MX theta=std::asin(window.amplitude/window.length)*MX::sin(phase);
        motion=MX::vertcat({window.length*(1.-MX::cos(theta)),-window.length*MX::sin(theta),MX(0.)});
    } else motion=MX::vertcat({MX(0.),-window.amplitude*MX::sin(phase),MX(0.)});
    const MX local=MX::mtimes(native_minco::rotation(window).T(),
        MX::vertcat(components)-native_minco::vector3(window.center))-motion;
    const MX value=native_minco::knot_safety(window,local(casadi::Slice(0,2)),radius)
        +local(2)*local(2);
    return casadi::Function("restored_frame_"+kind,{state,offset},{value},{{"cse",true},{"never_inline",true}});
}

} // namespace restored_kernels
