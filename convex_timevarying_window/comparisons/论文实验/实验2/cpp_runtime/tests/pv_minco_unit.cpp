#include "native_minco_pvaj.hpp"
#include <fstream>
#include <iostream>
using casadi::DM;using casadi::MX;using casadi::Function;
std::vector<double> pv_coordinates(const Json& meta,int slack=0) {
    auto x=meta["initials"][0]["original"].get<std::vector<double>>();
    const int n=meta["n"],vs=meta["velocity_start"],nv=meta["nvar"];
    const auto scale=meta["state_scale"].get<std::vector<double>>();
    for(int k=0;k<n;++k){const double t=std::exp(x[k]);x[k]=t>1.?std::sqrt(2.*t-1.)-1.:1.-std::sqrt(2./t-1.);}
    for(int k=vs;k<nv;++k)x[k]/=scale[k-vs];
    x.resize(nv+slack,0.);return x;
}
int main(int argc,char** argv) {
 try {
    if(argc!=5)return 2;
    Json full,scene,source,model;
    std::ifstream(std::string(argv[1])+"/manifest.json")>>full;
    std::ifstream(std::string(argv[1])+"/exchange_dynamics.json")>>model;
    std::ifstream(argv[2])>>scene;std::ifstream(argv[3])>>source;
    native_minco::remove_domain_bounds(full);native_minco::use_direct_point_coordinates(full);
    Json pv=full;native_minco::use_derivative_order(pv,1);pv["dynamic_residual_scale"]=model["residual_scale"];
    std::vector<Window> windows;for(const auto& row:pv["windows"])windows.push_back(load_window(row));
    const int n=pv["n"],m=pv["m"],nv=pv["nvar"],vs=pv["velocity_start"],slack=n*33*6;
    const DM parameters(pv["initials"][0]["parameters"].get<std::vector<double>>());
    native_minco::Graph original(full,scene,windows),changed(pv,scene,windows);
    const auto old_values=original.trajectory(std::vector<DM>{DM(pv_coordinates(full)),parameters});
    const auto values=changed.trajectory(std::vector<DM>{DM(pv_coordinates(pv)),parameters});
    const auto old_coeff=old_values[2].get_elements(),new_coeff=values[2].get_elements(),times=values[1].get_elements();
    double coefficient_error=0.,source_error=0.,time_error=0.;
    for(int piece=0;piece<n;++piece) {
        time_error=std::max(time_error,std::abs(times[piece]-source["durations"][piece].get<double>()));
        for(int axis=0;axis<3;++axis)for(int power=0;power<8;++power) {
            const int index=24*piece+8*axis+power;
            coefficient_error=std::max(coefficient_error,std::abs(new_coeff[index]-old_coeff[index])/(1.+std::abs(old_coeff[index])));
            const double expected=source["coefficients"][piece][power][axis];
            source_error=std::max(source_error,std::abs(new_coeff[index]-expected)/(1.+std::abs(expected)));
        }
    }
    const auto dynamics=Function::load(std::string(argv[1])+"/exchange_dynamic_node.casadi");
    native_minco::Graph soft(pv,scene,windows,dynamics,1000.);
    auto x=pv_coordinates(pv,slack);
    if(pv["free_points"].get<bool>())for(int gate=0;gate<m;++gate) {
        const int start=pv["slices"][gate]["begin"];
        if(windows[gate].circle){x[start]=.05*(windows[gate].opening_radius+.085-pv["finite_radius"].get<double>());x[start+1]=.7+.15*gate;}
        else{x[start]+=.01;x[start+1]-=.01;}
    }
    const auto residual=soft.residuals(std::vector<DM>{DM(x),parameters})[0].get_elements();
    const auto scaling=pv["slack_scale"].get<std::vector<double>>();
    for(int k=0;k<slack;++k)x[nv+k]=std::max(residual[k],0.)/scaling[k]+.2;
    std::vector<double> weights(soft.expressions[6].numel());
    for(size_t k=0;k<weights.size();++k)weights[k]=.001*std::sin(.19*k+.4);
    const MX objective=soft.expressions[5]+MX::dot(DM(weights),soft.expressions[6]);
    const Function probe("pv_objective_gradient_probe",{soft.expressions[0],soft.expressions[1]},
        {objective,MX::gradient(objective,soft.expressions[0])});
    const auto gradient=probe(std::vector<DM>{DM(x),parameters})[1].get_elements();
    double gradient_error=0.;
    for(int group=0;group<4;++group) {
        auto plus=x,minus=x;double automatic=0.;constexpr double h=1e-6;
        for(size_t k=0;k<x.size();++k) {
            const bool use=(group==0 && k<static_cast<size_t>(n)) || (group==1 && k>=static_cast<size_t>(n) && k<static_cast<size_t>(vs)) ||
                (group==2 && k>=static_cast<size_t>(vs) && k<static_cast<size_t>(nv)) || (group==3 && k>=static_cast<size_t>(nv));
            const double direction=use?std::sin(.21*k+.5*group):0.;
            plus[k]+=h*direction;minus[k]-=h*direction;automatic+=gradient[k]*direction;
        }
        const double finite=(static_cast<double>(probe(std::vector<DM>{DM(plus),parameters})[0])-static_cast<double>(probe(std::vector<DM>{DM(minus),parameters})[0]))/(2*h);
        gradient_error=std::max(gradient_error,std::abs(finite-automatic)/(1.+std::abs(automatic)));
    }
    drolib::PVAJ head=drolib::PVAJ::Zero(),tail=drolib::PVAJ::Zero();
    for(int axis=0;axis<3;++axis){head(axis,0)=scene["start"][axis];tail(axis,0)=scene["goal"][axis];}
    native_minco::CoefficientJacobian jacobian("pv_nonlocal_jacobian_probe",head,tail,n,1);
    const auto pvals=values[3].get_elements(),states=pv["initials"][0]["original"].get<std::vector<double>>();
    std::vector<double> physical=times;physical.insert(physical.end(),pvals.begin(),pvals.end());physical.insert(physical.end(),states.begin()+vs,states.end());
    const DM jac=jacobian(std::vector<DM>{DM(physical),values[2]})[0];
    const double nonlocal=std::abs(static_cast<double>(jac(7,n+3*m+3*(m-1))));
    const auto pattern=native_minco::coefficient_sparsity(n,1);
    const bool passed=coefficient_error<1e-8 && source_error<1e-8 && time_error<1e-10 && gradient_error<2e-5 &&
        nv==(pv["free_points"].get<bool>()?43:29) && pv["state_scale"].size()==21 && nonlocal>1e-12;
    const Json result{{"passed",passed},{"derivative_order",1},{"trajectory_variables",nv},{"velocity_variables",21},
        {"initial_coefficient_relative_error",coefficient_error},{"initial_source_relative_error",source_error},
        {"initial_duration_error_s",time_error},{"gradient_relative_error",gradient_error},
        {"nonlocal_last_velocity_to_first_piece",nonlocal},{"coefficient_jacobian_declared_nonzeros",pattern.nnz()}};
    std::ofstream(argv[4])<<result.dump(2)<<'\n';std::cout<<result.dump(2)<<'\n';return passed?0:1;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
