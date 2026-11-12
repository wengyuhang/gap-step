#include "native_minco_pvaj.hpp"
#include <fstream>
#include <iostream>
using casadi::DM;using casadi::MX;using casadi::Function;
std::vector<double> finite_initial(const Json& metadata,const Json& seed,int slack=0) {
    auto x=seed["original"].get<std::vector<double>>();
    const int n=metadata["n"],vs=metadata["velocity_start"],nv=metadata["nvar"];
    const auto scale=metadata["state_scale"].get<std::vector<double>>();
    for(int k=0;k<n;++k){const double t=std::exp(x[k]);x[k]=t>1.?std::sqrt(2.*t-1.)-1.:1.-std::sqrt(2./t-1.);}
    for(int k=vs;k<nv;++k)x[k]/=scale[k-vs];x.resize(nv+slack,0.);return x;
}
int main(int argc,char** argv) {
 try {
    if(argc!=4)return 2;Json old,scene,dynamics_metadata;
    std::ifstream(std::string(argv[1])+"/manifest.json")>>old;std::ifstream(argv[2])>>scene;
    std::ifstream(std::string(argv[1])+"/exchange_dynamics.json")>>dynamics_metadata;
    native_minco::remove_domain_bounds(old);Json direct=old;native_minco::use_direct_point_coordinates(direct);
    direct["dynamic_residual_scale"]=dynamics_metadata["residual_scale"];
    std::vector<Window> windows;for(const auto& row:old["windows"])windows.push_back(load_window(row));
    native_minco::Graph baseline(old,scene,windows),changed(direct,scene,windows);
    double coefficient_error=0.,time_error=0.,point_error=0.;
    for(size_t seed=0;seed<old["initials"].size();++seed) {
        auto a=finite_initial(old,old["initials"][seed]);auto b=finite_initial(direct,direct["initials"][seed]);
        const DM p(old["initials"][seed]["parameters"].get<std::vector<double>>());
        const auto expected=baseline.trajectory(std::vector<DM>{DM(a),p});
        const auto actual=changed.trajectory(std::vector<DM>{DM(b),p});
        for(int output:{1,2,3}) {
            const auto x=expected[output].get_elements(),y=actual[output].get_elements();
            for(size_t k=0;k<x.size();++k) {
                const double error=std::abs(x[k]-y[k]);
                if(output==1)time_error=std::max(time_error,error);
                if(output==2)coefficient_error=std::max(coefficient_error,error/(1.+std::abs(x[k])));
                if(output==3)point_error=std::max(point_error,error);
            }
        }
    }
    const auto dynamics=Function::load(std::string(argv[1])+"/exchange_dynamic_node.casadi");
    native_minco::Graph soft(direct,scene,windows,dynamics,1000.);
    const int nv=direct["nvar"],n=direct["n"],vs=direct["velocity_start"],slack=n*33*6;
    auto x=finite_initial(direct,direct["initials"][0],slack);
    const DM p(direct["initials"][0]["parameters"].get<std::vector<double>>());
    for(int gate=0;gate<direct["m"].get<int>();++gate) {
        const int start=direct["slices"][gate]["begin"];
        if(windows[gate].circle){x[start]=.1*(windows[gate].opening_radius+.085-direct["finite_radius"].get<double>());x[start+1]=.7+.2*gate;}
        else {x[start]+=.02;x[start+1]-=.01;}
    }
    const auto residuals=soft.residuals(std::vector<DM>{DM(x),p})[0].get_elements();
    const auto scaling=direct["slack_scale"].get<std::vector<double>>();
    for(int k=0;k<slack;++k)x[nv+k]=std::max(0.,residuals[k])/scaling[k]+.2;
    std::vector<double> weights(soft.expressions[6].numel());
    for(size_t k=0;k<weights.size();++k)weights[k]=.001*std::sin(.17*k+.3);
    const MX value=soft.expressions[5]+MX::dot(DM(weights),soft.expressions[6]);
    const Function probe("direct_point_probe",{soft.expressions[0],soft.expressions[1]},
        {value,MX::gradient(value,soft.expressions[0])});
    const auto gradient=probe(std::vector<DM>{DM(x),p})[1].get_elements();double gradient_error=0.;
    for(int group=0;group<4;++group) {
        auto plus=x,minus=x;double automatic=0.;constexpr double h=1e-6;
        for(size_t k=0;k<x.size();++k) {
            const bool use=(group==0 && k<static_cast<size_t>(n)) || (group==1 && k>=static_cast<size_t>(n) && k<static_cast<size_t>(vs)) ||
                (group==2 && k>=static_cast<size_t>(vs) && k<static_cast<size_t>(nv)) || (group==3 && k>=static_cast<size_t>(nv));
            const double direction=use ? std::sin(.27*k+.4*group) : 0.;
            plus[k]+=h*direction;minus[k]-=h*direction;automatic+=gradient[k]*direction;
        }
        const double finite=(static_cast<double>(probe(std::vector<DM>{DM(plus),p})[0])-static_cast<double>(probe(std::vector<DM>{DM(minus),p})[0]))/(2*h);
        gradient_error=std::max(gradient_error,std::abs(finite-automatic)/(1.+std::abs(automatic)));
    }
    auto axis=finite_initial(direct,direct["initials"][0],slack);
    for(const auto& slice:direct["slices"])if(slice["circle"].get<bool>())axis[slice["begin"].get<int>()+1]=0.;
    const auto gx=soft.point_objective_gradient(std::vector<DM>{DM(axis),p})[0].get_elements();
    for(const auto& slice:direct["slices"])if(slice["circle"].get<bool>())axis[slice["begin"].get<int>()+1]=.5*pi;
    const auto gy=soft.point_objective_gradient(std::vector<DM>{DM(axis),p})[0].get_elements();
    for(const auto& slice:direct["slices"])if(slice["circle"].get<bool>()) {
        const int start=slice["begin"];double theta=std::atan2(-gy[start],-gx[start]);if(theta<0.)theta+=2*pi;axis[start+1]=theta;
    }
    const auto descent=soft.point_objective_gradient(std::vector<DM>{DM(axis),p})[0].get_elements();
    double maximum_radial_slope=-INFINITY;
    for(const auto& slice:direct["slices"])if(slice["circle"].get<bool>())maximum_radial_slope=std::max(maximum_radial_slope,descent[slice["begin"].get<int>()]);
    const bool passed=coefficient_error<1e-8 && time_error<1e-10 && point_error<1e-8 && gradient_error<2e-5 &&
        nv==85 && vs==22 && direct["point_domain_rows"]==19 && changed.expressions[6].numel()==26 && maximum_radial_slope<=1e-7;
    const Json result{{"passed",passed},{"initial_trajectories_checked",old["initials"].size()},
        {"coefficient_relative_error",coefficient_error},{"time_error_s",time_error},{"point_error_m",point_error},
        {"gradient_relative_error",gradient_error},{"maximum_initial_radial_slope",maximum_radial_slope},
        {"trajectory_variables",nv},{"point_domain_rows",direct["point_domain_rows"]},{"auxiliary_variables",slack}};
    std::ofstream(argv[3])<<result.dump(2)<<'\n';std::cout<<result.dump(2)<<'\n';return passed?0:1;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
