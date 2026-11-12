#include "stable_dynamics.hpp"

void require(bool value,const std::string& message) {
    if(!value)throw std::runtime_error(message);
}
int main() {
    try {
        const Json metadata=Json::parse(R"model({"mass": 2.0643076923076924, "inertia": [0.02383948, 0.02394241, 0.04399995], "inverse_mixer": [[0.25, -1.4367816091954024, -1.4367816091954024, -15.624999999999998], [0.25, 1.4367816091954024, 1.4367816091954024, -15.625000000000002], [0.25, 1.4367816091954024, -1.4367816091954024, 15.625000000000002], [0.25, -1.4367816091954024, 1.4367816091954024, 15.625]], "gravity": 9.8066, "rate_squared_limits": [13.305994508549999, 10.996688852975], "rotor_midpoint": 4.0605755, "residual_scale": [13.305994508549999, 10.996688852975, 16.48827339120025, 16.48827339120025, 16.48827339120025, 16.48827339120025]})model");
        const dynamics_exchange::Model model(metadata);
        const auto kernel=stable_dynamics::instantaneous(model);
        const casadi::SX x=casadi::SX::sym("derivatives",9);
        const auto residual=kernel(std::vector<casadi::SX>{x})[0];
        const auto jac=casadi::SX::jacobian(residual,x);
        const auto hessian=casadi::SX::hessian(casadi::SX::sum1(residual),x);
        const casadi::Function checks("stable_kernel_checks",{x},{residual,jac,hessian});
        for(int trial=0;trial<40;++trial) {
            std::vector<double> state(9);for(int k=0;k<9;++k)state[k]=3.*std::sin(.13+trial*1.2+k*.7);
            const auto actual=kernel(std::vector<casadi::DM>{casadi::DM(state)})[0].get_elements();
            const auto expected=model.residuals(dynamics_exchange::Vec<double>{state[0],state[1],state[2]},
                dynamics_exchange::Vec<double>{state[3],state[4],state[5]},
                dynamics_exchange::Vec<double>{state[6],state[7],state[8]});
            for(int k=0;k<6;++k)require(std::abs(actual[k]-expected[k])<1e-9*(1.+std::abs(expected[k])),"Original dynamics parity");
        }
        const auto norm_kernel=stable_dynamics::instantaneous(model,true);
        for(int trial=0;trial<40;++trial) {
            std::vector<double> state(9);for(int k=0;k<9;++k)state[k]=3.*std::sin(.13+trial*1.2+k*.7);
            const auto norm=norm_kernel(std::vector<casadi::DM>{casadi::DM(state)})[0].get_elements();
            const auto squared=kernel(std::vector<casadi::DM>{casadi::DM(state)})[0].get_elements();
            for(int k=0;k<6;++k) {
                const double recovered=std::pow(norm[k]+std::sqrt(1.01),2)-1.01;
                require(std::abs(recovered-squared[k])<1e-9*(1.+std::abs(squared[k])),"Equivalent norm constraints");
            }
        }
        const auto lifted_kernel=stable_dynamics::instantaneous(model,true,true);
        for(int trial=0;trial<20;++trial) {
            std::vector<double> state(9);for(int k=0;k<9;++k)state[k]=3.*std::sin(.13+trial*1.2+k*.7);
            const double force=std::sqrt(state[0]*state[0]+state[1]*state[1]+std::pow(state[2]+model.gravity,2));
            const auto lifted=lifted_kernel(std::vector<casadi::DM>{casadi::DM(state),casadi::DM(force)})[0].get_elements();
            const auto original=norm_kernel(std::vector<casadi::DM>{casadi::DM(state)})[0].get_elements();
            for(int k=0;k<6;++k)require(std::abs(lifted[k]-original[k])<1e-10,"Lifted thrust physical equivalence");
            require(std::abs(lifted[6])<1e-12,"Thrust algebraic link");
        }
        for(double force_fraction:{0.,1e-8,.25,.9,1.,1.1}) {
            std::vector<double> state{0.,0.,-model.gravity+force_fraction*minimum_thrust::collective_floor/model.mass,
                3.,-2.,1.,20.,-10.,4.};
            for(const auto& value:checks(std::vector<casadi::DM>{casadi::DM(state)}))
                for(double number:value.get_elements())require(std::isfinite(number),"Finite free-fall extension through Hessian");
        }
        const double floor=std::pow(minimum_thrust::collective_floor/model.mass,2);
        const casadi::SX q=casadi::SX::sym("q");const auto extension=stable_dynamics::positive_extension(q,floor);
        const casadi::SX first=casadi::SX::jacobian(extension,q),second=casadi::SX::jacobian(first,q);
        const casadi::Function smooth("extension_derivatives",{q},{extension,first,second});
        const auto boundary=smooth(std::vector<casadi::DM>{floor});
        require(std::abs(double(boundary[0])-floor)<1e-12 && std::abs(double(boundary[1])-1.)<1e-12 && std::abs(double(boundary[2]))<1e-12,"C2 boundary identity");
        std::cout<<"Original-domain parity, finite derivatives and C2 extension checks passed\n";
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
