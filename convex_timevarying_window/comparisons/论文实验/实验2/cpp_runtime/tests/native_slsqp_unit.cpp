#include "native_slsqp.hpp"
#include <iostream>
int main() {
    using casadi::MX; using casadi::DM; using casadi::Function;
    const MX x=MX::sym("x",2),p=MX::sym("p",0);
    const MX f=MX::sumsqr(x-DM(std::vector<double>{2.,2.})),g=2.-x(0)-x(1);
    const Function values("values",{x,p},{f,g});
    const Function derivatives("derivatives",{x,p},{MX::gradient(f,x),MX::jacobian(g,x)});
    const auto result=native_slsqp::minimize(values,derivatives,DM::zeros(0,1),{.2,.2},0,100,1e-10,{},
        {0.,0.},{.75,INFINITY});
    const auto scaled=native_slsqp::minimize(values,derivatives,DM::zeros(0,1),{.2,.2},0,100,1e-10,{},
        {0.,0.},{.75,INFINITY},true);
    const Function small_values("small_values",{x,p},{f/1000.,g});
    const Function small_derivatives("small_derivatives",{x,p},{MX::gradient(f/1000.,x),MX::jacobian(g,x)});
    const auto small=native_slsqp::minimize(small_values,small_derivatives,DM::zeros(0,1),{.2,.2},0,100,1e-13,{},
        {0.,0.},{.75,INFINITY},true);
    const bool cost_units=small.mode==0 && small.violation<1e-8 &&
        std::abs(small.x[0]-.75)<1e-7 && std::abs(small.x[1]-1.25)<1e-7 &&
        std::abs(small.objective*1000.-2.125)<1e-8;
    const bool equivalent=cost_units &&scaled.mode==0 && scaled.violation<1e-8 &&
        std::abs(scaled.x[0]-.75)<1e-7 && std::abs(scaled.x[1]-1.25)<1e-7 &&
        std::abs(scaled.objective-2.125)<1e-8;
    const bool passed=equivalent &&result.mode==0 && result.violation<1e-8 &&
        std::abs(result.x[0]-.75)<1e-7 && std::abs(result.x[1]-1.25)<1e-7 &&
        std::abs(result.objective-2.125)<1e-8;
    std::cout<<"native SLSQP bound/constraint regression "<<passed<<'\n';
    return passed?0:1;
}
