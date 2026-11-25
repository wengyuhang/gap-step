#include "damped_bfgs.hpp"
#include <iostream>
int main(){
 try {
  using casadi::MX;using casadi::DM;using casadi::Function;
  for(int task=0;task<2;++task){
   const MX x=MX::sym("x",2),p=MX::sym("p",0);
   const MX f=task==0?.5*MX::sumsqr(x-2.):MX::pow(1.-x(0),2)+100.*MX::pow(x(1)-x(0)*x(0),2);
   const MX g=task==0?MX::sumsqr(x):MX::zeros(0,1);
   const Function derivatives("derivatives",{x,p},{MX::gradient(f,x),MX::jacobian(g,x)});
   DampedBfgs h(derivatives,2,0,g.numel());
   const auto solver=casadi::nlpsol("test","ipopt",casadi::MXDict{{"x",x},{"p",p},{"f",f},{"g",g}},
    {{"hess_lag",Function(h)},{"ipopt.print_level",0},{"ipopt.sb","yes"},{"ipopt.tol",1e-9},{"ipopt.max_iter",1000},{"print_time",false}});
   casadi::DMDict in{{"x0",DM(std::vector<double>{-1.2,1.})}};
   if(task==0){in["lbg"]=-INFINITY;in["ubg"]=1.;}
   const auto out=solver(in);const auto answer=out.at("x").get_elements();
   const double expected=task==0?1./std::sqrt(2.):1.;
   if(!solver.stats().at("success").to_bool() || std::abs(answer[0]-expected)>1e-5 || std::abs(answer[1]-expected)>1e-5)
     throw std::runtime_error("Known constrained/unconstrained optimum not recovered");
  }
  std::cout<<"Damped BFGS constrained quadratic and Rosenbrock passed\n";return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
