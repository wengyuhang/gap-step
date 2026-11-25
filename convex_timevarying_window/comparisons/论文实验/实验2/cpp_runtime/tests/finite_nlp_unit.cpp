#include "finite_nlp.hpp"
#include "native_slsqp.hpp"
#include <iostream>
using casadi::MX;using casadi::DM;using casadi::Function;
int main(){try{
 const MX x=MX::sym("x",3),p=MX::sym("p",1);
 const MX f=MX::pow(x(0)-p(0),2)+MX::pow(x(1),2)+MX::pow(x(2)-1.,2);
 const MX g=MX::vertcat({x(0)+x(1),x(0),x(1),x(2),x(0)-x(2)});
 finite_nlp::Model model(x,p,f,g,{2.,.25,-INFINITY,-.4,-INFINITY},{2.,INFINITY,1.75,.4,INFINITY});
 const std::vector<double> initial{.8,1.2,0.},lb{0.,0.,-INFINITY},ub{1.2,INFINITY,INFINITY};
 const DM par(std::vector<double>{3.});
 const auto raw=model.values(std::vector<DM>{DM(initial),par});const auto d=model.derivatives(std::vector<DM>{DM(initial),par});
 for(double unit:{1.,1000.}){
  const auto view=model.slsqp(unit);const auto converted=view.values(std::vector<DM>{DM(initial),par}),cd=view.derivatives(std::vector<DM>{DM(initial),par});
  const auto original=raw[1].get_elements(),c=converted[1].get_elements(),j=d[1].get_elements(),cj=cd[1].get_elements();
  if(view.equalities!=1 || c.size()!=5)throw std::runtime_error("Bound conversion count");
  for(int k=0;k<c.size();++k){if(std::abs(c[k]-view.signs[k]*original[view.original_rows[k]]-view.offsets[k])>1e-14)throw std::runtime_error("Bound conversion value");for(int col=0;col<3;++col)if(std::abs(cj[col*c.size()+k]-view.signs[k]*j[col*5+view.original_rows[k]])>1e-14)throw std::runtime_error("Bound conversion derivative");}
  const auto result=native_slsqp::minimize(view.values,view.derivatives,par,initial,view.equalities,200,1e-10/unit,{},lb,ub);
  if(result.mode!=0 || result.violation>1e-8 || std::abs(result.objective*unit-4.24)>1e-7)throw std::runtime_error("SLSQP common finite problem optimum");
 }
 casadi::Dict options{{"ipopt.print_level",0},{"ipopt.sb","yes"},{"print_time",false},{"ipopt.tol",1e-10},{"ipopt.bound_relax_factor",0.}};
 if(const char* backend=std::getenv("SIP_TEST_LINEAR_SOLVER"))options["ipopt.linear_solver"]=std::string(backend);
 if(const char* search=std::getenv("SIP_TEST_LINE_SEARCH"))options["ipopt.line_search_method"]=std::string(search);
 const auto ipopt=casadi::nlpsol("common_ipopt_unit","ipopt",model.formulation(),options);
 auto solved=ipopt(casadi::DMDict{{"x0",DM(initial)},{"p",par},{"lbg",DM(model.lower)},{"ubg",DM(model.upper)},{"lbx",DM(lb)},{"ubx",DM(ub)}});
 if(std::abs(double(solved.at("f"))-4.24)>1e-7)throw std::runtime_error("IPOPT common finite problem optimum");
 // A zero-iteration probe tests initialization, not optimization success.
 // Supplying a previously converged bound-active point must not materially
 // alter its primal coordinates when preserving the warm start.
 auto warm_options=options;warm_options["ipopt.warm_start_init_point"]="yes";warm_options["ipopt.max_iter"]=0;
 for(const char* key:{"ipopt.warm_start_bound_push","ipopt.warm_start_bound_frac","ipopt.warm_start_slack_bound_push","ipopt.warm_start_slack_bound_frac","ipopt.warm_start_mult_bound_push"})warm_options[key]=1e-9;
 const auto warm_probe=casadi::nlpsol("warm_preservation_probe","ipopt",model.formulation(),warm_options);
 const auto initialized=warm_probe(casadi::DMDict{{"x0",solved.at("x")},{"lam_x0",solved.at("lam_x")},{"lam_g0",solved.at("lam_g")},{"p",par},{"lbg",DM(model.lower)},{"ubg",DM(model.upper)},{"lbx",DM(lb)},{"ubx",DM(ub)}});
 for(double v:(initialized.at("x")-solved.at("x")).get_elements())if(std::abs(v)>1e-8)throw std::runtime_error("Warm initialization changed converged primal");
 const MX qf=MX::pow(x(0)-1.,2)+MX::pow(x(1)-2.,2)+MX::pow(x(2)-3.,2),qg=x(0)+x(1);
 finite_nlp::Model quadratic(x,p,qf,qg,{1.},{INFINITY});
 DM transform=DM::eye(3);transform(0,0)=.01;transform(0,1)=.003;transform(1,1)=.2;
 const DM origin(std::vector<double>{1000.,-1000.,.25});const MX y=MX::sym("centered_variables",3);
 const auto mapped=quadratic.values(std::vector<MX>{origin+MX::mtimes(transform,y),p});
 // Check the exact affine Hessian pullback on a nonlinear constrained model.
 const MX nonlinear_f=MX::sin(x(0)*x(1))+MX::pow(x(2),4);
 const MX nonlinear_g=MX::vertcat({x(0)*x(2),MX::cos(x(1))});
 const MX lf=MX::sym("lf"),lg=MX::sym("lg",2);
 const Function original_h("original_h",{x,p,lf,lg},{MX::hessian(lf*nonlinear_f+MX::dot(lg,nonlinear_g),x)},{{"never_inline",true}});
 const Function nonlinear_values("nonlinear_values",{x,p},{nonlinear_f,nonlinear_g});
 const MX affine=origin+MX::mtimes(transform,y);
 const auto nv=nonlinear_values(std::vector<MX>{affine,p});
 const MX automatic=MX::hessian(lf*nv[0]+MX::dot(lg,nv[1]),y);
 const MX pulled=MX::mtimes(transform.T(),MX::mtimes(original_h(std::vector<MX>{affine,p,lf,lg})[0],transform));
 const Function hessian_error("hessian_error",{y,p,lf,lg},{automatic-pulled});
 const auto he=hessian_error(std::vector<DM>{DM(std::vector<double>{.7,-.2,.9}),par,DM(.8),DM(std::vector<double>{.4,-.3})})[0];
 for(double v:he.get_elements())if(std::abs(v)>1e-10)throw std::runtime_error("Explicit affine Hessian pullback differs from AD");
 const auto centered=casadi::nlpsol("centered_equivalence_unit","ipopt",casadi::MXDict{{"x",y},{"p",p},{"f",mapped[0]},{"g",mapped[1]}},options);
 const auto cy=centered(casadi::DMDict{{"x0",DM::zeros(3)},{"p",par},{"lbg",DM(1.)},{"ubg",DM(INFINITY)},{"lbx",DM(std::vector<double>{-INFINITY,-INFINITY,-INFINITY})},{"ubx",DM(std::vector<double>{INFINITY,INFINITY,1.25})}});
 const DM original=origin+DM::mtimes(transform,cy.at("x"));
 const auto deriv=quadratic.derivatives(std::vector<DM>{original,par});
 const DM stationarity=deriv[0]+DM::mtimes(deriv[1].T(),cy.at("lam_g"))+DM::solve(transform.T(),cy.at("lam_x"));
 double error=0.;for(double v:stationarity.get_elements())error=std::max(error,std::abs(v));
 if(centered.stats().at("return_status").to_string()!="Solve_Succeeded" || std::abs(double(cy.at("f"))-2.25)>1e-7 || error>1e-7)throw std::runtime_error("Centered full affine coordinates or dual pullback failed");
 std::cout<<"common finite NLP IPOPT/SLSQP bounds, derivatives, objectives passed\n";return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
