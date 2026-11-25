#pragma once
#include <casadi/casadi.hpp>
#include <cmath>
#include <stdexcept>
#include <vector>

// One mathematical problem. Backend adapters may reorder/sign-transform its
// bounded constraints but must not rebuild trajectory, dynamics or geometry.
namespace finite_nlp {
using casadi::MX;using casadi::Function;
struct SlsqpView {
    Function values,derivatives;
    int equalities=0;
    std::vector<int> original_rows;
    std::vector<double> signs,offsets;
};
struct Model {
    MX x,p,f,g;
    std::vector<double> lower,upper;
    Function values,derivatives;
    Model(const MX& variables,const MX& parameters,const MX& objective,const MX& constraints,
          const std::vector<double>& lb,const std::vector<double>& ub):
        x(variables),p(parameters),f(objective),g(constraints),lower(lb),upper(ub) {
        if(lower.size()!=g.numel() || upper.size()!=g.numel())
            throw std::runtime_error("Finite NLP bound/constraint dimension mismatch");
        for(int i=0;i<g.numel();++i)
            if(std::isnan(lower[i]) || std::isnan(upper[i]) || lower[i]>upper[i])
                throw std::runtime_error("Invalid finite NLP constraint bounds");
        values=Function("canonical_finite_values",{x,p},{f,g});
        derivatives=Function("canonical_finite_derivatives",{x,p},{MX::gradient(f,x),MX::jacobian(g,x)});
    }
    casadi::MXDict formulation() const{return {{"x",x},{"p",p},{"f",f},{"g",g}};}
    SlsqpView slsqp(double cost_unit=1.) const {
        if(!std::isfinite(cost_unit) || cost_unit<=0.)throw std::runtime_error("Invalid objective unit");
        SlsqpView out;std::vector<MX> eq,ineq;
        auto add=[&](int row,double sign,double offset,bool equality){
            (equality?eq:ineq).push_back(sign*g(row)+offset);
            out.original_rows.push_back(row);out.signs.push_back(sign);out.offsets.push_back(offset);
        };
        for(int i=0;i<g.numel();++i)if(std::isfinite(lower[i]) && lower[i]==upper[i])add(i,1.,-lower[i],true);
        out.equalities=eq.size();
        for(int i=0;i<g.numel();++i)if(!(std::isfinite(lower[i]) && lower[i]==upper[i])){
            if(std::isfinite(lower[i]))add(i,1.,-lower[i],false);
            if(std::isfinite(upper[i]))add(i,-1.,upper[i],false);
        }
        eq.insert(eq.end(),ineq.begin(),ineq.end());const MX c=MX::vertcat(eq);
        out.values=Function("finite_slsqp_values",{x,p},{f/cost_unit,c});
        out.derivatives=Function("finite_slsqp_derivatives",{x,p},{MX::gradient(f/cost_unit,x),MX::jacobian(c,x)});
        return out;
    }
};
} // namespace finite_nlp
