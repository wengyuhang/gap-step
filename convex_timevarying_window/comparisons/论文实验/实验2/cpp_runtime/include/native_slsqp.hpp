#pragma once
#include <casadi/casadi.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>

extern "C" void slsqp_(int*,int*,int*,int*,double*,double*,double*,double*,double*,double*,double*,
    double*,int*,int*,double*,int*,int*,int*,double*,double*,double*,double*,double*,double*,
    double*,double*,double*,double*,int*,int*,int*,int*,int*,int*,int*,int*);

namespace native_slsqp {
using casadi::DM;
using casadi::Function;
struct Result {
    std::vector<double> x, constraints;
    double objective=0., seconds=0., violation=INFINITY;
    int mode=0, iterations=0, evaluations=0, derivatives=0;
    double coordinate_scale_min=1.,coordinate_scale_max=1.;
};
inline std::string status(int mode) {
    switch(mode) {
        case 0:return "Solve_Succeeded";
        case 2:return "Too_Many_Equalities";
        case 3:return "LSQ_Iteration_Limit";
        case 4:return "Inequality_Constraints_Incompatible";
        case 5:return "Singular_Matrix_E";
        case 6:return "Singular_Matrix_C";
        case 7:return "Rank_Deficient_Equality_Subproblem";
        case 8:return "Positive_Directional_Derivative";
        case 9:return "Maximum_Iterations_Exceeded";
        case -100:return "Nonfinite_Evaluation";
        default:return "SLSQP_Mode_"+std::to_string(mode);
    }
}
inline Result minimize(const Function& values,const Function& derivatives,const DM& parameters,
                       std::vector<double> x,int meq,int maximum=5000,double tolerance=1e-8,
                       const std::function<void(int,double)>& progress={},
                       const std::vector<double>& input_lower={},const std::vector<double>& input_upper={},
                       bool equilibrate=false) {
    using Clock=std::chrono::steady_clock;
    const auto began=Clock::now();
    int n=x.size(),m=values.size1_out(1),la=std::max(1,m),mode=0,iterations=maximum-1;
    const int np=n+1,mineq=m-meq+2*np;
    int lw=(3*np+m)*(np+1)+(np-meq+1)*(mineq+2)+2*mineq+(np+mineq)*(np-meq)+2*meq+np+np*n/2+2*m+3*n+3*np+1;
    int ljw=mineq;
    std::vector<double> w(lw,0.),lower(n,NAN),upper(n,NAN),c(la),g(n+1),a(la*(n+1));
    if(!input_lower.empty())for(int k=0;k<n;++k)if(std::isfinite(input_lower[k]))lower[k]=input_lower[k];
    if(!input_upper.empty())for(int k=0;k<n;++k)if(std::isfinite(input_upper[k]))upper[k]=input_upper[k];
    // This is a solver-coordinate change only. The trajectory is still fitted
    // independently from the actual PVAJ values passed to the CasADi graph.
    const auto origin=x;
    std::vector<double> diagonal(n,1.);
    if(equilibrate) {
        const auto initial=derivatives(std::vector<DM>{DM(origin),parameters});
        const auto initial_gradient=initial[0].get_elements(),initial_jacobian=initial[1].get_elements();
        for(int j=0;j<n;++j) {
            double size=std::max(1.,std::sqrt(std::abs(initial_gradient[j])));
            for(int i=0;i<m;++i)size=std::max(size,std::abs(initial_jacobian[j*m+i]));
            if(!std::isfinite(size))throw std::runtime_error("Nonfinite initial equilibration");
            diagonal[j]=std::max(1e-6,1./size);
            if(std::isfinite(lower[j]))lower[j]=(lower[j]-origin[j])/diagonal[j];
            if(std::isfinite(upper[j]))upper[j]=(upper[j]-origin[j])/diagonal[j];
            x[j]=0.;
        }
    }
    auto physical=[&]() {auto actual=x;if(equilibrate)for(int j=0;j<n;++j)actual[j]=origin[j]+diagonal[j]*x[j];return actual;};
    std::vector<int> jw(ljw,0);
    double f=0.,alpha=0.,f0=0.,gs=0.,h1=0.,h2=0.,h3=0.,h4=0.,t=0.,t0=0.,tol=0.;
    int iexact=0,incons=0,ireset=0,itermx=0,line=0,n1=0,n2=0,n3=0;
    Result result;
    auto evaluate=[&](bool jacobian) {
      try {
        const auto v=values(std::vector<DM>{DM(physical()),parameters});
        f=static_cast<double>(v[0]);
        const auto constraints=v[1].get_elements();
        std::copy(constraints.begin(),constraints.end(),c.begin());++result.evaluations;
        bool finite=std::isfinite(f) && std::all_of(c.begin(),c.end(),[](double z){return std::isfinite(z);});
        if(jacobian) {
            const auto d=derivatives(std::vector<DM>{DM(physical()),parameters});
            const auto gradient=d[0].get_elements(),jac=d[1].get_elements();
            for(int j=0;j<n;++j)g[j]=gradient[j]*diagonal[j];g[n]=0.;
            for(int column=0;column<n;++column)for(int row=0;row<m;++row)a[column*la+row]=jac[column*m+row]*diagonal[column];
            ++result.derivatives;
            finite=finite && std::all_of(g.begin(),g.end(),[](double z){return std::isfinite(z);}) &&
                std::all_of(a.begin(),a.end(),[](double z){return std::isfinite(z);});
        }
        return finite;
      } catch(const std::exception&) { return false; }
    };
    if(!evaluate(true)){mode=-100;iterations=0;}
    else {
        int last=0;
        for(;;) {
            slsqp_(&m,&meq,&la,&n,x.data(),lower.data(),upper.data(),&f,c.data(),g.data(),a.data(),
                &tolerance,&iterations,&mode,w.data(),&lw,jw.data(),&ljw,&alpha,&f0,&gs,&h1,&h2,&h3,&h4,
                &t,&t0,&tol,&iexact,&incons,&ireset,&itermx,&line,&n1,&n2,&n3);
            if(progress && iterations>last){progress(iterations,f);last=iterations;}
            if(std::abs(mode)!=1)break;
            if(!evaluate(mode==-1)){mode=-100;break;}
        }
    }
    if(mode!=-100 && !evaluate(false))mode=-100;
    result.coordinate_scale_min=*std::min_element(diagonal.begin(),diagonal.end());
    result.coordinate_scale_max=*std::max_element(diagonal.begin(),diagonal.end());
    result.x=physical();result.mode=mode;result.iterations=iterations;result.objective=f;
    result.constraints.assign(c.begin(),c.begin()+m);result.violation=0.;
    for(int row=0;row<m;++row)result.violation=std::max(result.violation,row<meq?std::abs(c[row]):std::max(0.,-c[row]));
    if(mode==-100)result.violation=INFINITY;
    result.seconds=std::chrono::duration<double>(Clock::now()-began).count();
    return result;
}
}
