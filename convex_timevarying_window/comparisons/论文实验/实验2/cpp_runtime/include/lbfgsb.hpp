#pragma once
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>
extern "C" void setulb(int,int,double*,double*,double*,int*,double*,double*,double,double,double*,int*,int*,int*,int*,double*,int,int*);

struct LBFGSBOptions {
    int memory=10,max_linesearch=20,max_iterations=2147483647,past=0;
    double ftol=0.,gtol=0.,past_tolerance=0.;
};
struct LBFGSBResult {
    std::vector<double> x,gradient;
    double objective=std::numeric_limits<double>::infinity();
    int iterations=0,evaluations=0,status=0,message=0;
    bool success=false,past_stop=false;
};

inline LBFGSBResult minimize_lbfgsb(
    const std::function<double(const std::vector<double>&,std::vector<double>&)>& evaluate,
    const std::vector<double>& initial,const std::vector<double>& lower,const std::vector<double>& upper,
    const LBFGSBOptions& options) {
    const int n=initial.size(),m=options.memory;
    LBFGSBResult result;result.x=initial;result.gradient.resize(n);
    std::vector<double> l=lower,u=upper,work(2*m*n+11*m*m+5*n+8*m,0.),past(options.past,0.);
    std::vector<int> bounds(n,0),integer_work(3*n,0);
    for(int k=0;k<n;++k) {
        if(std::isfinite(l[k]) && std::isfinite(u[k]))bounds[k]=2;
        else if(std::isfinite(l[k]))bounds[k]=1;
        else if(std::isfinite(u[k]))bounds[k]=3;
    }
    int task[2]={0,0},lsave[4]={0},isave[44]={0},line_task[2]={0,0};double dsave[29]={0.};
    while(true) {
        setulb(n,m,result.x.data(),l.data(),u.data(),bounds.data(),&result.objective,result.gradient.data(),
            options.ftol/2.220446049250313e-16,options.gtol,work.data(),integer_work.data(),task,
            lsave,isave,dsave,options.max_linesearch,line_task);
        if(task[0]==3) {
            result.objective=evaluate(result.x,result.gradient);++result.evaluations;
            if(result.evaluations==1 && options.past)past[0]=result.objective;
        } else if(task[0]==1) {
            ++result.iterations;
            if(options.past) {
                const int slot=result.iterations%options.past;
                if(result.iterations>=options.past && std::abs(past[slot]-result.objective)/std::max(1.,std::abs(result.objective))<options.past_tolerance) {
                    result.past_stop=true;result.success=true;break;
                }
                past[slot]=result.objective;
            }
            if(result.iterations>=options.max_iterations){task[0]=5;task[1]=504;}
        } else {result.success=task[0]==4;break;}
    }
    result.status=task[0];result.message=task[1];return result;
}
