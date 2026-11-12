// Independent bridge; upstream LBFGS-Lite remains unmodified and MIT licensed.
#include "lbfgs.hpp"
#include <chrono>
#include <limits>
#include <stdexcept>
using Clock = std::chrono::steady_clock;
using Evaluate = double (*)(const double *, double *, int);
struct Deadline {};
struct Context {
    Evaluate callback;
    Clock::time_point deadline;
    Eigen::VectorXd best;
    double value = std::numeric_limits<double>::infinity();
    int calls = 0, iterations = 0;
};
double evaluate(void *opaque, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
    auto &c = *static_cast<Context *>(opaque);
    if (Clock::now() >= c.deadline) throw Deadline{};
    double value = c.callback(x.data(), g.data(), x.size()); ++c.calls;
    if (std::isfinite(value) && g.allFinite() && value < c.value) { c.value=value; c.best=x; }
    if (Clock::now() >= c.deadline) throw Deadline{};
    return value;
}
int progress(void *opaque, const Eigen::VectorXd &, const Eigen::VectorXd &,
             double, double, int k, int) {
    static_cast<Context *>(opaque)->iterations=k; return 0;
}
extern "C" int wang_optimize(int size, double *data, Evaluate callback,
                             double budget_ms, int max_iterations, double epsilon,
                             double delta, double *summary) {
    Context c; c.callback=callback;
    auto began=Clock::now();
    c.deadline=began+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double,std::milli>(budget_ms));
    Eigen::VectorXd x=Eigen::Map<Eigen::VectorXd>(data,size);
    lbfgs::lbfgs_parameter_t p; p.max_iterations=max_iterations;
    p.g_epsilon=epsilon; p.delta=delta;
    double f=std::numeric_limits<double>::infinity(); int status;
    try { status=lbfgs::lbfgs_optimize(x,f,evaluate,nullptr,progress,&c,p); }
    catch (const Deadline &) { status=1001; }
    catch (...) { status=1002; }
    const bool converged = status == lbfgs::LBFGS_CONVERGENCE || status == lbfgs::LBFGS_STOP;
    if (converged) {
        Eigen::Map<Eigen::VectorXd>(data,size)=x;
    } else if (c.best.size()==size) {
        Eigen::Map<Eigen::VectorXd>(data,size)=c.best;
    }
    summary[0]=converged ? f : c.value;
    summary[1]=c.calls; summary[2]=c.iterations;
    summary[3]=std::chrono::duration<double,std::milli>(Clock::now()-began).count();
    summary[4]=converged ? 0.0 : 1.0;
    return status;
}
extern "C" const char *wang_status(int code) {
    if (code==1001) return "budget_exhausted_not_converged";
    if (code==1002) return "native_exception";
    return lbfgs::lbfgs_strerror(code);
}
