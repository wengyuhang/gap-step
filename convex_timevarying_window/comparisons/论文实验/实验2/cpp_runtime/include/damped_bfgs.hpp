#pragma once
#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

// Powell-damped BFGS: a solver Hessian approximation, never an objective term.
class DampedBfgs final : public casadi::Callback {
    casadi::Function derivatives_;
    int nx_,np_,ng_;
    mutable Eigen::MatrixXd b_;
    mutable Eigen::LLT<Eigen::MatrixXd> factor_;
    mutable std::vector<double> previous_;
public:
    DampedBfgs(const casadi::Function& derivatives,int nx,int np,int ng):
      derivatives_(derivatives),nx_(nx),np_(np),ng_(ng),b_(Eigen::MatrixXd::Identity(nx,nx)),factor_(b_) {
        construct("powell_damped_bfgs");
    }
    casadi_int get_n_in() override{return 4;}
    casadi_int get_n_out() override{return 1;}
    std::string get_name_in(casadi_int k) override{return std::vector<std::string>{"x","p","lam_f","lam_g"}.at(k);}
    std::string get_name_out(casadi_int) override{return "triu_hess_gamma_x_x";}
    casadi::Sparsity get_sparsity_in(casadi_int k) override{return casadi::Sparsity::dense(k==0?nx_:k==1?np_:k==2?1:ng_,1);}
    casadi::Sparsity get_sparsity_out(casadi_int) override {
        std::vector<casadi_int> rows,columns;
        for(int c=0;c<nx_;++c)for(int r=0;r<=c;++r){rows.push_back(r);columns.push_back(c);}
        return casadi::Sparsity::triplet(nx_,nx_,rows,columns);
    }
    std::vector<casadi::DM> eval(const std::vector<casadi::DM>& args) const override {
        using casadi::DM;
        const auto x=args[0].get_elements();
        const double objective_multiplier=static_cast<double>(args[2]);
        if(objective_multiplier>0.) {
            if(!previous_.empty()) {
                const Eigen::Map<const Eigen::VectorXd> now(x.data(),nx_),old(previous_.data(),nx_);
                const Eigen::VectorXd s=now-old;
                if(s.norm()>1e-14*(1.+old.norm())) {
                    auto lag_gradient=[&](const DM& point) {
                        const auto d=derivatives_(std::vector<DM>{point,args[1]});
                        return (objective_multiplier*d[0]+DM::mtimes(d[1].T(),args[3])).get_elements();
                    };
                    const auto gn=lag_gradient(args[0]),go=lag_gradient(DM(previous_));
                    Eigen::VectorXd y(nx_);for(int k=0;k<nx_;++k)y[k]=gn[k]-go[k];
                    const Eigen::VectorXd ls=factor_.matrixU()*s;
                    const Eigen::VectorXd bs=factor_.matrixL()*ls;const double sbs=ls.squaredNorm(),sy=s.dot(y);
                    if(!y.allFinite() || !(sbs>0.) || !std::isfinite(sbs))throw std::runtime_error("Invalid damped-BFGS curvature");
                    const double theta=sy>=.2*sbs?1.:.8*sbs/(sbs-sy);
                    const Eigen::VectorXd r=theta*y+(1.-theta)*bs;
                    const double sr=s.dot(r);
                    // Factor updates avoid subtracting two huge nearly equal
                    // dense matrices. Skip a numerically singular downdate.
                    auto next=factor_;
                    next.rankUpdate(r/std::sqrt(sr),1.);
                    next.rankUpdate(bs/std::sqrt(sbs),-1.);
                    if(next.info()==Eigen::Success)factor_=next;
                    b_=factor_.matrixL().toDenseMatrix()*factor_.matrixU().toDenseMatrix();
                    if(!b_.allFinite())throw std::runtime_error("Nonfinite damped-BFGS matrix");
                }
            }
            previous_=x;
        }
        // The restoration phase does not supply an original-objective secant.
        auto pattern=const_cast<DampedBfgs*>(this)->get_sparsity_out(0);
        DM result=DM::zeros(pattern);auto& entries=result.nonzeros();size_t k=0;
        for(int c=0;c<nx_;++c)for(int r=0;r<=c;++r)entries[k++]=b_(r,c);
        return {result};
    }
};
