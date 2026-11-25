#pragma once
#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include "json.hpp"
#include "drolib/solver/minco_snap.hpp"
// Solver metric only. The physical trajectory still uses independently
// optimized full PVAJ. No minimum-snap baseline is added to that trajectory.
class MincoTangentMetric final : public casadi::Callback {
    casadi::Function placement_,gn_,exact_;
    bool absolute_curvature_=false;
    int n_,m_,nv_,nq_,np_,ng_;
    std::vector<double> units_;
    drolib::PVAJ head_,tail_;
public:
    void set_exact_hessian(const casadi::Function& exact,bool absolute_curvature=false) { exact_=exact;absolute_curvature_=absolute_curvature; }
    MincoTangentMetric(const casadi::Function& placement,const casadi::Function& gn,
                      const nlohmann::json& manifest,const nlohmann::json& scene,int np,int ng):
      placement_(placement),gn_(gn),n_(manifest["n"]),m_(manifest["m"]),nv_(manifest["nvar"]),
      nq_(manifest["velocity_start"]),np_(np),ng_(ng),units_(manifest["state_scale"].get<std::vector<double>>()) {
        head_.setZero();tail_.setZero();for(int axis=0;axis<3;++axis){head_(axis,0)=scene["start"][axis];tail_(axis,0)=scene["goal"][axis];}
        construct("minimum_snap_tangent_metric");
    }
    Eigen::VectorXd interpolated_states(const casadi::DM& x,const casadi::DM& p) const {
        auto data=placement_(std::vector<casadi::DM>{x,p})[0].get_elements();
        Eigen::Map<const Eigen::VectorXd> times(data.data(),n_);Eigen::Map<const Eigen::Matrix3Xd> points(data.data()+n_,3,m_);
        drolib::MincoSnap minco;minco.setConditions(head_,tail_,n_);minco.setParameters(points,times);const auto c=minco.getCoeffs();
        Eigen::VectorXd result(9*m_);
        for(int order=1;order<=3;++order)for(int gate=0;gate<m_;++gate)for(int axis=0;axis<3;++axis){double value=0.;
            for(int power=7;power>=order;--power){double factor=1.;for(int k=0;k<order;++k)factor*=power-k;value=value*times[gate]+factor*c(8*gate+power,axis);}
            const int row=3*((order-1)*m_+gate)+axis;result[row]=value/units_[row];}
        return result;
    }
    Eigen::MatrixXd tangent(const casadi::DM& x,const casadi::DM& p) const {
        const auto placed=placement_(std::vector<casadi::DM>{x,p});const auto data=placed[0].get_elements(),jac=placed[1].get_elements();
        Eigen::Map<const Eigen::VectorXd> times(data.data(),n_);Eigen::Map<const Eigen::Matrix3Xd> points(data.data()+n_,3,m_);
        const Eigen::Map<const Eigen::MatrixXd> chain(jac.data(),n_+3*m_,nv_);
        drolib::MincoSnap minco;minco.setConditions(head_,tail_,n_);minco.setParameters(points,times);const auto c=minco.getCoeffs();
        Eigen::MatrixXd b(9*m_,nq_);Eigen::MatrixX3d gc=Eigen::MatrixX3d::Zero(8*n_,3);
        Eigen::VectorXd gt(n_),partial(n_);Eigen::Matrix3Xd gp(3,m_);
        for(int order=1;order<=3;++order)for(int gate=0;gate<m_;++gate)for(int axis=0;axis<3;++axis){
            gc.setZero();partial.setZero();
            for(int power=order;power<8;++power){double factor=1.;for(int k=0;k<order;++k)factor*=power-k;gc(8*gate+power,axis)=factor*std::pow(times[gate],power-order);}
            for(int power=7;power>=order+1;--power){double factor=1.;for(int k=0;k<order+1;++k)factor*=power-k;partial[gate]=partial[gate]*times[gate]+factor*c(8*gate+power,axis);}
            minco.propagateGrad(gc,partial,gp,gt);
            Eigen::VectorXd gradient(n_+3*m_);gradient.head(n_)=gt;gradient.tail(3*m_)=Eigen::Map<const Eigen::VectorXd>(gp.data(),3*m_);
            const int row=3*((order-1)*m_+gate)+axis;
            b.row(row)=gradient.transpose()*chain.leftCols(nq_)/units_[row];
        }
        if(!b.allFinite())throw std::runtime_error("Nonfinite MINCO tangent preconditioner");
        return b;
    }
    casadi_int get_n_in() override{return 4;}casadi_int get_n_out() override{return 1;}
    std::string get_name_in(casadi_int k) override{return std::vector<std::string>{"x","p","lam_f","lam_g"}.at(k);}
    std::string get_name_out(casadi_int) override{return "triu_hess_gamma_x_x";}
    casadi::Sparsity get_sparsity_in(casadi_int k) override{return casadi::Sparsity::dense(k==0?nv_:k==1?np_:k==2?1:ng_,1);}
    casadi::Sparsity get_sparsity_out(casadi_int) override{std::vector<casadi_int> r,c;for(int j=0;j<nv_;++j)for(int i=0;i<=j;++i){r.push_back(i);c.push_back(j);}return casadi::Sparsity::triplet(nv_,nv_,r,c);}
    std::vector<casadi::DM> eval(const std::vector<casadi::DM>& arg) const override {
        const Eigen::MatrixXd b=tangent(arg[0],arg[1]);
        Eigen::MatrixXd h=Eigen::MatrixXd::Identity(nv_,nv_);
        h.topLeftCorner(nq_,nq_)+=b.transpose()*b;h.topRightCorner(nq_,nv_-nq_)=-b.transpose();h.bottomLeftCorner(nv_-nq_,nq_)=-b;
        if(!exact_.is_null()) {
            const auto values=exact_(arg)[0].get_elements();
            const Eigen::MatrixXd original=Eigen::Map<const Eigen::MatrixXd>(values.data(),nv_,nv_);
            Eigen::MatrixXd transform=Eigen::MatrixXd::Identity(nv_,nv_);
            transform.bottomLeftCorner(nv_-nq_,nq_)=b;
            Eigen::MatrixXd pulled=transform.transpose()*original*transform;
            pulled=(.5*(pulled+pulled.transpose())).eval();
            const double scale=std::max(1.,pulled.cwiseAbs().maxCoeff());
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(pulled/scale);
            if(eig.info()!=Eigen::Success)throw std::runtime_error("MINCO metric eigendecomposition failed");
            Eigen::VectorXd spectrum=eig.eigenvalues()*scale;
            if(absolute_curvature_)spectrum=spectrum.cwiseAbs().eval();
            spectrum=spectrum.cwiseMax(1.).eval();
            transform.bottomLeftCorner(nv_-nq_,nq_)=-b;
            const Eigen::MatrixXd factor=eig.eigenvectors().transpose()*transform;
            h=factor.transpose()*spectrum.asDiagonal()*factor;
        } else if(!gn_.is_null()){const auto values=gn_(std::vector<casadi::DM>{arg[0],arg[1]})[0].get_elements();h+=double(arg[2])*Eigen::Map<const Eigen::MatrixXd>(values.data(),nv_,nv_);}
        if(!h.allFinite())throw std::runtime_error("Nonfinite MINCO metric Hessian");
        auto result=casadi::DM::zeros(const_cast<MincoTangentMetric*>(this)->get_sparsity_out(0));auto& out=result.nonzeros();size_t k=0;
        for(int j=0;j<nv_;++j)for(int i=0;i<=j;++i)out[k++]=h(i,j);return {result};
    }
};
