#pragma once
#include <casadi/casadi.hpp>
#include "geometry.hpp"
#include "minco_pv.hpp"
#include <memory>

namespace native_minco {
using casadi::DM;
using casadi::MX;
using casadi::Function;
using casadi::Sparsity;

inline Sparsity coefficient_sparsity(int n,int derivative_order=3) {
    const int m=n-1;
    std::vector<casadi_int> rows,columns;
    for(int piece=0;piece<n;++piece)for(int axis=0;axis<3;++axis)
        for(int power=0;power<8;++power) {
            const int row=24*piece+8*axis+power;
            if(derivative_order==3) {
                rows.push_back(row);columns.push_back(piece);
                for(int knot:{piece-1,piece})if(knot>=0 && knot<m) {
                    rows.push_back(row);columns.push_back(n+3*knot+axis);
                    for(int order=0;order<derivative_order;++order) {
                        rows.push_back(row);columns.push_back(n+3*m+3*(order*m+knot)+axis);
                    }
                }
            } else {
                // Eliminated knot derivatives couple every piece along an axis.
                for(int time=0;time<n;++time){rows.push_back(row);columns.push_back(time);}
                for(int knot=0;knot<m;++knot) {
                    rows.push_back(row);columns.push_back(n+3*knot+axis);
                    for(int order=0;order<derivative_order;++order) {
                        rows.push_back(row);columns.push_back(n+3*m+3*(order*m+knot)+axis);
                    }
                }
            }
        }
    return Sparsity::triplet(24*n,n+3*m*(1+derivative_order),rows,columns);
}

// Input contains physical T, world-frame P, and prescribed knot derivatives.
// MincoPV eliminates all remaining knot derivatives through its banded solve.
class CoefficientJacobian final : public casadi::Callback {
    int n_, m_, input_size_,derivative_order_;
    mutable togt_pv::MincoPV minco_;
    Sparsity jacobian_sparsity_;
    std::vector<std::vector<std::pair<int,int>>> row_entries_;
public:
    CoefficientJacobian(const std::string& name, const drolib::PVAJ& head,
                        const drolib::PVAJ& tail, int n,int derivative_order=3)
        : n_(n), m_(n-1), input_size_(n+3*(1+derivative_order)*(n-1)),derivative_order_(derivative_order),minco_(head,tail,n,derivative_order) {
        jacobian_sparsity_=coefficient_sparsity(n_,derivative_order_);
        std::vector<casadi_int> rows,columns;
        jacobian_sparsity_.get_triplet(rows,columns);
        row_entries_.resize(24*n_);
        for(size_t k=0;k<rows.size();++k)row_entries_[rows[k]].push_back({columns[k],k});
        construct(name);
    }
    casadi_int get_n_in() override { return 2; }
    casadi_int get_n_out() override { return 1; }
    Sparsity get_sparsity_in(casadi_int i) override {
        return Sparsity::dense(i==0 ? input_size_ : 24*n_,1);
    }
    Sparsity get_sparsity_out(casadi_int) override {
        return jacobian_sparsity_;
    }
    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        const auto x=arg[0].get_elements();
        Eigen::Map<const Eigen::VectorXd> times(x.data(),n_);
        Eigen::Map<const Eigen::Matrix3Xd> points(x.data()+n_,3,m_);
        Eigen::Map<const Eigen::Matrix3Xd> states(x.data()+n_+3*m_,3,derivative_order_*m_);
        if(!minco_.fit(points,states,times))throw std::runtime_error("MincoPV Jacobian fit failed");
        DM result=DM::zeros(jacobian_sparsity_);
        auto& jac=result.nonzeros();
        Eigen::MatrixX3d partial=Eigen::MatrixX3d::Zero(8*n_,3);
        Eigen::Matrix3Xd gp(3,m_),gd(3,derivative_order_*m_);
        for(int piece=0;piece<n_;++piece)for(int axis=0;axis<3;++axis)
            for(int power=0;power<8;++power) {
                const int row=24*piece+8*axis+power;
                partial.setZero();
                partial(8*piece+power,axis)=std::pow(times[piece],power);
                Eigen::VectorXd gt=Eigen::VectorXd::Zero(n_);
                // Differentiating normalized coefficients adds a direct T term.
                if(power)gt[piece]=power*minco_.coefficients()(8*piece+power,axis)*
                    std::pow(times[piece],power-1);
                minco_.propagate(partial,gp,gd,gt);
                for(const auto& [column,index]:row_entries_[row]) {
                    if(column<n_)jac[index]=gt[column];
                    else if(column<n_+3*m_)jac[index]=gp.data()[column-n_];
                    else jac[index]=gd.data()[column-n_-3*m_];
                }
            }
        return {result};
    }
};

class Coefficients final : public casadi::Callback {
    int n_,m_,derivative_order_;
    drolib::PVAJ head_,tail_;
    mutable togt_pv::MincoPV minco_;
    mutable std::vector<std::unique_ptr<CoefficientJacobian>> derivatives_;
public:
    Coefficients(const drolib::PVAJ& head,const drolib::PVAJ& tail,int n,int derivative_order=3)
        : n_(n),m_(n-1),derivative_order_(derivative_order),head_(head),tail_(tail),minco_(head,tail,n,derivative_order) {
        construct("native_minco_pvaj_coefficients");
    }
    casadi_int get_n_in() override { return 1; }
    casadi_int get_n_out() override { return 1; }
    Sparsity get_sparsity_in(casadi_int) override { return Sparsity::dense(n_+3*(1+derivative_order_)*m_,1); }
    Sparsity get_sparsity_out(casadi_int) override { return Sparsity::dense(24*n_,1); }
    bool has_jacobian() const override { return true; }
    bool has_jac_sparsity(casadi_int,casadi_int) const override { return true; }
    Sparsity get_jac_sparsity(casadi_int,casadi_int,bool) const override {
        return coefficient_sparsity(n_,derivative_order_);
    }
    Function get_jacobian(const std::string& name,const std::vector<std::string>&,
                          const std::vector<std::string>&,const casadi::Dict&) const override {
        auto derivative=std::make_unique<CoefficientJacobian>(name,head_,tail_,n_,derivative_order_);
        Function function=*derivative;
        derivatives_.push_back(std::move(derivative));
        return function;
    }
    std::vector<DM> eval(const std::vector<DM>& arg) const override {
        const auto x=arg[0].get_elements();
        Eigen::Map<const Eigen::VectorXd> times(x.data(),n_);
        Eigen::Map<const Eigen::Matrix3Xd> points(x.data()+n_,3,m_);
        Eigen::Map<const Eigen::Matrix3Xd> states(x.data()+n_+3*m_,3,derivative_order_*m_);
        if(!minco_.fit(points,states,times))throw std::runtime_error("MincoPV fit failed");
        std::vector<double> result(24*n_);
        for(int piece=0;piece<n_;++piece)for(int axis=0;axis<3;++axis)
            for(int power=0;power<8;++power)
                result[24*piece+8*axis+power]=minco_.coefficients()(8*piece+power,axis)*std::pow(times[piece],power);
        return {DM(result)};
    }
};

inline DM vector2(const Eigen::Vector2d& value) {return DM(std::vector<double>{value[0],value[1]});}
inline DM vector3(const Vector& value) {return DM(std::vector<double>{value[0],value[1],value[2]});}
inline DM rotation(const Window& w) {
    DM value=DM::zeros(3,3);
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)value(i,j)=w.rotation(i,j);
    return value;
}
inline std::vector<Eigen::Vector2d> inset_vertices(const Window& w,double radius) {
    return inset_frame_vertices(w,radius);
}
inline MX point_map(const Window& w,const MX& d,double radius) {
    if(w.circle)return 2.*(w.opening_radius+.085-radius)*d/(1.+MX::sumsqr(d));
    const auto vertices=inset_vertices(w,radius);
    MX point=vector2(vertices[0]);
    for(size_t k=1;k<vertices.size();++k)
        point+=vector2(vertices[k]-vertices[0])*d(k-1)*d(k-1)/(MX::sumsqr(d)+1e-24);
    return point;
}
inline MX knot_safety(const Window& w,const MX& point,double radius) {
    if(w.circle) {
        const MX rho=MX::sqrt(MX::sumsqr(point)+1e-14);
        return MX::pow(rho-w.opening_radius-.085,2)-radius*radius;
    }
    std::vector<MX> rows;
    for(size_t k=0;k<w.frame.size();++k) {
        const DM edge=vector2(w.frame[(k+1)%w.frame.size()]-w.frame[k]);
        const MX delta=point-vector2(w.frame[k]);
        const MX u=MX::fmin(1.,MX::fmax(0.,MX::dot(delta,edge)/MX::dot(edge,edge)));
        rows.push_back(MX::sumsqr(delta-u*edge)-radius*radius);
    }
    return MX::vertcat(rows);
}

// Remove the legacy workspace and segment-duration rows from native inputs.
inline void remove_domain_bounds(Json& metadata) {
    if(metadata.value("domain_bounds_removed",false))return;
    const int workspace=metadata.at("workspace_rows"),m=metadata.at("m"),n=metadata.at("n");
    for(const auto& key:{"lower_g","upper_g"}) {
        auto& bounds=metadata.at(key);
        bounds.erase(bounds.begin()+workspace+m,bounds.begin()+workspace+m+n);
        bounds.erase(bounds.begin(),bounds.begin()+workspace);
    }
    metadata["workspace_rows"]=0;
    metadata["domain_bounds_removed"]=true;
    metadata["config"]["workspace_lower"]=nullptr;
    metadata["config"]["workspace_upper"]=nullptr;
}

inline void use_direct_point_coordinates(Json& metadata) {
    if(metadata.value("point_coordinates",std::string())=="direct_polar_polygon")return;
    metadata["point_coordinates"]="direct_polar_polygon";
    metadata["point_domain_rows"]=0;
    if(!metadata.at("free_points").get<bool>())return;
    const int n=metadata.at("n"),m=metadata.at("m"),old_vs=metadata.at("velocity_start");
    const int new_vs=n+2*m,new_nv=new_vs+9*m;
    const auto old_slices=metadata.at("slices");
    const auto old_initials=metadata.at("initials");
    int old_gauges=0,domain_rows=0;
    Json slices=Json::array();
    for(int gate=0;gate<m;++gate) {
        const auto w=load_window(metadata.at("windows").at(gate));
        if(!w.circle){++old_gauges;domain_rows+=w.frame.size();}
        slices.push_back(Json{{"begin",n+2*gate},{"end",n+2*gate+2},{"circle",w.circle}});
    }
    for(const auto& key:{"lower_g","upper_g"}) {
        const auto old=metadata.at(key);Json bounds=Json::array();
        for(int j=0;j<m;++j)bounds.push_back(old.at(j));
        for(int j=0;j<domain_rows;++j)bounds.push_back(std::string(key)=="lower_g" ? Json(0.) : Json(nullptr));
        for(size_t j=m+old_gauges;j<old.size();++j)bounds.push_back(old.at(j));
        metadata[key]=bounds;
    }
    Json initials=Json::array();
    for(const auto& old_seed:old_initials) {
        const auto old=old_seed.at("original").get<std::vector<double>>();
        std::vector<double> physical(new_nv,0.);
        std::copy(old.begin(),old.begin()+n,physical.begin());
        std::copy(old.begin()+old_vs,old.end(),physical.begin()+new_vs);
        for(int gate=0;gate<m;++gate) {
            const auto w=load_window(metadata.at("windows").at(gate));
            const int begin=old_slices.at(gate).at("begin"),end=old_slices.at(gate).at("end");
            Eigen::Vector2d q=Eigen::Vector2d::Zero();
            if(w.circle) {
                const Eigen::Vector2d d(old[begin],old[begin+1]);
                q=2.*(w.opening_radius+.085-metadata.at("finite_radius").get<double>())*d/(1.+d.squaredNorm());
                physical[n+2*gate]=q.norm();
                physical[n+2*gate+1]=q.norm()>0. ? std::atan2(q.y(),q.x()) : 0.;
                if(physical[n+2*gate+1]<0.)physical[n+2*gate+1]+=2.*pi;
            } else {
                const auto vertices=inset_vertices(w,metadata.at("finite_radius"));
                double squared=1e-24;for(int j=begin;j<end;++j)squared+=old[j]*old[j];
                q=vertices[0];
                for(size_t j=1;j<vertices.size();++j)q+=(vertices[j]-vertices[0])*old[begin+j-1]*old[begin+j-1]/squared;
                physical[n+2*gate]=q.x();physical[n+2*gate+1]=q.y();
            }
        }
        std::vector<double> finite(new_nv+metadata.at("slack_scale").size(),0.);
        std::copy(physical.begin(),physical.end(),finite.begin());
        Json seed=old_seed;seed["original"]=physical;seed["finite"]=finite;initials.push_back(seed);
    }
    metadata["initials"]=initials;metadata["slices"]=slices;
    metadata["velocity_start"]=new_vs;metadata["nvar"]=new_nv;metadata["point_domain_rows"]=domain_rows;
}

inline void use_derivative_order(Json& metadata,int derivative_order) {
    const int current=metadata.value("minco_derivative_order",3);
    if(derivative_order<1 || derivative_order>current)
        throw std::runtime_error("Unsupported prescribed knot derivative order");
    metadata["minco_derivative_order"]=derivative_order;
    if(derivative_order==current)return;
    const int vs=metadata.at("velocity_start"),m=metadata.at("m");
    const int nv=vs+3*m*derivative_order;
    for(auto& seed:metadata["initials"]) {
        auto physical=seed.at("original").get<std::vector<double>>();physical.resize(nv);
        std::vector<double> finite(nv+metadata.at("slack_scale").size(),0.);
        std::copy(physical.begin(),physical.end(),finite.begin());
        seed["original"]=physical;seed["finite"]=finite;
    }
    auto scaling=metadata.at("state_scale").get<std::vector<double>>();scaling.resize(3*m*derivative_order);
    metadata["state_scale"]=scaling;metadata["nvar"]=nv;
}

class Graph {
    std::unique_ptr<Coefficients> coefficients_;
public:
    std::vector<MX> expressions;
    Function trajectory,residuals,point_objective_gradient;
    Graph(const Json& metadata,const Json& scene,const std::vector<Window>& windows,
          const Function& dynamics=Function(),double dynamics_weight=1.) {
        const int n=metadata["n"],m=metadata["m"],nv=metadata["nvar"],vs=metadata["velocity_start"];
        if(!std::isfinite(dynamics_weight) || dynamics_weight<0.)
            throw std::runtime_error("Dynamics weight must be finite and nonnegative");
        const bool soft=!dynamics.is_null();
        const bool direct=metadata.value("point_coordinates",std::string())=="direct_polar_polygon";
        const int slack_count=soft ? n*33*6 : 0;
        const MX z=MX::sym("native_actual_pvaj_variables",nv+slack_count);
        const MX parameters=MX::sym("normalization_parameters",1+2*n);
        const MX k=z(casadi::Slice(0,n));
        const MX times=MX::if_else(k>0.,.5*k*k+k+1.,1./(.5*k*k-k+1.));
        const MX arrivals=MX::cumsum(times);
        const MX states=DM(metadata["state_scale"].get<std::vector<double>>())*z(casadi::Slice(vs,nv));
        std::vector<MX> points,local_points,gauges,point_domain;
        for(int gate=0;gate<m;++gate) {
            const auto& w=windows[gate];
            MX q=MX::zeros(2,1);
            if(metadata["free_points"].get<bool>()) {
                const auto& slice=metadata["slices"][gate];
                const MX d=z(casadi::Slice(slice["begin"].get<int>(),slice["end"].get<int>()));
                if(direct) {
                    q=w.circle ? MX::vertcat({d(0)*MX::cos(d(1)),d(0)*MX::sin(d(1))}) : d;
                    if(!w.circle)for(size_t edge=0;edge<w.frame.size();++edge) {
                        const Eigen::Vector2d delta=w.frame[(edge+1)%w.frame.size()]-w.frame[edge];
                        const Eigen::Vector2d normal=Eigen::Vector2d(delta.y(),-delta.x()).normalized();
                        const double limit=normal.dot(w.frame[edge])-metadata["finite_radius"].get<double>();
                        point_domain.push_back(limit-MX::dot(vector2(normal),q));
                    }
                } else {
                    q=point_map(w,d,metadata["finite_radius"]);
                    if(!w.circle)gauges.push_back(MX::sumsqr(d)-1.);
                }
            }
            local_points.push_back(q);
            const MX phase=2.*pi*w.frequency*arrivals(gate)+w.phase;
            MX motion;
            if(w.swing) {
                const MX theta=std::asin(w.amplitude/w.length)*MX::sin(phase);
                motion=MX::vertcat({w.length*(1.-MX::cos(theta)),-w.length*MX::sin(theta),MX(0.)});
            } else motion=MX::vertcat({MX(0.),-w.amplitude*MX::sin(phase),MX(0.)});
            points.push_back(vector3(w.center)+MX::mtimes(rotation(w),MX::vertcat({q,MX(0.)})+motion));
        }
        const MX packed_points=MX::vertcat(points);
        drolib::PVAJ head=drolib::PVAJ::Zero(),tail=drolib::PVAJ::Zero();
        for(int axis=0;axis<3;++axis){head(axis,0)=scene["start"][axis];tail(axis,0)=scene["goal"][axis];}
        coefficients_=std::make_unique<Coefficients>(head,tail,n,metadata.value("minco_derivative_order",3));
        const MX coefficients=(*coefficients_)(std::vector<MX>{MX::vertcat({times,packed_points,states})})[0];
        std::vector<MX> rows;
        const bool historical_domain=metadata.value("restore_historical_domain",false);
        if(historical_domain) {
            const auto lower=metadata["config"]["workspace_lower"].get<std::vector<double>>();
            const int nodes=metadata["config"]["nodes_per_piece"];
            for(int piece=0;piece<n;++piece)for(int node=0;node<=nodes;++node) {
                const double u=double(node)/nodes;
                for(int axis=0;axis<3;++axis) {
                    MX position=coefficients(24*piece+8*axis+7);
                    for(int power=6;power>=0;--power)position=position*u+coefficients(24*piece+8*axis+power);
                    rows.push_back((position-lower[axis])/30.);
                }
            }
        }
        for(int gate=0;gate<m;++gate) {
            const Vector normal=-scene["windows"][gate]["sign"].get<double>()*windows[gate].rotation.col(2);
            rows.push_back(MX::dot(vector3(normal),states(casadi::Slice(3*gate,3*(gate+1))))-.1);
        }
        if(historical_domain)rows.push_back(times);
        if(!gauges.empty())rows.push_back(MX::vertcat(gauges));
        if(!point_domain.empty())rows.push_back(MX::vertcat(point_domain));
        const MX physical=MX::vertcat({MX::log(times),z(casadi::Slice(n,vs)),states});
        const MX total=MX::sum1(times);
        MX objective=total,actual_penalty=MX(0.);
        if(soft) {
            const auto scaling=metadata["slack_scale"].get<std::vector<double>>();
            if(static_cast<int>(scaling.size())!=slack_count)
                throw std::runtime_error("Soft dynamics requires 33 nodes and six residuals per piece");
            const MX scaled_slack=DM(scaling)*z(casadi::Slice(nv,nv+slack_count));
            std::vector<MX> dynamic_rows;
            auto penalty=[](const MX& value) {
                constexpr double mu=.01;
                const MX transition=(mu-.5*value)*MX::pow(value/mu,3);
                return MX::if_else(value<0.,MX(0.),
                    MX::if_else(value>mu,value-.5*mu,transition));
            };
            const auto residual_scale=metadata["dynamic_residual_scale"].get<std::vector<double>>();
            for(int piece=0;piece<n;++piece) {
                const MX state=MX::vertcat({coefficients(casadi::Slice(24*piece,24*(piece+1))),times(piece)});
                for(int node=0;node<=32;++node) {
                    const MX values=dynamics(std::vector<MX>{state,MX(node/32.)})[0];
                    dynamic_rows.push_back(values);
                    const double weight=(node==0 || node==32) ? .5 : 1.;
                    for(int component=0;component<6;++component) {
                        const int index=(piece*33+node)*6+component;
                        objective+=dynamics_weight*times(piece)/32.*weight*penalty(scaled_slack(index)*residual_scale[component]);
                        actual_penalty+=times(piece)/32.*weight*penalty(MX::fmax(values(component),0.)*residual_scale[component]);
                    }
                }
            }
            const MX dynamic_values=MX::vertcat(dynamic_rows);
            rows.push_back(scaled_slack-dynamic_values);
            residuals=Function("native_soft_residuals",{z,parameters},{dynamic_values});
        }
        expressions={z,parameters,physical,arrivals,coefficients,objective/parameters(0),MX::vertcat(rows)};
        for(int gate=0;gate<m;++gate)expressions.push_back(knot_safety(windows[gate],local_points[gate],metadata["finite_radius"]));
        if(direct && metadata["free_points"].get<bool>())
            point_objective_gradient=Function("direct_point_initial_gradient",{z,parameters},
                {MX::gradient(total+dynamics_weight*actual_penalty,z)});
        trajectory=Function("native_minco_pvaj_trajectory",{z,parameters},
            {physical,times,coefficients,packed_points,total+dynamics_weight*actual_penalty,actual_penalty});
    }
};
} // namespace native_minco
