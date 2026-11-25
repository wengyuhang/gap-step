#pragma once
#include <casadi/casadi.hpp>
#include "geometry.hpp"
#include "planar_points.hpp"
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
        if(derivative_order_==3) {
            using casadi::SX;
            const SX x=SX::sym("physical_pvaj_input",n_+12*m_);
            const SX nominal=SX::sym("nominal_coefficients",24*n_);
            Eigen::Matrix<double,8,8> inverse;
            for(int row=0;row<8;++row)for(int col=0;col<8;++col)
                inverse(row,col)=static_cast<double>(togt_pv::pvaj_unit_hermite_inverse[row][col]);
            std::vector<SX> outputs;
            for(int piece=0;piece<n_;++piece)for(int axis=0;axis<3;++axis) {
                std::vector<SX> boundary;
                for(int endpoint=0;endpoint<2;++endpoint) {
                    const int knot=piece+endpoint-1;
                    for(int order=0;order<4;++order) {
                        SX value;
                        if(knot<0)value=SX(head_(axis,order));
                        else if(knot>=m_)value=SX(tail_(axis,order));
                        else if(order==0)value=x(n_+3*knot+axis);
                        else value=x(n_+3*m_+3*((order-1)*m_+knot)+axis);
                        boundary.push_back(value*SX::pow(x(piece),order));
                    }
                }
                for(int power=0;power<8;++power) {
                    SX value=0.;for(int k=0;k<8;++k)value+=inverse(power,k)*boundary[k];
                    outputs.push_back(value);
                }
            }
            const SX coefficients=SX::vertcat(outputs);
            return Function(name,{x,nominal},{SX::jacobian(coefficients,x)},{{"cse",true}});
        }
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
                result[24*piece+8*axis+power]=minco_.normalizedCoefficients()(8*piece+power,axis);
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
    metadata["restore_historical_domain"]=false;
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
    Function trajectory,residuals,point_objective_gradient,gauss_newton_hessian,canonical_lift,interpolation_inputs,control_features;
    Graph(const Json& metadata,const Json& scene,const std::vector<Window>& windows,
          const Function& dynamics=Function(),double dynamics_weight=1.) {
        const int n=metadata["n"],m=metadata["m"],nv=metadata["nvar"],vs=metadata["velocity_start"];
        if(!std::isfinite(dynamics_weight) || dynamics_weight<0.)
            throw std::runtime_error("Dynamics weight must be finite and nonnegative");
        const bool soft=!dynamics.is_null();
        const bool direct=metadata.value("point_coordinates",std::string())=="direct_polar_polygon";
        const bool xy=metadata.value("point_coordinates",std::string())=="scaled_xy";
        if(xy)planar_points::validate(metadata);
        constexpr int intervals=16;
        const bool direct_soft=soft && metadata.value("native_soft_formulation",std::string())=="direct";
        const bool hermite_dynamics=metadata.value("hermite_dynamics_evaluation",false);
        if(hermite_dynamics && (!soft || metadata.value("minco_derivative_order",3)!=3 || dynamics.size1_in(0)!=9))
            throw std::runtime_error("Hermite dynamics evaluation requires full PVAJ and derivative-input flatness model");
        const bool lift_coefficients=metadata.value("lift_minco_coefficients",false);
        const bool bernstein_lift=metadata.value("bernstein_coefficient_coordinates",false);
        if(bernstein_lift && (!lift_coefficients || !hermite_dynamics))
            throw std::runtime_error("Bernstein lifting requires coefficient lifting and Hermite dynamics");
        const bool lift_points=metadata.value("lift_world_points",false);
        const bool lift_arrivals=metadata.value("lift_arrival_times",false);
        if(lift_arrivals && !lift_points && !lift_coefficients)throw std::runtime_error("Arrival lifting requires world-point or coefficient lifting");
        if(lift_points && (lift_coefficients || !direct_soft || metadata.value("minco_derivative_order",3)!=3))
            throw std::runtime_error("World-point lifting requires direct full PVAJ without coefficient lifting");
        if(lift_coefficients && (!soft || metadata.value("minco_derivative_order",3)!=3))
            throw std::runtime_error("Coefficient lifting requires full-PVAJ soft model");
        const int slack_count=soft && !direct_soft ? n*(intervals+1)*6 : 0;
        const int coefficient_start=nv+slack_count;
        const int arrival_start=coefficient_start+(lift_coefficients?24*n:lift_points?3*m:0);
        const MX z=MX::sym("native_actual_pvaj_variables",nv+slack_count+(lift_coefficients?24*n:lift_points?3*m:0)+(lift_arrivals?n:0));
        const MX parameters=MX::sym("normalization_parameters",1+2*n);
        const MX k=z(casadi::Slice(0,n));
        const MX times=MX::if_else(k>0.,.5*k*k+k+1.,1./(.5*k*k-k+1.));
        const MX physical_arrivals=MX::cumsum(times);
        const MX arrivals=lift_arrivals?z(casadi::Slice(arrival_start,arrival_start+n)):physical_arrivals;
        MX state_units=DM(metadata["state_scale"].get<std::vector<double>>());
        // An invertible change of units, not a MINCO baseline plus deviations.
        // At the raw initial durations the old and new physical states coincide.
        if(metadata.value("duration_normalized_states",false)) {
            const MX initial_arrivals=parameters(casadi::Slice(1,1+n));
            const MX initial_times=MX::vertcat({initial_arrivals(0),initial_arrivals(casadi::Slice(1,n))-initial_arrivals(casadi::Slice(0,n-1))});
            std::vector<MX> factors;
            for(int order=1;order<=metadata.value("minco_derivative_order",3);++order)
                for(int gate=0;gate<m;++gate) {
                    const MX ratio=MX::sqrt(initial_times(gate)*initial_times(gate+1)/(times(gate)*times(gate+1)));
                    for(int axis=0;axis<3;++axis)factors.push_back(MX::pow(ratio,order));
                }
            state_units*=MX::vertcat(factors);
        }
        const MX states=state_units*z(casadi::Slice(vs,nv));
        std::vector<MX> points,local_points,gauges,point_domain;
        for(int gate=0;gate<m;++gate) {
            const auto& w=windows[gate];
            MX q = MX::zeros(w.waypoint ? 3 : 2, 1);
            if(metadata["free_points"].get<bool>()) {
                const auto& slice=metadata["slices"][gate];
                const MX d=z(casadi::Slice(slice["begin"].get<int>(),slice["end"].get<int>()));
                if(xy) {
                    const auto box=planar_points::domain(w,planar_points::erosion_radius(metadata));
                    q = DM(std::vector<double>(box.scale.data(),
                                               box.scale.data() + box.scale.size())) *
                        d;
                    if (w.waypoint)
                        point_domain.push_back(1. - MX::sumsqr(d));
                    else if (w.circle)
                        point_domain.push_back((box.safe_radius * box.safe_radius - MX::sumsqr(q)) /
                                               (w.opening_radius * w.opening_radius));
                    else for(size_t edge=0;edge<box.normals.size();++edge)
                        point_domain.push_back((box.limits[edge]-MX::dot(vector2(box.normals[edge]),q))/box.scale.maxCoeff());
                } else if(direct) {
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
            if (w.motion_angle != 0.) {
                const double c = std::cos(w.motion_angle), s = std::sin(w.motion_angle);
                motion = MX::vertcat({c * motion(0) - s * motion(1),
                                        s * motion(0) + c * motion(1), motion(2)});
            }
            points.push_back(
                vector3(w.center) +
                MX::mtimes(rotation(w), (w.waypoint ? q : MX::vertcat({q, MX(0.)})) + motion));
        }
        const MX packed_points=MX::vertcat(points);
        if(metadata.value("native_hessian",std::string())=="minco_tangent") {
            const MX geometry=MX::vertcat({times,packed_points});
            interpolation_inputs=Function("interpolation_geometry_jacobian",{z,parameters},{geometry,MX::jacobian(geometry,z)});
        }
        drolib::PVAJ head=drolib::PVAJ::Zero(),tail=drolib::PVAJ::Zero();
        for(int axis=0;axis<3;++axis){head(axis,0)=scene["start"][axis];tail(axis,0)=scene["goal"][axis];}
        coefficients_=std::make_unique<Coefficients>(head,tail,n,metadata.value("minco_derivative_order",3));
        const MX evaluated_points=lift_points?z(casadi::Slice(nv,nv+3*m)):packed_points;
        const MX fitted_coefficients=(*coefficients_)(std::vector<MX>{MX::vertcat({times,evaluated_points,states})})[0];
        MX coefficients=lift_coefficients?z(casadi::Slice(coefficient_start,coefficient_start+24*n)):fitted_coefficients;
        MX canonical_coefficients=fitted_coefficients;
        if(bernstein_lift) {
            // Same unique degree-seven PVAJ interpolant, with explicit relative
            // Bernstein controls. Eliminate these equality-linked auxiliaries
            // before accepting a trajectory. No extra cost or physical bound.
            std::vector<MX> canonical_controls,power_coefficients;
            auto choose=[](int a,int b){double c=1.;for(int k=1;k<=b;++k)c=c*(a-k+1)/k;return c;};
            for(int piece=0;piece<n;++piece) {
                const MX pl=piece?evaluated_points(casadi::Slice(3*(piece-1),3*piece)):vector3(head.col(0));
                const MX pr=piece<m?evaluated_points(casadi::Slice(3*piece,3*(piece+1))):vector3(tail.col(0));
                const MX t=times(piece),delta=pr-pl;
                std::vector<MX> l,r;
                for(int order=1;order<=3;++order) {
                    l.push_back(piece?MX::pow(t,order)*states(casadi::Slice(3*((order-1)*m+piece-1),3*((order-1)*m+piece))):MX::zeros(3,1));
                    r.push_back(piece<m?MX::pow(t,order)*states(casadi::Slice(3*((order-1)*m+piece),3*((order-1)*m+piece+1))):MX::zeros(3,1));
                }
                const std::vector<MX> b{MX::zeros(3,1),l[0]/7.,2.*l[0]/7.+l[1]/42.,3.*l[0]/7.+l[1]/14.+l[2]/210.,
                    delta-3.*r[0]/7.+r[1]/14.-r[2]/210.,delta-2.*r[0]/7.+r[1]/42.,delta-r[0]/7.,delta};
                for(int axis=0;axis<3;++axis)for(int power=0;power<8;++power) {
                    canonical_controls.push_back(b[power](axis));
                    MX coefficient=power==0?pl(axis):MX(0.);
                    for(int k=0;k<=power;++k)coefficient+=choose(7,power)*choose(power,k)*((power-k)%2?-1.:1.)*z(coefficient_start+24*piece+8*axis+k);
                    power_coefficients.push_back(coefficient);
                }
            }
            canonical_coefficients=MX::vertcat(canonical_controls);
            coefficients=MX::vertcat(power_coefficients);
        }
        if(lift_coefficients) {
            // Restore cumulative times before reconstructing the unique PVAJ
            // polynomial. Merely appending times would leave controls fitted to
            // stale auxiliary arrival times after a finite solve.
            if(lift_arrivals) {
                const Function placement("canonical_controls_at_arrivals",{z,parameters},{canonical_coefficients});
                canonical_coefficients=placement(std::vector<MX>{MX::vertcat({z(casadi::Slice(0,arrival_start)),physical_arrivals}),parameters})[0];
            }
            std::vector<MX> canonical{z(casadi::Slice(0,coefficient_start)),canonical_coefficients};
            if(lift_arrivals)canonical.push_back(physical_arrivals);
            canonical_lift=Function("canonical_minco_lift",{z,parameters},{MX::vertcat(canonical)});
        }
        if(lift_points) {
            MX canonical_points=packed_points;
            if(lift_arrivals) {
                const Function placement("lifted_gate_placement",{z,parameters},{packed_points});
                canonical_points=placement(std::vector<MX>{MX::vertcat({z(casadi::Slice(0,nv+3*m)),physical_arrivals}),parameters})[0];
            }
            std::vector<MX> canonical{z(casadi::Slice(0,nv)),canonical_points};
            if(lift_arrivals)canonical.push_back(physical_arrivals);
            canonical_lift=Function("canonical_world_points",{z,parameters},{MX::vertcat(canonical)});
        }
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
            if (windows[gate].waypoint) {
                rows.push_back(MX(1.));
                continue;
            }
            const Vector normal=-scene["windows"][gate]["sign"].get<double>()*windows[gate].rotation.col(2);
            rows.push_back(MX::dot(vector3(normal),states(casadi::Slice(3*gate,3*(gate+1))))-.1);
        }
        if(historical_domain)rows.push_back(times);
        if(!gauges.empty())rows.push_back(MX::vertcat(gauges));
        if(!point_domain.empty())rows.push_back(MX::vertcat(point_domain));
        std::vector<MX> physical_points;
        for(const auto& point:local_points)if(xy && metadata["free_points"].get<bool>())physical_points.push_back(point);
        const MX physical=MX::vertcat({MX::log(times),
            xy && !physical_points.empty()?MX::vertcat(physical_points):z(casadi::Slice(n,vs)),states});
        const MX total=MX::sum1(times);
        MX objective=total,actual_penalty=MX(0.);
        if(soft) {
            const auto scaling=metadata["slack_scale"].get<std::vector<double>>();
            if(!direct_soft && static_cast<int>(scaling.size())!=slack_count)
                throw std::runtime_error("Soft dynamics requires 17 nodes and six residuals per piece");
            const MX scaled_slack=direct_soft?MX::zeros(0,1):DM(scaling)*z(casadi::Slice(nv,nv+slack_count));
            std::vector<MX> dynamic_rows,cost_rows;
            const bool cost_epigraph=metadata.value("native_soft_formulation",std::string())=="cost_epigraph";
            const bool generalized_gn=metadata.value("native_hessian",std::string())=="generalized_gauss_newton" || metadata.value("native_hessian",std::string())=="minco_tangent";
            const bool feature_metric=metadata.value("feature_coordinate_equilibration",false);
            std::vector<MX> features,feature_weights,residual_weights;
            auto penalty=[](const MX& value) {
                constexpr double mu=.01;
                const MX transition=(mu-.5*value)*MX::pow(value/mu,3);
                return MX::if_else(value<0.,MX(0.),
                    MX::if_else(value>mu,value-.5*mu,transition));
            };
            const auto residual_scale=metadata["dynamic_residual_scale"].get<std::vector<double>>();
            for(int piece=0;piece<n;++piece) {
                const MX state=MX::vertcat({coefficients(casadi::Slice(24*piece,24*(piece+1))),times(piece)});
                std::vector<MX> left,right;
                std::vector<std::vector<MX>> derivative_controls;
                if(hermite_dynamics) {
                    for(int order=1;order<=3;++order) {
                        left.push_back(piece?states(casadi::Slice(3*((order-1)*m+piece-1),3*((order-1)*m+piece))):MX::zeros(3,1));
                        right.push_back(piece<m?states(casadi::Slice(3*((order-1)*m+piece),3*((order-1)*m+piece+1))):MX::zeros(3,1));
                    }
                    const MX pleft=piece?evaluated_points(casadi::Slice(3*(piece-1),3*piece)):vector3(head.col(0));
                    const MX pright=piece<m?evaluated_points(casadi::Slice(3*piece,3*(piece+1))):vector3(tail.col(0));
                    const MX delta=pright-pleft,t=times(piece),vl=t*left[0],vr=t*right[0],al=t*t*left[1],ar=t*t*right[1],jl=t*t*t*left[2],jr=t*t*t*right[2];
                    std::vector<MX> controls{MX::zeros(3,1),vl/7.,2.*vl/7.+al/42.,3.*vl/7.+al/14.+jl/210.,
                        delta-3.*vr/7.+ar/14.-jr/210.,delta-2.*vr/7.+ar/42.,delta-vr/7.,delta};
                    if(bernstein_lift)for(int k=0;k<8;++k)controls[k]=MX::vertcat({z(coefficient_start+24*piece+k),z(coefficient_start+24*piece+8+k),z(coefficient_start+24*piece+16+k)});
                    for(int order=1;order<=4;++order) {
                        std::vector<MX> next;for(int k=0;k+1<controls.size();++k)next.push_back((8.-order)*(controls[k+1]-controls[k]));
                        controls=std::move(next);if(order>=2)derivative_controls.push_back(controls);
                    }
                }
                for(int node=0;node<=intervals;++node) {
                    MX evaluated_state=state;
                    if(hermite_dynamics) {
                        std::vector<MX> ajs;const double u=double(node)/intervals;
                        for(int order=2;order<=4;++order) {
                            auto controls=derivative_controls[order-2];
                            while(controls.size()>1){std::vector<MX> next;for(int k=0;k+1<controls.size();++k)next.push_back((1.-u)*controls[k]+u*controls[k+1]);controls=std::move(next);}
                            ajs.push_back(!bernstein_lift && order<4 && node==0?left[order-1]:!bernstein_lift && order<4 && node==intervals?right[order-1]:controls[0]/MX::pow(times(piece),order));
                        }
                        evaluated_state=MX::vertcat(ajs);
                    }
                    const auto evaluated=dynamics(std::vector<MX>{evaluated_state,MX(double(node)/intervals)});
                    const MX values=evaluated[0];
                    if(generalized_gn || feature_metric)features.push_back(evaluated.at(1));
                    dynamic_rows.push_back(values);
                    const double weight=(node==0 || node==intervals) ? .5 : 1.;
                    for(int component=0;component<6;++component) {
                        const int index=(piece*(intervals+1)+node)*6+component;
                        if(!direct_soft)objective+=dynamics_weight*times(piece)/intervals*weight*(cost_epigraph?scaled_slack(index):penalty(scaled_slack(index)*residual_scale[component]));
                        if(cost_epigraph)cost_rows.push_back(penalty(values(component)*residual_scale[component]));
                        actual_penalty+=times(piece)/intervals*weight*penalty(values(component)*residual_scale[component]);
                        if(generalized_gn) {
                            const MX v=values(component)*residual_scale[component];
                            constexpr double mu=.01;
                            const MX first=MX::if_else(v<=0.,MX(0.),MX::if_else(v>=mu,MX(1.),3.*v*v/(mu*mu)-2.*v*v*v/(mu*mu*mu)));
                            const MX second=MX::if_else(v<=0.,MX(0.),MX::if_else(v>=mu,MX(0.),6.*v*(mu-v)/(mu*mu*mu)));
                            const MX factor=dynamics_weight*times(piece)/intervals*weight/parameters(0);
                            residual_weights.push_back(factor*second*residual_scale[component]*residual_scale[component]);
                            feature_weights.push_back(factor*2.*first);
                            if(component==0)feature_weights.push_back(factor*2.*first);
                        }
                    }
                }
            }
            const MX dynamic_values=MX::vertcat(dynamic_rows);
            if(direct_soft)objective+=dynamics_weight*actual_penalty;
            else rows.push_back(scaled_slack-(cost_epigraph?MX::vertcat(cost_rows):dynamic_values));
            residuals=Function("native_soft_residuals",{z,parameters},{dynamic_values});
            if(feature_metric) {
                std::vector<double> units;
                for(int sample=0;sample<n*(intervals+1);++sample)for(int component=0;component<6;++component){units.push_back(std::sqrt(residual_scale[component]));if(component==0)units.push_back(std::sqrt(residual_scale[component]));}
                control_features=Function("native_normalized_control_features",{z,parameters},{MX::vertcat(features)/DM(units)});
            }
            if(generalized_gn) {
                if(!direct_soft)throw std::runtime_error("Generalized Gauss-Newton requires direct soft objective");
                const MX jr=MX::jacobian(dynamic_values,z),jf=MX::jacobian(MX::vertcat(features),z);
                const MX h=MX::hessian(total/parameters(0),z)+
                    MX::mtimes(jr.T(),MX::mtimes(MX::diag(MX::vertcat(residual_weights)),jr))+
                    MX::mtimes(jf.T(),MX::mtimes(MX::diag(MX::vertcat(feature_weights)),jf))+
                    metadata.value("native_gn_residual_metric",0.)*MX::mtimes(jr.T(),jr);
                gauss_newton_hessian=Function("native_generalized_gauss_newton",{z,parameters},{h});
            }
        }
        if(lift_coefficients)rows.push_back(z(casadi::Slice(coefficient_start,coefficient_start+24*n))-canonical_coefficients);
        if(lift_points)rows.push_back(evaluated_points-packed_points);
        if(lift_arrivals)rows.push_back(arrivals-MX::vertcat({MX(0.),arrivals(casadi::Slice(0,n-1))})-times);
        expressions={z,parameters,physical,arrivals,coefficients,objective/parameters(0),MX::vertcat(rows)};
        for (int gate = 0; gate < m; ++gate)
            expressions.push_back(
                windows[gate].waypoint
                    ? MX(1.)
                    : knot_safety(windows[gate], local_points[gate], metadata["finite_radius"]));
        if(direct && metadata["free_points"].get<bool>())
            point_objective_gradient=Function("direct_point_initial_gradient",{z,parameters},
                {MX::gradient(total+dynamics_weight*actual_penalty,z)});
        trajectory=Function("native_minco_pvaj_trajectory",{z,parameters},
            {physical,times,coefficients,packed_points,total+dynamics_weight*actual_penalty,actual_penalty});
    }
};
} // namespace native_minco
