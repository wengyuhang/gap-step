#pragma once
#include "trajectory.hpp"
#include "drolib/planner/traj_solver.hpp"
#include "drolib/shape/ball.h"
#include "drolib/shape/polyhedron.h"
#include <chrono>
#include <filesystem>
#include <memory>

namespace dynamic_togt {
using Clock=std::chrono::steady_clock;

struct SafePoint {
    std::shared_ptr<drolib::ShapeBase> shape;
    bool circle;
    int dimension;
    explicit SafePoint(const Window& window,double inset):circle(window.circle) {
        if(circle) {
            auto disk=std::make_shared<drolib::Ball>();
            disk->position.setZero();disk->radius=window.opening_radius+.085-inset;disk->margin=0.;
            if(!(disk->radius>0.))throw std::runtime_error("Empty circular inset");
            disk->initialize();shape=disk;dimension=2;
        } else {
            std::vector<Eigen::Vector3d> vertices;
            const auto inset_vertices=inset_frame_vertices(window,inset);
            for(const auto& point:inset_vertices) {
                if(!point.allFinite())throw std::runtime_error("Invalid polygon inset");
                for(size_t edge=0;edge<window.frame.size();++edge) {
                    const auto delta=(window.frame[(edge+1)%window.frame.size()]-window.frame[edge]).eval();
                    const Eigen::Vector2d normal(delta.y(),-delta.x());
                    if(normal.dot(point-window.frame[edge])> -inset*normal.norm()+1e-9)
                        throw std::runtime_error("Empty polygon inset");
                }
                vertices.emplace_back(point.x(),point.y(),0.);
            }
            shape=std::make_shared<drolib::Polyhedron>(vertices);dimension=shape->dimension();
        }
    }
    Eigen::VectorXd upstream_coordinates(const Eigen::VectorXd& d) const {
        if(!circle)return d;
        Eigen::Vector3d value=Eigen::Vector3d::Zero();value.head<2>()=d;return value;
    }
    Eigen::Vector3d point(const Eigen::VectorXd& d) const {
        return shape->toP(upstream_coordinates(d));
    }
    Eigen::VectorXd initial() const {
        return shape->toD(shape->position).head(dimension);
    }
    Eigen::VectorXd gradient(const Eigen::VectorXd& d,const Eigen::Vector3d& local_gradient) const {
        Eigen::Matrix3Xd gp(3,1);gp.col(0)=local_gradient;
        Eigen::VectorXd gd(shape->dimension());Eigen::Map<Eigen::VectorXd> mapped(gd.data(),gd.size());
        shape->getGradD(upstream_coordinates(d),gp,0,0,mapped);
        return gd.head(dimension);
    }
};

struct Evaluation {
    double objective=0.,time_cost=0.,penalty=0.,energy=0.;
    Eigen::VectorXd durations,gradient;
    Eigen::Matrix3Xd points;
    std::vector<Curve> coefficients;
};

class Planner {
    bool center_;
    int n_,m_,dimension_;
    double inset_;
    std::vector<Window> windows_;
    std::vector<SafePoint> maps_;
    std::vector<int> offsets_;
    drolib::PVAJ head_=drolib::PVAJ::Zero(),tail_=drolib::PVAJ::Zero();
    drolib::TrajParams params_;
    drolib::LbfgsParams lbfgs_;
    drolib::QuadManifold quad_;
    drolib::ConstAngle yaw_{0.};
    drolib::MincoSnap minco_;
    int evaluations_=0,iterations_=0;
    Json metadata_;
public:
    Planner(const Json& scene,bool center,const std::filesystem::path& parameter_directory,
            const Json& vehicle):center_(center),m_(scene.at("windows").size()),inset_(vehicle.at("finite_safety_radius_m")) {
        if(m_<=0)throw std::runtime_error("A nonempty gate sequence is required");
        n_=m_+1;dimension_=n_;
        for(const auto& row:scene.at("windows")) {
            auto window=load_window(row);
            if(!window.rotation.allFinite() || !window.center.allFinite() ||
               (window.rotation.transpose()*window.rotation-Eigen::Matrix3d::Identity()).norm()>1e-8 ||
               window.frequency<0. || !std::isfinite(window.frequency) ||
               window.amplitude<0. || !std::isfinite(window.amplitude) ||
               (window.swing && !(window.length>window.amplitude)))
                throw std::runtime_error("Invalid fixed-plane window motion");
            windows_.push_back(window);maps_.emplace_back(window,inset_);
            offsets_.push_back(dimension_);if(!center_)dimension_+=maps_.back().dimension;
        }
        for(int axis=0;axis<3;++axis){head_(axis,0)=scene.at("start")[axis];tail_(axis,0)=scene.at("goal")[axis];}
        if(!params_.load(parameter_directory/"init/cpc_planning.yaml") ||
           !lbfgs_.load(parameter_directory/"init/cpc_lbfgs.yaml"))
            throw std::runtime_error("Cannot load upstream TOGT initialization parameters");
        if(params_.piecesPerSegment!=1)
            throw std::runtime_error("Gate-only TOGT requires upstream piecesPerSegment=1");
        const double original_speed_guess=params_.speedGuess;
        params_.speedGuess=vehicle.at("initial_speed_guess_mps");
        if(!(params_.speedGuess>0.))
            throw std::runtime_error("Positive initial speed is required");
        drolib::QuadParams quad_params;
        quad_params.name="x500";quad_params.mass=vehicle.at("mass");
        for(int axis=0;axis<3;++axis)quad_params.inertia[axis]=vehicle.at("inertia")[axis];
        quad_params.T_mb.resize(4,4);
        for(int i=0;i<4;++i)for(int j=0;j<4;++j)quad_params.T_mb(i,j)=vehicle.at("inverse_mixer")[i][j];
        quad_params.T_bm=quad_params.T_mb.inverse();quad_=drolib::QuadManifold(quad_params);
        params_.maxOmgXYSqr=vehicle.at("rate_squared_limits")[0];
        params_.maxOmgZSqr=vehicle.at("rate_squared_limits")[1];
        params_.maxOmgXY=std::sqrt(params_.maxOmgXYSqr);params_.maxOmgZ=std::sqrt(params_.maxOmgZSqr);
        params_.minThr=vehicle.at("rotor_min_n");params_.maxThr=vehicle.at("rotor_max_n");
        params_.thrMean=.5*(params_.maxThr+params_.minThr);params_.thrRadi=.5*(params_.maxThr-params_.minThr);
        params_.thrRadiSqr=params_.thrRadi*params_.thrRadi;
        params_.collectivtThrMean=4.*params_.thrMean;params_.collectivtThrRadi=4.*params_.thrRadi;
        params_.collectivtThrRadiSqr=params_.collectivtThrRadi*params_.collectivtThrRadi;
        minco_.setConditions(head_,tail_,n_);
        metadata_={{"weight_time",params_.weightTime},{"weight_energy",params_.weightEnergy},
            {"weight_position",params_.weightPos},{"weight_velocity",params_.weightVel},
            {"weight_bodyrate",params_.weightOmg},{"weight_rotation",params_.weightRot},
            {"weight_thrust",params_.weightThr},{"smoothing_eps",params_.smoothingEps},
            {"quadrature_intervals",params_.numConstPena},{"dynamic_quadrature",params_.dynamicConstCheck},
            {"original_static_speed_guess_mps",original_speed_guess},
            {"stopping_policy","original TOGT cpc/init/cpc_lbfgs.yaml"},
            {"speed_guess_mps",params_.speedGuess},{"lbfgs_memory",lbfgs_.params.mem_size},
            {"lbfgs_past",lbfgs_.params.past},{"lbfgs_delta",lbfgs_.params.delta},
            {"lbfgs_gradient_tolerance",lbfgs_.params.g_epsilon},{"lbfgs_max_iterations",lbfgs_.params.max_iterations},
            {"lbfgs_max_linesearch",lbfgs_.params.max_linesearch},{"lbfgs_min_step",lbfgs_.params.min_step},
            {"rotor_min_n",params_.minThr},{"rotor_max_n",params_.maxThr},
            {"max_omega_xy_radps",params_.maxOmgXY},{"max_omega_z_radps",params_.maxOmgZ},
            {"radius",vehicle.at("planning_proxy_radius_m")},{"finite_safety_radius_m",inset_},{"vehicle",vehicle},{"parameter_directory",parameter_directory.string()}};
    }
    int dimension() const {return dimension_;}
    int pieces() const {return n_;}
    const drolib::TrajParams& params() const {return params_;}
    const drolib::QuadManifold& quad() const {return quad_;}
    const drolib::PVAJ& head() const {return head_;}
    const drolib::PVAJ& tail() const {return tail_;}
    Eigen::VectorXd initial() const {
        Eigen::VectorXd x(dimension_),times(n_);x.setZero();
        Vector previous=head_.col(0);
        for(int gate=0;gate<m_;++gate) {
            const Vector local=center_ ? Vector::Zero().eval() : maps_[gate].shape->position;
            const Vector point=windows_[gate].center+windows_[gate].rotation*(windows_[gate].motion(0.)+local);
            times[gate]=(point-previous).norm()/params_.speedGuess;
            if(!center_)x.segment(offsets_[gate],maps_[gate].dimension)=maps_[gate].initial();
            previous=point;
        }
        times[m_]=(tail_.col(0)-previous).norm()/params_.speedGuess;
        if((times.array()<=0.).any())throw std::runtime_error("Coincident initial waypoints yield zero segment time");
        Eigen::Map<Eigen::VectorXd> time_coordinates(x.data(),n_);
        drolib::TrajSolver::backwardT(times,time_coordinates);
        return x;
    }
    Eigen::VectorXd initial_from_center(const Json& result) const {
        if(center_ || !result.value("found",false) || result.at("method")!="dynamic_togt_center" ||
           result.at("config")!=metadata_ || result.at("variables").size()!=static_cast<size_t>(n_))
            throw std::runtime_error("Expected a successful center result with matching TOGT configuration");
        Eigen::VectorXd seed(dimension_);
        for(int piece=0;piece<n_;++piece)seed[piece]=result.at("variables").at(piece);
        for(int gate=0;gate<m_;++gate)
            seed.segment(offsets_[gate],maps_[gate].dimension)=maps_[gate].shape->toD(Vector::Zero()).head(maps_[gate].dimension);
        if(!seed.allFinite())throw std::runtime_error("Nonfinite center initialization");
        return seed;
    }
    Evaluation evaluate(const Eigen::VectorXd& x) {
        if(x.size()!=dimension_)throw std::runtime_error("Invalid dynamic TOGT vector length");
        Evaluation out;out.durations.resize(n_);out.points.resize(3,m_);out.gradient=Eigen::VectorXd::Zero(dimension_);
        const Eigen::VectorXd k=x.head(n_);drolib::TrajSolver::forwardT(k,out.durations);
        double arrival=0.;
        for(int gate=0;gate<m_;++gate) {
            arrival+=out.durations[gate];
            const Vector local=center_ ? Vector::Zero().eval() : maps_[gate].point(x.segment(offsets_[gate],maps_[gate].dimension));
            out.points.col(gate)=windows_[gate].center+windows_[gate].rotation*(windows_[gate].motion(arrival)+local);
        }
        minco_.setParameters(out.points,out.durations);
        Eigen::MatrixX3d partial=Eigen::MatrixX3d::Zero(8*n_,3);
        Eigen::VectorXd direct_times=Eigen::VectorXd::Zero(n_);
        out.energy=drolib::TrajSolver::addEnergyCost(minco_,params_,partial,direct_times);
        out.penalty=drolib::TrajSolver::addPenaltyCost(out.durations,minco_.getCoeffs(),quad_,&yaw_,params_,partial,direct_times);
        Eigen::Matrix3Xd point_gradient(3,m_);Eigen::VectorXd time_gradient(n_);
        minco_.propagateGrad(partial,direct_times,point_gradient,time_gradient);
        out.time_cost=drolib::TrajSolver::addTimeCost(out.durations,params_,time_gradient);
        // Arrival i is the sum of all earlier segment durations. Its moving
        // waypoint contribution therefore propagates backward to every T_j<=i.
        std::vector<double> motion_gradient(m_);arrival=0.;
        for(int gate=0;gate<m_;++gate) {
            arrival+=out.durations[gate];
            motion_gradient[gate]=point_gradient.col(gate).dot(windows_[gate].rotation*windows_[gate].motion_velocity(arrival));
            if(!center_)out.gradient.segment(offsets_[gate],maps_[gate].dimension)=maps_[gate].gradient(
                x.segment(offsets_[gate],maps_[gate].dimension),windows_[gate].rotation.transpose()*point_gradient.col(gate));
        }
        double suffix=0.;for(int gate=m_-1;gate>=0;--gate){suffix+=motion_gradient[gate];time_gradient[gate]+=suffix;}
        Eigen::Map<Eigen::VectorXd> time_coordinates(out.gradient.data(),n_);
        drolib::TrajSolver::backPropagateT(k,time_gradient,time_coordinates);
        out.objective=out.time_cost+out.energy+out.penalty;
        for(int piece=0;piece<n_;++piece) {
            Curve curve;
            for(int power=0;power<8;++power)curve.row(power)=minco_.getCoeffs().row(8*piece+power)*std::pow(out.durations[piece],power);
            out.coefficients.push_back(curve);
        }
        return out;
    }
    static double cost(void* context,const Eigen::VectorXd& x,Eigen::VectorXd& gradient) {
        auto& planner=*static_cast<Planner*>(context);++planner.evaluations_;
        const auto value=planner.evaluate(x);gradient=value.gradient;return value.objective;
    }
    static int progress(void* context,const Eigen::VectorXd&,const Eigen::VectorXd&,
                        double,double,int iteration,int) {
        static_cast<Planner*>(context)->iterations_=iteration;
        return 0;
    }
    Json solve(const Json& center_result=Json(nullptr),const Eigen::VectorXd& supplied_time_coordinates=Eigen::VectorXd()) {
        if(!center_ && center_result.is_null())
            throw std::runtime_error("Free-point TOGT requires the optimized center result");
        const auto began=Clock::now();
        if(supplied_time_coordinates.size() && (!center_ || supplied_time_coordinates.size()!=n_ || !supplied_time_coordinates.allFinite()))
            throw std::runtime_error("External timing coordinates require a matching center problem");
        const Eigen::VectorXd seed=supplied_time_coordinates.size() ? supplied_time_coordinates : center_result.is_null() ? initial() : initial_from_center(center_result);
        const auto starting=evaluate(seed);
        double point_error=0.,duration_error=0.,coefficient_error=0.;
        if(!center_result.is_null()) {
            for(int gate=0;gate<m_;++gate)for(int axis=0;axis<3;++axis)
                point_error=std::max(point_error,std::abs(starting.points(axis,gate)-center_result.at("crossing_points").at(gate).at(axis).get<double>()));
            for(int piece=0;piece<n_;++piece) {
                duration_error=std::max(duration_error,std::abs(starting.durations[piece]-center_result.at("durations").at(piece).get<double>()));
                for(int power=0;power<8;++power)for(int axis=0;axis<3;++axis)
                    coefficient_error=std::max(coefficient_error,std::abs(starting.coefficients[piece](power,axis)-center_result.at("coefficients").at(piece).at(power).at(axis).get<double>()));
            }
            if(point_error>1e-6 || duration_error>1e-10)
                throw std::runtime_error("Center result does not reproduce this scene's initial trajectory");
        }
        const double initialization_s=std::chrono::duration<double>(Clock::now()-began).count();
        Eigen::VectorXd x=seed;double objective=0.;evaluations_=iterations_=0;
        const auto solve_begin=Clock::now();
        const int status=drolib::lbfgs_optimize(x,objective,&Planner::cost,nullptr,&Planner::progress,this,lbfgs_.params);
        const double optimizer_s=std::chrono::duration<double>(Clock::now()-solve_begin).count();
        const auto final=evaluate(x);
        const bool found=status>=0 && std::isfinite(final.objective) && final.gradient.allFinite() &&
            final.durations.allFinite() && (final.durations.array()>0.).all();
        Json coefficients=Json::array(),points=Json::array(),crossings=Json::array();double arrival=0.;
        for(const auto& curve:final.coefficients)coefficients.push_back(curve_json(curve));
        for(int gate=0;gate<m_;++gate){points.push_back({final.points(0,gate),final.points(1,gate),final.points(2,gate)});arrival+=final.durations[gate];crossings.push_back(arrival);}
        return Json{{"found",found},{"method",center_?"dynamic_togt_center":"dynamic_togt"},
            {"runtime","C++ original TOGT 2024 MINCO, shape maps, dynamics cost and L-BFGS"},
            {"status",status},{"message",drolib::lbfgs_strerror(status)},
            {"iterations",iterations_},{"evaluations",evaluations_},
            {"initialization_origin",supplied_time_coordinates.size()?"archived independent segment timing":center_result.is_null()?"distance / speed guess":"optimized center result"},
            {"initial_objective",starting.objective},{"initial_trajectory_time_s",starting.durations.sum()},
            {"center_embedding_point_error_m",point_error},{"center_embedding_duration_error_s",duration_error},
            {"center_embedding_coefficient_error",coefficient_error},
            {"number_initial_starts",1},{"initial_sample_index",0},{"initial_variables",std::vector<double>(seed.data(),seed.data()+seed.size())},
            {"variables",std::vector<double>(x.data(),x.data()+x.size())},{"optimizer_variables",dimension_},
            {"point_variables",center_?0:dimension_-n_},{"internal_derivatives_optimized",false},
            {"durations",std::vector<double>(final.durations.data(),final.durations.data()+n_)},{"coefficients",coefficients},
            {"crossing_points",points},{"crossing_times_s",crossings},{"nominal_time_s",final.durations.sum()},
            {"objective",final.objective},{"dynamic_soft_integral",final.penalty},{"energy_cost",final.energy},
            {"relative_gradient_norm",final.gradient.cwiseAbs().maxCoeff()/std::max(1.,x.cwiseAbs().maxCoeff())},
            {"solve_s",std::chrono::duration<double>(Clock::now()-began).count()},{"initialization_s",initialization_s},
            {"optimizer_s",optimizer_s},{"gradient_inf_norm",final.gradient.cwiseAbs().maxCoeff()},
            {"config",metadata_},{"global_planning",true},{"all_seven_joint",m_==7},{"replanning",false},{"made_at",0.}};
    }
};
} // namespace dynamic_togt
