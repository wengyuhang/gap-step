#include "dynamic_togt.hpp"
#include <fstream>
#include <iostream>

class FixedPoint final : public drolib::ShapeBase {
public:
    explicit FixedPoint(const Vector& point){position=point;dim=0;}
    Eigen::VectorXd toD(const Vector&) const override {return Eigen::VectorXd(0);}
    Vector toP(const Eigen::VectorXd&) const override {return position;}
    void getGradD(const Eigen::VectorXd&,const Eigen::Matrix3Xd&,int,int,
                  Eigen::Map<Eigen::VectorXd>&) const override {}
};

Json verify(Json scene,bool center,const Json& vehicle) {
    using dynamic_togt::Planner;
    Planner planner(scene,center,TOGT_UPSTREAM_PARAMETER_DIR,vehicle);
    const auto seed=planner.initial();const auto initial=planner.evaluate(seed);
    Json static_scene=scene;for(auto& gate:static_scene["windows"])gate["amplitude"]=0.;
    Planner stationary(static_scene,center,TOGT_UPSTREAM_PARAMETER_DIR,vehicle);
    drolib::TrajSolver reference;reference.quad=stationary.quad();reference.tparams=stationary.params();
    reference.setConstYawBeforeTilt();reference.minco.setConditions(stationary.head(),stationary.tail(),planner.pieces());
    std::vector<int> native_offsets,reference_offsets;
    int native_offset=planner.pieces(),reference_offset=planner.pieces();
    std::vector<Window> windows;
    for(const auto& row:static_scene["windows"]) {
        const auto window=load_window(row);windows.push_back(window);
        dynamic_togt::SafePoint map(window,vehicle["finite_safety_radius_m"]);
        std::shared_ptr<drolib::ShapeBase> shape;
        if(center)shape=std::make_shared<FixedPoint>(window.center);
        else if(window.circle) {
            auto ball=std::make_shared<drolib::Ball>();ball->position=window.center;
            ball->radius=window.opening_radius+.085-vehicle["finite_safety_radius_m"].get<double>();
            ball->margin=0.;ball->initialize();shape=ball;
        } else {
            std::vector<Vector> vertices;
            for(const auto& vertex:inset_frame_vertices(window,vehicle["finite_safety_radius_m"]))
                vertices.push_back(window.center+window.rotation*Vector(vertex.x(),vertex.y(),0.));
            shape=std::make_shared<drolib::Polyhedron>(vertices);
        }
        reference.data.waypoints.emplace_back(shape);
        native_offsets.push_back(native_offset);reference_offsets.push_back(reference_offset);
        native_offset+=center?0:map.dimension;reference_offset+=shape->dimension();
    }
    reference.data.allocateSpace();
    double coefficient_error=0.,value_error=0.,static_gradient_error=0.,dynamic_gradient_error=0.;
    double nonzero_point_gradient=0.;
    for(int trial=0;trial<4;++trial) {
        auto x=stationary.initial();Eigen::VectorXd times=stationary.evaluate(x).durations;
        for(int piece=0;piece<times.size();++piece)times[piece]*=stationary.params().speedGuess*(.13+.02*trial)*(1.+.07*std::sin(piece+.3));
        Eigen::Map<Eigen::VectorXd> k(x.data(),planner.pieces());drolib::TrajSolver::backwardT(times,k);
        for(int j=planner.pieces();j<x.size();++j)x[j]+=.04*std::sin(j+.4*trial);
        reference.data.x.head(planner.pieces())=x.head(planner.pieces());
        for(size_t gate=0;gate<windows.size();++gate) {
            if(center)continue;
            if(windows[gate].circle) {
                Vector local=Vector::Zero();local.head<2>()=x.segment(native_offsets[gate],2);
                reference.data.x.segment(reference_offsets[gate],3)=windows[gate].rotation*local;
            } else {
                const int size=reference.data.waypoints[gate].shape->dimension();
                reference.data.x.segment(reference_offsets[gate],size)=x.segment(native_offsets[gate],size);
            }
        }
        Eigen::VectorXd reference_gradient(reference.data.x.size());
        const double expected=drolib::TrajSolver::costFunction(&reference,reference.data.x,reference_gradient);
        const auto actual=stationary.evaluate(x);
        value_error=std::max(value_error,std::abs(actual.objective-expected)/(1.+std::abs(expected)));
        Eigen::VectorXd projected=Eigen::VectorXd::Zero(x.size());projected.head(planner.pieces())=reference_gradient.head(planner.pieces());
        for(size_t gate=0;gate<windows.size();++gate) {
            if(center)continue;
            const int size=windows[gate].circle?2:reference.data.waypoints[gate].shape->dimension();
            if(windows[gate].circle)projected.segment(native_offsets[gate],2)=
                (windows[gate].rotation.transpose()*reference_gradient.segment(reference_offsets[gate],3)).head<2>();
            else projected.segment(native_offsets[gate],size)=reference_gradient.segment(reference_offsets[gate],size);
        }
        for(int j=0;j<x.size();++j)static_gradient_error=std::max(static_gradient_error,
            std::abs(actual.gradient[j]-projected[j])/(1.+std::abs(projected[j])));
        for(int piece=0;piece<planner.pieces();++piece)for(int power=0;power<8;++power)for(int axis=0;axis<3;++axis) {
            const double expected_coefficient=reference.minco.getCoeffs()(8*piece+power,axis)*std::pow(times[piece],power);
            coefficient_error=std::max(coefficient_error,std::abs(actual.coefficients[piece](power,axis)-expected_coefficient)/(1.+std::abs(expected_coefficient)));
        }
        const auto moving=planner.evaluate(x);
        if(!center)nonzero_point_gradient=std::max(nonzero_point_gradient,moving.gradient.tail(x.size()-planner.pieces()).cwiseAbs().maxCoeff());
        for(int j=0;j<x.size();++j) {
            const double h=2e-6*(1.+std::abs(x[j]));auto high=x,low=x;high[j]+=h;low[j]-=h;
            const double finite=(planner.evaluate(high).objective-planner.evaluate(low).objective)/(2.*h);
            dynamic_gradient_error=std::max(dynamic_gradient_error,
                std::abs(finite-moving.gradient[j])/(1.+std::abs(finite)+std::abs(moving.gradient[j])));
        }
    }
    double center_error=0.,motion_velocity_error=0.,initial_time_error=0.;
    double arrival=0.;Vector previous=planner.head().col(0);
    for(size_t gate=0;gate<windows.size();++gate) {
        auto window=load_window(scene["windows"][gate]);dynamic_togt::SafePoint map(window,vehicle["finite_safety_radius_m"]);
        arrival+=initial.durations[gate];
        const Vector local=center?Vector::Zero().eval():map.shape->position;
        const Vector initial_waypoint=window.center+window.rotation*(window.motion(0.)+local);
        initial_time_error=std::max(initial_time_error,std::abs(initial.durations[gate]-(initial_waypoint-previous).norm()/planner.params().speedGuess));
        previous=initial_waypoint;
        if(center)center_error=std::max(center_error,(initial.points.col(gate)-window.center-window.rotation*window.motion(arrival)).norm());
        for(double t:{.123,.732,1.351}) {
            constexpr double h=1e-6;const Vector finite=(window.motion(t+h)-window.motion(t-h))/(2*h);
            motion_velocity_error=std::max(motion_velocity_error,(finite-window.motion_velocity(t)).norm());
        }
    }
    Planner stopping_regression(scene,center,TOGT_UPSTREAM_PARAMETER_DIR,vehicle);
    Planner center_planner(scene,true,TOGT_UPSTREAM_PARAMETER_DIR,vehicle);
    const auto center_result=center_planner.solve();
    const auto solved=center ? stopping_regression.solve() : stopping_regression.solve(center_result);
    const auto& config=solved["config"];
    const bool stopping_passed=solved["found"].get<bool>() && solved["status"].get<int>()==drolib::LBFGS_STOP &&
        config["lbfgs_past"].get<int>()==3 && config["lbfgs_gradient_tolerance"].get<double>()==0. &&
        config["lbfgs_delta"].get<double>()==1e-5 && config["lbfgs_memory"].get<int>()==256 &&
        config["lbfgs_max_iterations"].get<int>()==0 && config["lbfgs_max_linesearch"].get<int>()==64 &&
        config["lbfgs_min_step"].get<double>()==1e-32 &&
        std::abs(stopping_regression.params().speedGuess-vehicle["initial_speed_guess_mps"].get<double>())<1e-12;
    const bool warm_start_passed=center ||
        (solved["initialization_origin"]=="optimized center result" &&
         solved["center_embedding_point_error_m"].get<double>()<1e-6 &&
         solved["center_embedding_duration_error_s"].get<double>()==0. &&
         std::abs(solved["initial_objective"].get<double>()-center_result["objective"].get<double>())<1e-6);
    const bool passed=stopping_passed && warm_start_passed && coefficient_error<1e-7 && value_error<1e-7 && static_gradient_error<1e-5 &&
        dynamic_gradient_error<2e-4 && center_error<1e-9 && motion_velocity_error<1e-6 && initial_time_error<1e-9 &&
        (center || nonzero_point_gradient>1e-4);
    return Json{{"center",center},{"passed",passed},{"author_stopping_regression_passed",stopping_passed},{"center_warm_start_passed",warm_start_passed},{"variables",planner.dimension()},
        {"point_variables",planner.dimension()-planner.pieces()},{"static_coefficient_error",coefficient_error},
        {"static_objective_error",value_error},{"static_gradient_error",static_gradient_error},
        {"dynamic_gradient_error",dynamic_gradient_error},{"active_point_gradient",nonzero_point_gradient},
        {"center_position_error",center_error},{"motion_velocity_error",motion_velocity_error},
        {"single_initial_duration_error",initial_time_error}};
}
int main(int argc,char** argv) {
    try {
        if(argc!=3)return 2;Json scene,vehicle;std::ifstream(argv[1])>>scene;std::ifstream(TOGT_X500_CONFIG)>>vehicle;
        Json checks=Json::array();checks.push_back(verify(scene,false,vehicle));checks.push_back(verify(scene,true,vehicle));
        bool passed=true;for(const auto& row:checks)passed=passed&&row["passed"].get<bool>();
        Json result{{"passed",passed},{"checks",checks}};std::ofstream(argv[2])<<result.dump(2)<<'\n';std::cout<<result.dump(2)<<'\n';return passed?0:1;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
