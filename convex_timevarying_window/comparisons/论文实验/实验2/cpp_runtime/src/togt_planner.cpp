#include <casadi/casadi.hpp>
#include "geometry.hpp"
#include "lbfgsb.hpp"
#include "drolib/solver/minco_snap.hpp"
#include <chrono>
#include <filesystem>
#include <numeric>

using casadi::DM;
using casadi::Function;

struct Evaluation {
    double cost;
    std::vector<double> gradient,times,points;
    std::vector<Curve> coefficients;
};
class Objective {
    Function geometry,penalty;
    int n;
    Eigen::Matrix<double,3,4> head,tail;
public:
    Objective(const std::string& directory,const Json& metadata):n(metadata.at("n")) {
        geometry=Function::load(directory+"/geometry.casadi");penalty=Function::load(directory+"/penalty.casadi");
        head.setZero();tail.setZero();
        for(int axis=0;axis<3;++axis){head(axis,0)=metadata["start"][axis];tail(axis,0)=metadata["goal"][axis];}
    }
    Evaluation evaluate(const std::vector<double>& x) {
        auto shape=geometry(std::vector<DM>{DM(x)});
        const auto values=shape[0].get_elements(),jacobian=shape[1].get_elements();
        Evaluation result;result.times.assign(values.begin(),values.begin()+n);result.points.assign(values.begin()+n,values.end());
        Eigen::VectorXd times(n);Eigen::Matrix3Xd points(3,n-1);
        for(int k=0;k<n;++k) {times[k]=result.times[k];if(!(times[k]>0.) || !std::isfinite(times[k]))throw std::runtime_error("Invalid TOGT duration");}
        for(int gate=0;gate<n-1;++gate)for(int axis=0;axis<3;++axis)points(axis,gate)=result.points[3*gate+axis];
        drolib::MincoSnap minco;minco.setConditions(head,tail,n);minco.setParameters(points,times);
        const auto coefficients=minco.getCoeffs();Eigen::MatrixX3d partial=Eigen::MatrixX3d::Zero(8*n,3);
        Eigen::VectorXd direct_times=Eigen::VectorXd::Zero(n);result.cost=times.sum();
        for(int piece=0;piece<n;++piece) {
            std::vector<double> state(25);Curve normalized;
            for(int axis=0;axis<3;++axis)for(int power=0;power<8;++power) {
                normalized(power,axis)=coefficients(8*piece+power,axis)*std::pow(times[piece],power);
                state[8*axis+power]=normalized(power,axis);
            }
            state[24]=times[piece];result.coefficients.push_back(normalized);
            auto outputs=penalty(std::vector<DM>{DM(state)});result.cost+=static_cast<double>(outputs[0]);
            const auto derivative=outputs[1].get_elements();direct_times[piece]=derivative[24];
            for(int axis=0;axis<3;++axis)for(int power=0;power<8;++power) {
                const double gradient=derivative[8*axis+power];
                partial(8*piece+power,axis)=gradient*std::pow(times[piece],power);
                direct_times[piece]+=power*normalized(power,axis)*gradient/times[piece];
            }
        }
        Eigen::Matrix3Xd point_gradient(3,n-1);Eigen::VectorXd time_gradient(n);
        minco.propagateGrad(partial,direct_times,point_gradient,time_gradient);time_gradient.array()+=1.;
        std::vector<double> geometry_gradient(values.size());
        for(int piece=0;piece<n;++piece)geometry_gradient[piece]=time_gradient[piece];
        for(int gate=0;gate<n-1;++gate)for(int axis=0;axis<3;++axis)geometry_gradient[n+3*gate+axis]=point_gradient(axis,gate);
        result.gradient.assign(x.size(),0.);
        for(size_t column=0;column<x.size();++column)for(size_t row=0;row<values.size();++row)
            result.gradient[column]+=jacobian[column*values.size()+row]*geometry_gradient[row];
        if(!std::isfinite(result.cost) || !std::all_of(result.gradient.begin(),result.gradient.end(),[](double v){return std::isfinite(v);}))
            throw std::runtime_error("Nonfinite TOGT value or gradient");
        return result;
    }
};

int main(int argc,char** argv) {
    try {
        if(argc!=5)throw std::runtime_error("Usage: togt_planner GRAPH_DIRECTORY OUTPUT_DIRECTORY verify|solve SEED_INDEX");
        const std::string directory=argv[1],output=argv[2],mode=argv[3];
        Json manifest;std::ifstream(directory+"/manifest.json")>>manifest;
        std::filesystem::create_directories(output);Objective objective(directory,manifest);
        if(mode=="verify") {
            Json checks=Json::array();
            for(const auto& initial:manifest["initials"]) {
                auto actual=objective.evaluate(initial["variables"].get<std::vector<double>>());
                double value_error=std::abs(actual.cost-initial["expected_cost"].get<double>())/(1.+std::abs(initial["expected_cost"].get<double>()));
                double gradient_error=0.,coefficient_error=0.;
                for(size_t k=0;k<actual.gradient.size();++k)gradient_error=std::max(gradient_error,
                    std::abs(actual.gradient[k]-initial["expected_gradient"][k].get<double>())/(1.+std::abs(initial["expected_gradient"][k].get<double>())));
                for(size_t piece=0;piece<actual.coefficients.size();++piece)for(int i=0;i<8;++i)for(int j=0;j<3;++j)
                    coefficient_error=std::max(coefficient_error,std::abs(actual.coefficients[piece](i,j)-initial["expected_coefficients"][piece][i][j].get<double>()));
                const bool passed=value_error<1e-7 && gradient_error<1e-5 && coefficient_error<1e-5;
                checks.push_back(Json{{"seed",initial["index"]},{"passed",passed},{"value_relative_error",value_error},
                    {"gradient_relative_error",gradient_error},{"coefficient_absolute_error",coefficient_error}});
                if(!passed){std::ofstream(output+"/validation.json")<<checks.dump(2)<<'\n';throw std::runtime_error("Native TOGT parity failed");}
            }
            std::ofstream(output+"/validation.json")<<Json{{"passed",true},{"checks",checks}}.dump(2)<<'\n';
            std::cout<<"Native TOGT parity passed for "<<checks.size()<<" starts\n";
        } else if(mode=="solve") {
            const int index=std::stoi(argv[4]);const auto initial=manifest["initials"][index]["variables"].get<std::vector<double>>();
            LBFGSBOptions settings;settings.memory=manifest["memory"];settings.max_linesearch=manifest["max_linesearch"];
            settings.max_iterations=manifest["max_iterations"];settings.past=manifest["past"];settings.past_tolerance=manifest["past_tolerance"];settings.gtol=manifest["gtol"];
            const double invalid_cost=manifest["invalid_trial_cost"];
            int invalid_trials=0;
            auto callback=[&](const std::vector<double>& x,std::vector<double>& gradient) {
                try {auto evaluation=objective.evaluate(x);gradient=evaluation.gradient;return evaluation.cost;}
                catch(const std::exception&) {
                    ++invalid_trials;double squared=0.;
                    for(size_t k=0;k<x.size();++k){const double value=std::clamp(x[k],-1e6,1e6);squared+=value*value;gradient[k]=2e-12*invalid_cost*value;}
                    return invalid_cost*(1.+1e-12*squared);
                }
            };
            const auto began=std::chrono::steady_clock::now();
            const auto solved=minimize_lbfgsb(callback,initial,std::vector<double>(initial.size(),-INFINITY),std::vector<double>(initial.size(),INFINITY),settings);
            Json result{{"found",solved.success},{"solve_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()},
                {"method",manifest["method"]},{"case",Json{{"layout","togt_whole"},{"level",manifest["level"]}}},
                {"initial_sample_index",index},{"initial_variables",initial},{"variables",solved.x},
                {"iterations",solved.iterations},{"evaluations",solved.evaluations},{"status",solved.status},{"message",solved.message},
                {"past_stop",solved.past_stop},{"invalid_trial_count",invalid_trials},{"runtime","C++ native MINCO adjoint and SciPy 1.15.3 C L-BFGS-B"}};
            if(solved.success) {
                const auto evaluation=objective.evaluate(solved.x);Json coefficients=Json::array(),points=Json::array();
                for(const auto& curve:evaluation.coefficients){Json c=Json::array();for(int k=0;k<8;++k)c.push_back({curve(k,0),curve(k,1),curve(k,2)});coefficients.push_back(c);}
                for(int gate=0;gate<manifest["m"];++gate)points.push_back({evaluation.points[3*gate],evaluation.points[3*gate+1],evaluation.points[3*gate+2]});
                const double total=std::accumulate(evaluation.times.begin(),evaluation.times.end(),0.);
                std::vector<double> crossing_times;double at=0.;for(size_t k=0;k+1<evaluation.times.size();++k){at+=evaluation.times[k];crossing_times.push_back(at);}
                result.update(Json{{"coefficients",coefficients},{"durations",evaluation.times},{"nominal_time_s",total},
                    {"objective",evaluation.cost},{"dynamic_soft_integral",evaluation.cost-total},{"crossing_points",points},{"crossing_times_s",crossing_times},{"made_at",0.}});
            }
            std::ofstream(output+"/result.json")<<result.dump(2)<<'\n';std::cout<<"TOGT_RESULT "<<result["found"]<<' '<<result.value("nominal_time_s",0.)<<'\n';
        } else throw std::runtime_error("Unknown TOGT mode");
    } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
