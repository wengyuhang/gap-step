#include "restored_kernels.hpp"
#include <fstream>

int main(int argc,char** argv) {
    try {
        if(argc!=5)throw std::runtime_error("Usage: restored_unit GRAPH SCENE LEGACY_GRAPH OUTPUT");
        using casadi::DM;using casadi::MX;using casadi::Function;
        Json meta,scene,model;
        std::ifstream(std::string(argv[1])+"/manifest.json")>>meta;
        std::ifstream(argv[2])>>scene;
        std::ifstream(std::string(argv[1])+"/exchange_dynamics.json")>>model;
        meta["dynamic_residual_scale"]=model["residual_scale"];
        std::vector<Window> windows;for(const auto& row:meta["windows"])windows.push_back(load_window(row));
        const Function dynamic=restored_kernels::dynamic_node(model);
        native_minco::Graph graph(meta,scene,windows,dynamic,1000.);
        const int n=meta["n"],nv=meta["nvar"],vs=meta["velocity_start"],aux=meta["slack_scale"].size();
        auto x=meta["initials"][0]["original"].get<std::vector<double>>();
        const auto scales=meta["state_scale"].get<std::vector<double>>();
        for(int k=0;k<n;++k){double t=std::exp(x[k]);x[k]=t>1.?std::sqrt(2*t-1)-1:1-std::sqrt(2/t-1);}
        for(int k=vs;k<nv;++k)x[k]/=scales[k-vs];
        x.resize(nv+aux,0.);
        const DM params(meta["initials"][0]["parameters"].get<std::vector<double>>());
        const auto curve=graph.trajectory(std::vector<DM>{DM(x),params});
        const auto coefficients=curve[2].get_elements(),times=curve[1].get_elements();
        const auto old_dynamic=Function::load(std::string(argv[3])+"/exchange_dynamic_node.casadi");
        double dynamics_error=0.,frames_error=0.;
        for(int piece=0;piece<n;++piece) {
            std::vector<double> state(coefficients.begin()+24*piece,coefficients.begin()+24*(piece+1));
            state.push_back(times[piece]);
            for(double u:{0.,.123,.5,.91,1.}) {
                const auto a=dynamic(std::vector<DM>{DM(state),DM(u)})[0].get_elements();
                const auto b=old_dynamic(std::vector<DM>{DM(state),DM(u)})[0].get_elements();
                for(int k=0;k<6;++k)dynamics_error=std::max(dynamics_error,std::abs(a[k]-b[k])/(1.+std::abs(b[k])));
            }
        }
        double start=0.;
        for(size_t gate=0;gate<windows.size();++gate) {
            const int piece=gate;
            std::vector<double> state(coefficients.begin()+24*piece,coefficients.begin()+24*(piece+1));
            state.push_back(times[piece]);state.push_back(start);
            for(const std::string kind:{"before","after","normalized"}) {
                const auto current=restored_kernels::frame_node(windows[gate],kind,meta["finite_radius"]);
                const auto old=Function::load(std::string(argv[3])+"/frame_"+std::to_string(gate)+"_"+kind+".casadi");
                for(double u:{0.,.03,.3,.9}) {
                    const auto a=current(std::vector<DM>{DM(state),DM(u)})[0].get_elements();
                    const auto b=old(std::vector<DM>{DM(state),DM(u)})[0].get_elements();
                    for(size_t k=0;k<a.size();++k)frames_error=std::max(frames_error,std::abs(a[k]-b[k])/(1.+std::abs(b[k])));
                }
            }
            start+=times[piece];
        }
        const auto residual=graph.residuals(std::vector<DM>{DM(x),params})[0].get_elements();
        const auto slack_scale=meta["slack_scale"].get<std::vector<double>>();
        for(int k=0;k<aux;++k)x[nv+k]=std::max(0.,residual[k])/slack_scale[k]+.2;
        std::vector<double> weights(graph.expressions[6].numel());
        for(size_t k=0;k<weights.size();++k)weights[k]=.001*std::sin(.19*k+.4);
        const MX value=graph.expressions[5]+MX::dot(DM(weights),graph.expressions[6]);
        const Function probe("restored_probe",{graph.expressions[0],graph.expressions[1]},
            {value,MX::gradient(value,graph.expressions[0])});
        const auto gradient=probe(std::vector<DM>{DM(x),params})[1].get_elements();
        double gradient_error=0.;
        for(int group=0;group<4;++group) {
            auto plus=x,minus=x;double automatic=0.;constexpr double h=1e-6;
            for(size_t k=0;k<x.size();++k) {
                const bool selected=(group==0 && k<static_cast<size_t>(n)) ||
                    (group==1 && k>=static_cast<size_t>(n) && k<static_cast<size_t>(vs)) ||
                    (group==2 && k>=static_cast<size_t>(vs) && k<static_cast<size_t>(nv)) || (group==3 && k>=static_cast<size_t>(nv));
                const double d=selected?std::sin(.23*k+.4*group):0.;
                plus[k]+=h*d;minus[k]-=h*d;automatic+=gradient[k]*d;
            }
            const double finite=(static_cast<double>(probe(std::vector<DM>{DM(plus),params})[0])-static_cast<double>(probe(std::vector<DM>{DM(minus),params})[0]))/(2*h);
            gradient_error=std::max(gradient_error,std::abs(finite-automatic)/(1.+std::abs(automatic)));
        }
        const bool passed=dynamics_error<1e-9 && frames_error<1e-9 && gradient_error<3e-5 &&
            graph.expressions[6].numel()==static_cast<int>(meta["lower_g"].size());
        Json result{{"passed",passed},{"dynamic_kernel_relative_error",dynamics_error},{"frame_kernel_relative_error",frames_error},
            {"gradient_relative_error",gradient_error},{"trajectory_variables",nv},{"constraint_rows",graph.expressions[6].numel()}};
        std::ofstream(argv[4])<<result.dump(2)<<'\n';std::cout<<result.dump(2)<<'\n';return passed?0:1;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 2;}
}
