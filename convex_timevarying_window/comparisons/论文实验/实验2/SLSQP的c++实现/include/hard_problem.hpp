#pragma once
#include "restored_kernels.hpp"
#include "slsqp.hpp"
#include <map>
#include <memory>

namespace hard_problem {
using casadi::MX;using casadi::DM;using casadi::Function;
using Key=std::pair<int,int>;
struct Witness {std::string kind;std::vector<double> values;};
using Witnesses=std::map<Key,Witness>;
inline Json read(const std::string& path){Json value;std::ifstream input(path);if(!input)throw std::runtime_error("Missing "+path);input>>value;return value;}
struct Finite {
    Function values,derivatives,geometry,dynamics;
    int equalities=0,constraints=0,dynamic_rows=0;
};
class Model {
public:
    Json metadata,scene,dynamic_model;
    std::vector<Window> windows;
    Function dynamic;
    std::unique_ptr<native_minco::Graph> graph;
    Model(const std::string& input,const std::string& scene_path) {
        metadata=read(input+"/manifest.json");scene=read(scene_path);dynamic_model=read(input+"/exchange_dynamics.json");
        metadata["restore_historical_domain"]=true;
        for(const auto& row:metadata["windows"])windows.push_back(load_window(row));
        dynamic=restored_kernels::dynamic_node(dynamic_model);
        graph=std::make_unique<native_minco::Graph>(metadata,scene,windows);
    }
    std::vector<double> initial(int seed) const {
        auto x=metadata["initials"][seed]["original"].get<std::vector<double>>();
        const int n=metadata["n"],vs=metadata["velocity_start"],nv=metadata["nvar"];
        const auto scale=metadata["state_scale"].get<std::vector<double>>();x.resize(nv);
        for(int i=0;i<n;++i){const double t=std::exp(x[i]);x[i]=t>1.?std::sqrt(2*t-1)-1:1-std::sqrt(2/t-1);}
        for(int i=vs;i<nv;++i)x[i]/=scale[i-vs];
        return x;
    }
    Finite build(const Witnesses& witnesses) const {
        const auto& e=graph->expressions;
        const MX& x=e[0];const MX& parameters=e[1];const MX times=MX::exp(e[2](casadi::Slice(0,metadata["n"].get<int>())));
        const int n=metadata["n"],m=metadata["m"];
        std::vector<MX> eq,ineq;
        const MX& base=e[6];
        for(int i=0;i<base.numel();++i) {
            const double lower=metadata["lower_g"][i];
            const bool upper_finite=!metadata["upper_g"][i].is_null();
            const double upper=upper_finite?metadata["upper_g"][i].get<double>():INFINITY;
            if(lower==upper){eq.push_back(base(i)-lower);continue;}
            if(std::isfinite(lower))ineq.push_back(base(i)-lower);
            if(std::isfinite(upper))ineq.push_back(upper-base(i));
        }
        const MX starts=MX::vertcat({MX(0.),MX::cumsum(times)(casadi::Slice(0,n-1))});
        for(const auto& item:witnesses) {
            const int gate=item.first.first,piece=item.first.second;
            const auto& witness=item.second;
            if(witness.values.empty())continue;
            const auto frame=restored_kernels::frame_node(windows[gate],witness.kind,metadata["finite_radius"]);
            const MX state=MX::vertcat({e[4](casadi::Slice(24*piece,24*(piece+1))),times(piece),starts(piece)});
            for(double offset:witness.values) {
                const bool knot=(witness.kind=="before" && piece==gate) || (witness.kind=="after" && piece==gate+1);
                const MX value=knot && offset==0.?e[7+gate]:frame(std::vector<MX>{state,MX(offset)})[0];
                ineq.push_back(value);
            }
        }
        const MX geometry=MX::vertcat(ineq);
        std::vector<MX> dynamic_rows;
        for(int piece=0;piece<n;++piece) {
            const MX state=MX::vertcat({e[4](casadi::Slice(24*piece,24*(piece+1))),times(piece)});
            for(int node=0;node<17;++node)dynamic_rows.push_back(dynamic(std::vector<MX>{state,MX(node/16.)})[0]);
        }
        const MX residuals=MX::vertcat(dynamic_rows);
        const MX equalities=eq.empty()?MX::zeros(0,1):MX::vertcat(eq);
        const MX c=MX::vertcat({equalities,geometry,-residuals});
        Finite finite;finite.equalities=equalities.numel();finite.constraints=c.numel();finite.dynamic_rows=residuals.numel();
        finite.values=Function("finite_slsqp_values",{x,parameters},{e[5],c});
        finite.derivatives=Function("finite_slsqp_derivatives",{x,parameters},
            {MX::densify(MX::gradient(e[5],x)),MX::densify(MX::jacobian(c,x))});
        finite.geometry=Function("finite_geometry",{x,parameters},{geometry});
        finite.dynamics=Function("finite_sampled_dynamics",{x,parameters},{residuals});
        return finite;
    }
};
}
