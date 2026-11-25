#include "native_minco_pvaj.hpp"
#include "restored_kernels.hpp"
#include <iostream>
using casadi::DM; using casadi::MX; using casadi::Function;
Json read(const std::string& file) {Json value;std::ifstream(file)>>value;return value;}
std::vector<double> initial(native_minco::Graph& graph,const Json& m,const DM& p,bool auxiliary) {
    const int n=m["n"],nv=m["nvar"],vs=m["velocity_start"];
    auto x=m["initials"][0]["original"].get<std::vector<double>>();
    x.resize(nv+(auxiliary?m["slack_scale"].size():0),0.);
    for(int k=0;k<n;++k){double t=std::exp(x[k]);x[k]=t>1.?std::sqrt(2.*t-1.)-1.:1.-std::sqrt(2./t-1.);}
    if(m["free_points"].get<bool>())for(int gate=0;gate<m["m"];++gate)for(int axis=0;axis<m["point_scale_m"][gate].size();++axis)
        x[m["slices"][gate]["begin"].get<int>()+axis]/=m["point_scale_m"][gate][axis].get<double>();
    const auto units=m["state_scale"].get<std::vector<double>>();for(int k=vs;k<nv;++k)x[k]/=units[k-vs];
    if(auxiliary){const auto r=graph.residuals(std::vector<DM>{DM(x),p})[0].get_elements();for(int k=0;k<r.size();++k)x[nv+k]=std::max(r[k],0.)/m["slack_scale"][k].get<double>();}
    return x;
}
int main(int argc,char**argv) {
    try {
        if(argc!=4 && argc!=5)throw std::runtime_error("INPUT_DIRECTORY SCENE OUTPUT [SEED]");
        auto m=read(std::string(argv[1])+"/manifest.json"),scene=read(argv[2]),model=read(std::string(argv[1])+"/exchange_dynamics.json");
        if(argc==5)m["initials"][0]=m["initials"].at(std::stoi(argv[4]));
        model["continuous_attitude_chart"]=true;
        const std::string form="residual_epigraph";
        native_minco::remove_domain_bounds(m);native_minco::use_derivative_order(m,3);
        const int n=m["n"],nv=m["nvar"],vs=m["velocity_start"];
        m["dynamic_residual_scale"]=model["residual_scale"];
        std::vector<double> scales;for(int k=0;k<n*17*6;++k)scales.push_back(.01/model["residual_scale"][k%6].get<double>());
        m["slack_scale"]=scales;m["native_soft_formulation"]=form;
        std::vector<Window> windows;for(const auto& row:m["windows"])windows.push_back(load_window(row));
        const auto node=restored_kernels::dynamic_node(model);
        native_minco::Graph lifted(m,scene,windows,node,1000.);
        const DM parameters(m["initials"][0]["parameters"].get<std::vector<double>>());
        const auto xl=initial(lifted,m,parameters,true);
        const auto lv=lifted.trajectory(std::vector<DM>{DM(xl),parameters});
        const double lf=static_cast<double>(Function("lifted_f",{lifted.expressions[0],lifted.expressions[1]},{lifted.expressions[5]})(std::vector<DM>{DM(xl),parameters})[0]);
        m["native_soft_formulation"]="direct";m["slack_scale"]=Json::array();
        native_minco::Graph direct(m,scene,windows,node,1000.);
        const auto x=initial(direct,m,parameters,false);const auto dv=direct.trajectory(std::vector<DM>{DM(x),parameters});
        const auto z=direct.expressions[0],p=direct.expressions[1],f=direct.expressions[5];
        const Function fg("direct_f_gradient",{z,p},{f,MX::gradient(f,z)});
        const auto actual=fg(std::vector<DM>{DM(x),parameters});const auto gradient=actual[1].get_elements();
        double coefficient_error=0.,derivative_error=0.;const auto a=lv[2].get_elements(),b=dv[2].get_elements();
        for(int k=0;k<a.size();++k)coefficient_error=std::max(coefficient_error,std::abs(a[k]-b[k])/(1.+std::abs(a[k])));
        std::vector<int> columns;for(int k=0;k<nv;++k)columns.push_back(k);
        Json bad=Json::array();double gradmax=0.;
        for(int col:columns){double best=INFINITY,bestfd=0.,beststep=0.;Json probes=Json::array();gradmax=std::max(gradmax,std::abs(gradient[col]));
          for(double step:{1e-4,1e-5,1e-6,1e-7,1e-8,1e-9,1e-10}) {
            auto xa=x,xb=x;const double h=step*(1.+std::abs(x[col]));xa[col]+=h;xb[col]-=h;
            const double fd=(static_cast<double>(fg(std::vector<DM>{DM(xa),parameters})[0])-static_cast<double>(fg(std::vector<DM>{DM(xb),parameters})[0]))/(2.*h);
            const double err=std::abs(fd-gradient[col])/std::max({1.,std::abs(fd),std::abs(gradient[col])});probes.push_back({step,fd,err});
            if(err<best){best=err;bestfd=fd;beststep=step;}
          }
          derivative_error=std::max(derivative_error,best);
          if(best>1e-3)bad.push_back({{"column",col},{"ad",gradient[col]},{"best_fd",bestfd},{"best_step",beststep},{"best_error",best},{"probes",probes}});
        }
        const double objective_error=std::abs(lf-static_cast<double>(actual[0]))/(1.+std::abs(lf));
        const bool passed=objective_error<1e-10 && coefficient_error<1e-10 && derivative_error<1e-3 && direct.expressions[0].numel()==nv && direct.residuals.size1_out(0)==n*17*6;
        const Json report{{"passed",passed},{"auxiliary_formulation",form},{"actual_penalty",static_cast<double>(dv[5])},{"quadrature_intervals",16},{"direct_variables",nv},{"lifted_variables",xl.size()},
            {"objective_relative_error",objective_error},{"coefficient_relative_error",coefficient_error},{"gradient_relative_error",derivative_error},{"gradient_inf",gradmax},{"bad_columns",bad}};
        std::ofstream(argv[3])<<report.dump(2)<<'\n';std::cout<<report.dump(2)<<'\n';return passed?0:1;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
