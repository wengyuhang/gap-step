#include "native_minco_pvaj.hpp"
#include "restored_kernels.hpp"
#include "minco_tangent_metric.hpp"
#include <iostream>
#include <Eigen/Eigenvalues>
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
    if(auxiliary){const auto r=graph.residuals(std::vector<DM>{DM(x),p})[0].get_elements();for(int k=0;k<r.size();++k)x[nv+k]=(m.value("native_soft_formulation",std::string())=="residual_equality"?r[k]:std::max(r[k],0.))/m["slack_scale"][k].get<double>();}
    if(auxiliary && m["native_soft_formulation"]=="cost_epigraph") {const auto r=graph.residuals(std::vector<DM>{DM(x),p})[0].get_elements();for(int k=0;k<r.size();++k){const double v=r[k]*m["dynamic_residual_scale"][k%6].get<double>();x[nv+k]=(v<=0.?0.:v>=.01?v-.005:(.01-.5*v)*std::pow(v/.01,3))/m["slack_scale"][k].get<double>();}}
    return x;
}
int main(int argc,char**argv) {
    try {
        if(argc!=4 && argc!=5)throw std::runtime_error("INPUT_DIRECTORY SCENE OUTPUT [SEED]");
        auto m=read(std::string(argv[1])+"/manifest.json"),scene=read(argv[2]),model=read(std::string(argv[1])+"/exchange_dynamics.json");
        if(argc==5)m["initials"][0]=m["initials"].at(std::stoi(argv[4]));
        const std::string selected=m.value("native_soft_formulation",std::string("residual_epigraph"));
        const std::string form=selected=="cost_epigraph"?selected:selected=="residual_equality"?selected:"residual_epigraph";
        m.erase("native_hessian");
        m.erase("lift_minco_coefficients");
        m.erase("lift_world_points");
        m.erase("lift_arrival_times");
        m.erase("hermite_dynamics_evaluation");
        m.erase("bernstein_coefficient_coordinates");
        native_minco::remove_domain_bounds(m);native_minco::use_derivative_order(m,3);
        const int n=m["n"],nv=m["nvar"],vs=m["velocity_start"];
        m["dynamic_residual_scale"]=model["residual_scale"];
        model["continuous_attitude_chart"]=true;
        std::vector<double> scales;for(int k=0;k<n*17*6;++k)scales.push_back(form=="cost_epigraph"?.01:m.value("normalized_auxiliary_units",false)?1.:.01/model["residual_scale"][k%6].get<double>());
        m["slack_scale"]=scales;m["native_soft_formulation"]=form;
        std::vector<Window> windows;for(const auto& row:m["windows"])windows.push_back(load_window(row));
        const auto node=restored_kernels::dynamic_node(model);
        native_minco::Graph lifted(m,scene,windows,node,1000.);
        const DM parameters(m["initials"][0]["parameters"].get<std::vector<double>>());
        const auto xl=initial(lifted,m,parameters,true);
        const auto lv=lifted.trajectory(std::vector<DM>{DM(xl),parameters});
        const double lf=static_cast<double>(Function("lifted_f",{lifted.expressions[0],lifted.expressions[1]},{lifted.expressions[5]})(std::vector<DM>{DM(xl),parameters})[0]);
        m["native_soft_formulation"]="direct";m["slack_scale"]=Json::array();
        m["native_hessian"]="minco_tangent";
        native_minco::Graph direct(m,scene,windows,node,1000.);
        const auto x=initial(direct,m,parameters,false);const auto dv=direct.trajectory(std::vector<DM>{DM(x),parameters});
        const auto z=direct.expressions[0],p=direct.expressions[1],f=direct.expressions[5];
        const Function fg("direct_f_gradient",{z,p},{f,MX::gradient(f,z)});
        const auto actual=fg(std::vector<DM>{DM(x),parameters});const auto gradient=actual[1].get_elements();
        auto hermite_metadata=m;hermite_metadata.erase("native_hessian");hermite_metadata["hermite_dynamics_evaluation"]=true;
        native_minco::Graph hermite(hermite_metadata,scene,windows,restored_kernels::dynamic_node(model,true),1000.);
        const auto hz=hermite.expressions[0],hp=hermite.expressions[1],hf=hermite.expressions[5];
        const auto hermite_value=Function("hermite_f_gradient",{hz,hp},{hf,MX::gradient(hf,hz)})(std::vector<DM>{DM(x),parameters});
        const double hermite_objective_error=std::abs(double(hermite_value[0])-double(actual[0]))/(1.+std::abs(double(actual[0])));
        double hermite_gradient_error=0.,hermite_residual_error=0.;const auto hermite_gradient=hermite_value[1].get_elements();
        for(int k=0;k<nv;++k)hermite_gradient_error=std::max(hermite_gradient_error,std::abs(hermite_gradient[k]-gradient[k])/std::max(1.,std::abs(gradient[k])));
        const auto hr=hermite.residuals(std::vector<DM>{DM(x),parameters})[0].get_elements(),dr=direct.residuals(std::vector<DM>{DM(x),parameters})[0].get_elements();
        for(int k=0;k<hr.size();++k)hermite_residual_error=std::max(hermite_residual_error,std::abs(hr[k]-dr[k])/(1.+std::abs(dr[k])));
        double auxiliary_elimination_gradient_error=0.;
        if(form=="cost_epigraph") {
            const MX r=direct.residuals(std::vector<MX>{z,p})[0];std::vector<MX> costs;
            for(int k=0;k<r.numel();++k){const MX v=r(k)*model["residual_scale"][k%6].get<double>();
                costs.push_back(MX::if_else(v<=0.,MX(0.),MX::if_else(v>=.01,v-.005,(.01-.5*v)*MX::pow(v/.01,3)))/scales[k]);}
            const Function lifted_f("cost_epigraph_elimination",{lifted.expressions[0],lifted.expressions[1]},{lifted.expressions[5]});
            const MX composed=lifted_f(std::vector<MX>{MX::vertcat({z,MX::vertcat(costs)}),p})[0];
            const auto composed_gradient=Function("eliminated_gradient",{z,p},{MX::gradient(composed,z)})(std::vector<DM>{DM(x),parameters})[0].get_elements();
            for(int k=0;k<nv;++k)auxiliary_elimination_gradient_error=std::max(auxiliary_elimination_gradient_error,std::abs(composed_gradient[k]-gradient[k])/std::max(1.,std::abs(gradient[k])));
        }
        MincoTangentMetric tangent(direct.interpolation_inputs,Function(),m,scene,parameters.numel(),direct.expressions[6].numel());
        const Eigen::MatrixXd tangent_jacobian=tangent.tangent(DM(x),parameters);double tangent_error=0.;
        for(int probe=0;probe<3;++probe){Eigen::VectorXd direction(vs);for(int k=0;k<vs;++k)direction[k]=std::sin((k+1.)*(probe+1.))/std::sqrt(double(vs));const Eigen::VectorXd expected=tangent_jacobian*direction;double best=INFINITY;
            for(double step:{1e-4,1e-5,1e-6,1e-7}){auto xa=x,xb=x;for(int k=0;k<vs;++k){xa[k]+=step*direction[k];xb[k]-=step*direction[k];}
                const Eigen::VectorXd finite=(tangent.interpolated_states(DM(xa),parameters)-tangent.interpolated_states(DM(xb),parameters))/(2.*step);
                best=std::min(best,(finite-expected).lpNorm<Eigen::Infinity>()/std::max(1.,expected.lpNorm<Eigen::Infinity>()));}
            tangent_error=std::max(tangent_error,best);}
        // Known-answer metric check: -M pulled into the tangent basis is -I;
        // clipping at one and pulling back must recover M exactly.
        Eigen::MatrixXd metric=Eigen::MatrixXd::Identity(nv,nv);
        metric.topLeftCorner(vs,vs)+=tangent_jacobian.transpose()*tangent_jacobian;
        metric.topRightCorner(vs,nv-vs)=-tangent_jacobian.transpose();
        metric.bottomLeftCorner(nv-vs,vs)=-tangent_jacobian;
        DM negative_metric=DM::zeros(nv,nv);
        for(int j=0;j<nv;++j)for(int i=0;i<nv;++i)negative_metric(i,j)=-metric(i,j);
        const MX test_lf=MX::sym("test_lf"),test_lg=MX::sym("test_lg",direct.expressions[6].numel());
        tangent.set_exact_hessian(Function("known_metric",{z,p,test_lf,test_lg},{MX(negative_metric)}));
        const auto clipped=tangent.eval({DM(x),parameters,DM(1.),DM::zeros(test_lg.numel())})[0];
        double metric_error=0.;
        for(int j=0;j<nv;++j)for(int i=0;i<=j;++i)metric_error=std::max(metric_error,std::abs(double(clipped(i,j))-metric(i,j)));
        metric_error/=std::max(1.,metric.cwiseAbs().maxCoeff());
        double lift_gradient_error=0.,lift_objective_error=0.,lift_coefficient_error=0.,lift_residual_error=0.,lift_link_error=0.;
        for(bool bernstein:{false,true}) {
        auto lifted_metadata=m;lifted_metadata.erase("native_hessian");lifted_metadata["lift_minco_coefficients"]=true;
        lifted_metadata["bernstein_coefficient_coordinates"]=bernstein;
        lifted_metadata["hermite_dynamics_evaluation"]=bernstein;
        native_minco::Graph coefficient_lift(lifted_metadata,scene,windows,restored_kernels::dynamic_node(model,bernstein),1000.);
        auto cx=x;cx.resize(nv+24*n,0.);const DM consistent=coefficient_lift.canonical_lift(std::vector<DM>{DM(cx),parameters})[0];
        const auto cz=coefficient_lift.expressions[0],cp=coefficient_lift.expressions[1];
        const Function raw_f("coefficient_objective",{cz,cp},{coefficient_lift.expressions[5]});
        const MX eliminated=raw_f(std::vector<MX>{coefficient_lift.canonical_lift(std::vector<MX>{cz,cp})[0],cp})[0];
        const auto eliminated_value=Function("coefficient_lift_elimination",{cz,cp},{eliminated,MX::gradient(eliminated,cz)})(std::vector<DM>{consistent,parameters});
        const auto lifted_gradient=eliminated_value[1].get_elements();
        for(int k=0;k<nv;++k)lift_gradient_error=std::max(lift_gradient_error,std::abs(lifted_gradient[k]-gradient[k])/std::max(1.,std::abs(gradient[k])));
        lift_objective_error=std::max(lift_objective_error,std::abs(double(eliminated_value[0])-double(actual[0]))/(1.+std::abs(double(actual[0]))));
        const auto cc=coefficient_lift.trajectory(std::vector<DM>{consistent,parameters})[2].get_elements(),dc=dv[2].get_elements();
        const auto cr=coefficient_lift.residuals(std::vector<DM>{consistent,parameters})[0].get_elements();
        for(int k=0;k<cc.size();++k)lift_coefficient_error=std::max(lift_coefficient_error,std::abs(cc[k]-dc[k])/(1.+std::abs(dc[k])));
        for(int k=0;k<cr.size();++k)lift_residual_error=std::max(lift_residual_error,std::abs(cr[k]-dr[k])/(1.+std::abs(dr[k])));
        const auto links=Function("lifted_links",{cz,cp},{coefficient_lift.expressions[6](casadi::Slice(coefficient_lift.expressions[6].numel()-24*n,coefficient_lift.expressions[6].numel()))})(std::vector<DM>{consistent,parameters})[0].get_elements();
        for(double v:links)lift_link_error=std::max(lift_link_error,std::abs(v));
        }
        double point_lift_error=0.;
        // Joint control/residual lifting must eliminate to the same objective,
        // including active soft penalties, without restricting control signs.
        double joint_lift_error=0.;
        for(bool joint_arrivals:{false,true}) for(bool signed_residual:{false,true}) {
            auto jm=m;jm.erase("native_hessian");jm["native_soft_formulation"]=signed_residual?"residual_equality":"residual_epigraph";
            jm["slack_scale"]=scales;jm["lift_minco_coefficients"]=true;
            jm["bernstein_coefficient_coordinates"]=true;jm["hermite_dynamics_evaluation"]=true;
            jm["lift_arrival_times"]=joint_arrivals;
            native_minco::Graph joint(jm,scene,windows,restored_kernels::dynamic_node(model,true),1000.);
            const auto jz=joint.expressions[0],jp=joint.expressions[1];
            const MX jc=joint.canonical_lift(std::vector<MX>{jz,jp})[0];
            const MX jr=joint.residuals(std::vector<MX>{jc,jp})[0];
            const MX eliminated_variables=MX::vertcat({jz(casadi::Slice(0,nv)),(signed_residual?jr:MX::fmax(jr,0.))/DM(scales),jc(casadi::Slice(nv+static_cast<int>(scales.size()),jc.numel()))});
            const Function jf("joint_raw_f",{jz,jp},{joint.expressions[5]});
            const MX je=jf(std::vector<MX>{eliminated_variables,jp})[0];
            auto jx=x;jx.resize(nv+scales.size()+24*n+(joint_arrivals?n:0),0.);
            const DM canonical_joint=joint.canonical_lift(std::vector<DM>{DM(jx),parameters})[0];
            const auto joint_coefficients=joint.trajectory(std::vector<DM>{canonical_joint,parameters})[2].get_elements();
            const auto original_coefficients=dv[2].get_elements();
            for(int k=0;k<joint_coefficients.size();++k)joint_lift_error=std::max(joint_lift_error,
                std::abs(joint_coefficients[k]-original_coefficients[k])/(1.+std::abs(original_coefficients[k])));
            const int equality_count=24*n+(joint_arrivals?n:0);
            const auto joint_links=Function("joint_equalities",{jz,jp},{joint.expressions[6](casadi::Slice(joint.expressions[6].numel()-equality_count,joint.expressions[6].numel()))})(std::vector<DM>{canonical_joint,parameters})[0].get_elements();
            for(double link:joint_links)joint_lift_error=std::max(joint_lift_error,std::abs(link));
            const auto jv=Function("joint_elimination",{jz,jp},{je,MX::gradient(je,jz)})(std::vector<DM>{DM(jx),parameters});
            joint_lift_error=std::max(joint_lift_error,std::abs(double(jv[0])-double(actual[0]))/(1.+std::abs(double(actual[0]))));
            const auto jgrad=jv[1].get_elements();
            for(int k=0;k<nv;++k)joint_lift_error=std::max(joint_lift_error,std::abs(jgrad[k]-gradient[k])/std::max(1.,std::abs(gradient[k])));
            for(int k=nv;k<jgrad.size();++k)joint_lift_error=std::max(joint_lift_error,std::abs(jgrad[k]));
            if(joint_lift_error>1e-7)throw std::runtime_error("Joint Bernstein/residual elimination mismatch");
        }
        for(bool arrivals:{false,true}) {
        auto point_metadata=m;point_metadata.erase("native_hessian");point_metadata["lift_world_points"]=true;point_metadata["lift_arrival_times"]=arrivals;
        native_minco::Graph point_lift(point_metadata,scene,windows,node,1000.);
        auto px=x;px.resize(nv+3*m["m"].get<int>()+(arrivals?n:0),0.);
        const DM point_consistent=point_lift.canonical_lift(std::vector<DM>{DM(px),parameters})[0];
        const auto pz=point_lift.expressions[0],pp=point_lift.expressions[1];
        const Function point_objective("point_lift_f",{pz,pp},{point_lift.expressions[5]});
        const MX point_composed=point_objective(std::vector<MX>{point_lift.canonical_lift(std::vector<MX>{pz,pp})[0],pp})[0];
        const auto point_value=Function("point_lift_elimination",{pz,pp},{point_composed,MX::gradient(point_composed,pz)})(std::vector<DM>{point_consistent,parameters});
        point_lift_error=std::max(point_lift_error,std::abs(double(point_value[0])-double(actual[0]))/(1.+std::abs(double(actual[0]))));
        const auto point_gradient=point_value[1].get_elements();
        for(int k=0;k<nv;++k)point_lift_error=std::max(point_lift_error,std::abs(point_gradient[k]-gradient[k])/std::max(1.,std::abs(gradient[k])));
        }
        const auto gn=direct.gauss_newton_hessian(std::vector<DM>{DM(x),parameters})[0].get_elements();
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(Eigen::Map<const Eigen::MatrixXd>(gn.data(),nv,nv));
        const double minimum_eigenvalue=spectrum.eigenvalues().minCoeff(),maximum_eigenvalue=spectrum.eigenvalues().maxCoeff();
        const bool psd=spectrum.info()==Eigen::Success && minimum_eigenvalue>=-1e-8*std::max(1.,maximum_eigenvalue);
        double coefficient_error=0.,derivative_error=0.;const auto a=lv[2].get_elements(),b=dv[2].get_elements();
        for(int k=0;k<a.size();++k)coefficient_error=std::max(coefficient_error,std::abs(a[k]-b[k])/(1.+std::abs(a[k])));
        std::vector<int> columns{0,n/2,n-1,vs,vs+3*m["m"].get<int>(),nv-1};if(vs>n)columns.push_back(n);
        for(int col:columns){auto xa=x,xb=x;const double h=1e-6*(1.+std::abs(x[col]));xa[col]+=h;xb[col]-=h;
            const double fd=(static_cast<double>(fg(std::vector<DM>{DM(xa),parameters})[0])-static_cast<double>(fg(std::vector<DM>{DM(xb),parameters})[0]))/(2.*h);
            derivative_error=std::max(derivative_error,std::abs(fd-gradient[col])/std::max({1.,std::abs(fd),std::abs(gradient[col])}));}
        const double objective_error=std::abs(lf-static_cast<double>(actual[0]))/(1.+std::abs(lf));
        const bool passed=lift_coefficient_error<1e-8 && lift_residual_error<1e-8 && lift_link_error<1e-12 && hermite_objective_error<1e-8 && hermite_gradient_error<1e-6 && hermite_residual_error<1e-8 && point_lift_error<1e-7 && auxiliary_elimination_gradient_error<1e-7 && metric_error<1e-8 && tangent_error<1e-4 && psd && lift_objective_error<1e-10 && lift_gradient_error<1e-7 && objective_error<1e-10 && coefficient_error<1e-10 && derivative_error<1e-3 && direct.expressions[0].numel()==nv && direct.residuals.size1_out(0)==n*17*6;
        const Json report{{"passed",passed},{"joint_lift_elimination_error",joint_lift_error},{"hermite_objective_error",hermite_objective_error},{"hermite_gradient_error",hermite_gradient_error},{"hermite_residual_error",hermite_residual_error},{"point_lift_elimination_error",point_lift_error},{"auxiliary_elimination_gradient_error",auxiliary_elimination_gradient_error},{"auxiliary_formulation",form},{"actual_penalty",static_cast<double>(dv[5])},{"quadrature_intervals",16},{"direct_variables",nv},{"lifted_variables",xl.size()},
            {"lift_coefficient_error",lift_coefficient_error},{"lift_residual_error",lift_residual_error},{"lift_link_error",lift_link_error},{"metric_projection_error",metric_error},{"minco_tangent_relative_error",tangent_error},{"coefficient_lift_gradient_error",lift_gradient_error},{"coefficient_lift_objective_error",lift_objective_error},{"gauss_newton_psd",psd},{"gn_minimum_eigenvalue",minimum_eigenvalue},{"gn_maximum_eigenvalue",maximum_eigenvalue},{"objective_relative_error",objective_error},{"coefficient_relative_error",coefficient_error},{"gradient_relative_error",derivative_error}};
        std::ofstream(argv[3])<<report.dump(2)<<'\n';std::cout<<report.dump(2)<<'\n';return passed?0:1;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
