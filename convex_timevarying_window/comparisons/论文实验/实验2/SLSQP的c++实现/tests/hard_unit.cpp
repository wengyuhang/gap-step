#include "hard_problem.hpp"
int main(int argc,char** argv) {
    try {
        using namespace casadi;
        MX x=MX::sym("x",2),p=MX::sym("p",0);
        MX f=pow(x(0)-2.,2)+pow(x(1)-1.,2),c=MX::vertcat({x(0)+x(1)-2.,x(0),x(1)});
        Function values("test_values",{x,p},{f,c});
        Function derivatives("test_derivatives",{x,p},{MX::densify(MX::gradient(f,x)),MX::densify(MX::jacobian(c,x))});
        const auto result=native_slsqp::minimize(values,derivatives,DM::zeros(0,1),{.2,1.8},1);
        if(result.mode!=0 || std::abs(result.x[0]-1.5)>1e-6 || std::abs(result.x[1]-.5)>1e-6 || result.violation>1e-7)
            throw std::runtime_error("SLSQP analytical regression failed");
        if(argc!=4)throw std::runtime_error("Usage: hard_unit INPUT SCENE OUTPUT");
        hard_problem::Model model(argv[1],argv[2]);hard_problem::Witnesses witnesses;
        for(int gate=0;gate<7;++gate)witnesses[{gate,gate}]={"before",{0.}};
        const auto finite=model.build(witnesses);auto point=model.initial(0);
        const DM params(model.metadata["initials"][0]["parameters"].get<std::vector<double>>());
        const auto d=finite.derivatives(std::vector<DM>{DM(point),params});
        const auto gradient=d[0].get_elements(),jac=d[1].get_elements();
        double gradient_error=0.,jacobian_error=0.;const int nv=point.size(),m=finite.constraints;
        for(int group=0;group<3;++group) {
            auto plus=point,minus=point;std::vector<double> direction(nv);constexpr double h=1e-6;
            for(int k=0;k<nv;++k){direction[k]=std::sin(.3*k+.7*group);if((group==0 && k>=8)||(group==1 && (k<8 || k>=model.metadata["velocity_start"].get<int>()))||(group==2 && k<model.metadata["velocity_start"].get<int>()))direction[k]=0.;plus[k]+=h*direction[k];minus[k]-=h*direction[k];}
            const auto high=finite.values(std::vector<DM>{DM(plus),params}),low=finite.values(std::vector<DM>{DM(minus),params});
            double automatic=0.;for(int k=0;k<nv;++k)automatic+=gradient[k]*direction[k];
            const double numeric=(static_cast<double>(high[0])-static_cast<double>(low[0]))/(2*h);
            gradient_error=std::max(gradient_error,std::abs(automatic-numeric)/(1.+std::abs(automatic)));
            const auto hc=high[1].get_elements(),lc=low[1].get_elements();
            for(int row=0;row<m;++row){double j=0.;for(int k=0;k<nv;++k)j+=jac[k*m+row]*direction[k];const double fd=(hc[row]-lc[row])/(2*h);jacobian_error=std::max(jacobian_error,std::abs(j-fd)/(1.+std::abs(j)));}
        }
        const bool passed=gradient_error<1e-5 && jacobian_error<1e-4 && finite.dynamic_rows==816 && static_cast<int>(point.size())==model.metadata["nvar"].get<int>();
        Json report{{"passed",passed},{"solver_backend_passed",true},{"trajectory_variables",point.size()},
            {"dynamic_rows",finite.dynamic_rows},{"dynamic_nodes_per_piece",17},{"constraint_rows",m},
            {"gradient_error",gradient_error},{"jacobian_error",jacobian_error}};
        std::ofstream(argv[3])<<report.dump(2)<<'\n';std::cout<<report.dump(2)<<'\n';return passed?0:1;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 2;}
}
