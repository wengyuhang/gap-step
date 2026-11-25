#pragma once
#include <casadi/casadi.hpp>
#include "geometry.hpp"
#include <fstream>
// Read-only diagnostic. Always returns zero: never changes acceptance, steps,
// stopping criteria, starting points, or the finite problem.
class IpoptIterationTrace final : public casadi::Callback {
    int nx_,ng_,np_;mutable size_t count_=0;mutable std::ofstream stream_;
public:
    IpoptIterationTrace(int nx,int ng,int np,const std::string& path):nx_(nx),ng_(ng),np_(np),stream_(path,std::ios::app){
        if(!stream_)throw std::runtime_error("Cannot open iteration trace");
        construct("readonly_ipopt_iteration_trace");
    }
    casadi_int get_n_in() override{return casadi::nlpsol_n_out();}
    casadi_int get_n_out() override{return 1;}
    std::string get_name_in(casadi_int i) override{return casadi::nlpsol_out(i);}
    std::string get_name_out(casadi_int) override{return "ret";}
    casadi::Sparsity get_sparsity_in(casadi_int i) override {
        auto name=casadi::nlpsol_out(i);int size=name=="x"||name=="lam_x"?nx_:name=="g"||name=="lam_g"?ng_:name=="lam_p"?np_:1;
        return casadi::Sparsity::dense(size,1);
    }
    casadi::Sparsity get_sparsity_out(casadi_int) override{return casadi::Sparsity::scalar();}
    std::vector<casadi::DM> eval(const std::vector<casadi::DM>& args) const override {
        const size_t index=count_++;
        if(index<80 || index%100==0) {
            Json row{{"callback_index",index},{"diagnostic_only",true}};
            for(size_t i=0;i<args.size();++i){const auto name=casadi::nlpsol_out(i);if(name!="lam_p")row[name]=args[i].get_elements();}
            stream_<<row.dump()<<std::endl;
        }
        return {casadi::DM(0.)};
    }
};
