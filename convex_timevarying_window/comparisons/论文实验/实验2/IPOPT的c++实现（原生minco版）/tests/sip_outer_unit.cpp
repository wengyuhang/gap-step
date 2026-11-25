#include "sip_exchange_policy.hpp"
#include <iostream>
#include <stdexcept>
using namespace sip_exchange;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
Snapshot state(double objective,const std::string& status,double marker) {
    return {{marker,2.},{marker+10.,20.},{marker+100.,200.},{{"old",marker+30.}},objective,status,0};
}
int main() {
    try {
        Incumbent incumbent;
        auto first=state(20.,"Solve_Succeeded",1.);
        require(incumbent.consider(first,0.),"first successful finite candidate must be retained");
        auto worse=state(22.,"Solve_Succeeded",2.);
        require(!incumbent.consider(worse,0.),"same-problem archive may retain lower objective");
        require(incumbent.value().variables[0]==1. && incumbent.value().constraint_duals.at("old")==31.,"incumbent primal and dual must survive");
        for(const auto& status:{"Maximum_Iterations_Exceeded","Solved_To_Acceptable_Level","Restoration_Failed","Infeasible_Problem_Detected"}) {
            auto failed=state(10.,status,99.);
            require(!incumbent.consider(failed,0.),"non-success must not replace best candidate");
            require(!dual_warm_start_allowed(status,incumbent.value().status),"failed previous solve forbids even successful incumbent multipliers");
            require(constraint_duals(incumbent.value(),{"old","new"},status)==std::vector<double>({0.,0.}),"rollback must zero constraint duals after failed previous solve");
            require(bound_duals(incumbent.value(),status)==std::vector<double>({0.,0.}),"rollback must zero bound duals after failed previous solve");
            require(constraint_duals(failed,{"old","new"})==std::vector<double>({0.,0.}),"failed multipliers must be zero");
            require(bound_duals(failed)==std::vector<double>({0.,0.}),"failed bound multipliers must be zero");
            require(accept_verified(failed,0.,true,true)==(std::string(status)=="Solved_To_Acceptable_Level"),"only successful IPOPT statuses may be accepted");
            require(!finite_ready(status,1e-3,true),"finite infeasibility forbids exchange regardless of status");
            require(finite_ready(status,1e-9,true)==(std::string(status)=="Solved_To_Acceptable_Level"),"iteration limit cannot enter exchange");
        }
        require(dual_warm_start_allowed("Solve_Succeeded",incumbent.value().status),"successful previous solve permits incumbent duals");
        require(constraint_duals(incumbent.value(),{"new","old"})==std::vector<double>({0.,31.}),"successful duals must remap by key and zero new rows");
        require(!incumbent.consider(state(10.,"Solve_Succeeded",3.),-1.1e-5),"geometrically infeasible solve must not replace incumbent");
        auto better=state(18.,"Solve_Succeeded",4.);
        require(incumbent.consider(better,-1e-5),"tolerance boundary is admissible");
        require(accept_verified(worse,0.,true,true),"verified higher-cost candidate must be accepted before fallback to unsafe incumbent");
        require(!accept_verified(better,0.,false,true),"finite incumbent alone is not final safety");
        require(!accept_verified(better,0.,true,false),"dynamic verification must still pass");
        require(incumbent.consider(worse,0.,1),"new finite constraint set may require higher objective");
        require(incumbent.value().variables[0]==2.,"successful progress on a new finite problem must be retained");
        require(finite_ready("Solve_Succeeded",1e-9,true),"complete finite feasibility allows exchange");
        require(!finite_ready("Solve_Succeeded",1e-3,true),"infeasible successful status must stop before exchange");
        require(!finite_ready("Solve_Succeeded",0.,false),"nonfinite curve cannot be verified");
        WorstWitnessPerGate worst;
        worst.consider({0,0,"before",.01,.9,.1});worst.consider({0,1,"after",.02,.1,.3});
        worst.consider({0,2,"normalized",.4,.4,.2});worst.consider({1,1,"before",.03,.8,.05});
        require(worst.values().size()==2 && worst.values().at(0).piece==1,"one worst witness per gate across all pieces");
        require(!new_witness({.02},.020000001) && new_witness({.02},.03),"duplicate worst point must be skipped before selection");
        require(default_finite_radius==.63094472,"current default safety radius must increase");
        std::cout<<"SIP finite feasibility gate, constraint revisions, status-gated duals and witness policy passed\n";
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
    return 0;
}
