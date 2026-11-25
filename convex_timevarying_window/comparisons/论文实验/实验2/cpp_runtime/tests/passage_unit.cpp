#include "numpy_passage.hpp"
#include "sip_exchange_policy.hpp"
#include <iostream>
#include <stdexcept>

void require(bool condition,const char* message) {
    if(!condition)throw std::runtime_error(message);
}

int main() {
    try {
        Window window;
        window.center=Vector::Zero();
        window.rotation=Eigen::Matrix3d::Identity();
        window.circle=true;
        window.swing=false;
        window.opening_radius=.75;
        window.frequency=1.;
        window.amplitude=.2;
        window.length=1.6;
        window.phase=0.;
        const double duration=.1,radius=.385;
        Curve following=Curve::Zero();
        following(0,1)=.4499;
        following(0,2)=.001;
        // A seventh-order expansion follows the moving circle with a small,
        // positive safety margin. Separate motion/curve bounds need many splits.
        const double angle=2.*pi*window.frequency*duration;
        following(1,1)=-window.amplitude*angle;
        following(3,1)=window.amplitude*std::pow(angle,3)/6.;
        following(5,1)=-window.amplitude*std::pow(angle,5)/120.;
        following(7,1)=window.amplitude*std::pow(angle,7)/5040.;
        auto limited=numpy_passage(following,duration,0.,window,radius,false,512);
        require(!limited.at("passed") && limited.at("outcome")=="budget_unresolved",
            "regression curve must exhaust the old 512 subdivision cap");
        require(limited.at("witness_u").is_null(),"an unresolved box has no violating witness");
        require(!limited.at("minimum_sample_u").is_null(),"keep the diagnostic minimum sample");
        require(!sip_exchange::geometry_witness_violates(limited.at("outcome"),
            limited.at("minimum_sample_distance_m"),radius),"unresolved samples must not enter the NLP");
        auto complete=numpy_passage(following,duration,0.,window,radius);
        require(complete.at("passed") && complete.at("boxes").get<int>()>512,
            "continue checking the same safe curve beyond the old cap");
        require(complete.at("distance_lower_bound_m").get<double>()>=radius,
            "a pass requires a complete lower bound above the unchanged threshold");
        auto immediate=numpy_passage(following,duration,0.,window,radius,false,0);
        require(immediate.at("outcome")=="budget_unresolved" && immediate.at("witness_u").is_null(),
            "zero work budget must remain unresolved");
        window.amplitude=0.;
        Curve touching=Curve::Zero();
        touching(0,1)=.75+.085-(radius-.0000774);
        auto tiny=numpy_passage(touching,duration,0.,window,radius);
        require(tiny.at("outcome")=="violated" && !tiny.at("witness_u").is_null(),
            "a 77 micrometre violation must not be hidden by the old 0.2 mm dead band");
        require(sip_exchange::geometry_witness_violates(tiny.at("outcome"),
            tiny.at("minimum_sample_distance_m"),radius),"a measured violation must be eligible for exchange");
        for(const std::string outcome:{"budget_unresolved","unresolved","repeat_aperture_crossing"})
            require(!sip_exchange::geometry_witness_violates(outcome,.1,radius),
                "only a measured distance violation may generate a safety witness");
        require(!sip_exchange::geometry_witness_violates("violated",radius,radius),
            "a point on the verification boundary is not a violated sample");
        Curve far=Curve::Zero();far(0,2)=2.;
        require(numpy_passage(far,duration,0.,window,radius,false,0).at("passed"),
            "a curve separated from the plane requires no subdivision");
        bool rejected=false;
        try {numpy_passage(far,duration,0.,window,radius,false,-1);}catch(const std::invalid_argument&){rejected=true;}
        require(rejected,"invalid budgets must be rejected");
        std::cout<<"Passage budget continuation, measured violations and witness admission passed; safe splits="
            <<complete.at("boxes")<<'\n';
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
