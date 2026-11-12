#include "minimum_thrust.hpp"

void require(bool condition,const std::string& message) {
    if(!condition)throw std::runtime_error(message);
}
int main() {
    try {
        constexpr double mass=2.0643076923076924,gravity=9.8066;
        Curve hover=Curve::Zero();
        auto check=minimum_thrust::check(hover,2.,mass,gravity);
        require(check["passed"] && std::abs(check["minimum_total_thrust_n"].get<double>()-mass*gravity)<1e-10,"Hover thrust");
        Curve fall=Curve::Zero();fall(2,2)=-gravity/2.;
        check=minimum_thrust::check(fall,1.,mass,gravity);
        require(!check["passed"].get<bool>() && check["minimum_total_thrust_n"].get<double>()<1e-12,"Free-fall rejection");
        Curve dip=Curve::Zero();dip(2,0)=-.3;dip(3,0)=1./3.;dip(2,2)=-gravity/2.;
        check=minimum_thrust::check(dip,1.,mass,gravity);
        require(!check["passed"].get<bool>() && std::abs(check["witness_u"].get<double>()-.3)<1e-8,"Interior thrust minimum");
        require(check["minimum_total_thrust_n"].get<double>()<1e-8,"Interior minimum evaluation");
        for(int trial=0;trial<30;++trial) {
            Curve curve;
            for(int k=0;k<8;++k)for(int axis=0;axis<3;++axis)curve(k,axis)=std::sin(1.23*trial+3.1*k+.7*axis);
            const double duration=.7+.07*trial;
            check=minimum_thrust::check(curve,duration,mass,gravity);
            double dense=INFINITY;
            for(int node=0;node<=10000;++node) {
                const double u=node/10000.;Vector a=Vector::Zero();
                for(int k=7;k>=2;--k)a=a*u+double(k*(k-1))*curve.row(k).transpose();
                a/=duration*duration;a[2]+=gravity;dense=std::min(dense,mass*a.norm());
            }
            require(check["minimum_total_thrust_n"].get<double>()<=dense+1e-6,"Stationary minimum versus dense grid");
        }
        std::cout<<"Minimum-thrust hover, free fall, interior root and 30 dense comparisons passed\n";
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}
