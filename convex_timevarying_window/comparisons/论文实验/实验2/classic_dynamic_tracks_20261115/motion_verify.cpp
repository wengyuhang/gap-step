#include "restored_kernels.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
namespace fs=std::filesystem;
Json read(const fs::path& p){std::ifstream f(p);if(!f)throw std::runtime_error("Missing "+p.string());Json j;f>>j;return j;}
int main(int argc,char**argv) {
 try {
    if(argc!=3)throw std::runtime_error("Usage: motion_verify INPUTS OUTPUT");
    const fs::path inputs=argv[1];Json rows=Json::array();int checked=0;
    const auto protocol=read(inputs/"PROTOCOL.json");
    for(const auto& level:protocol["levels"]) {
        const auto scene=read(inputs/"assets"/level.get<std::string>()/"scene.json");
        double maximum_velocity_error=0.,maximum_frame_error=0.;
        for(const auto& row:scene["windows"]) {
            const auto w=load_window(row);const auto node=restored_kernels::frame_node(w,"u",.3851);
            const double period=w.frequency>0.?1./w.frequency:1.;
            for(int k=0;k<40;++k) {
                const double a=(-.7+k*.047)*period,b=a+(.03+.01*(k%9))*period;
                const auto bound=w.bounds(a,b);
                for(int sample=0;sample<=30;++sample) {
                    const Vector point=w.motion(a+(b-a)*sample/30.);
                    if((point.array()<bound.first.array()-1e-11).any() ||
                       (point.array()>bound.second.array()+1e-11).any())
                        throw std::runtime_error("Motion box misses known motion");
                }
                const double t=.37+k*.091,step=1e-6;
                const Vector numeric=(w.motion(t+step)-w.motion(t-step))/(2*step);
                const double velocity_error=(numeric-w.motion_velocity(t)).norm()/(1.+numeric.norm());
                maximum_velocity_error=std::max(maximum_velocity_error,velocity_error);
                std::vector<double> state(26,0.);state[24]=1.7;state[25]=t-.37*1.7;
                const Vector local(.13+.01*k,-.11,.09),world=w.center+w.rotation*(local+w.motion(t));
                for(int axis=0;axis<3;++axis)state[axis*8]=world[axis];
                const auto value=node(std::vector<casadi::DM>{casadi::DM(state),casadi::DM(.37)})[0].get_elements();
                const double numeric_frame=w.distance_squared(local)-.3851*.3851;
                maximum_frame_error=std::max(maximum_frame_error,std::abs(*std::min_element(value.begin(),value.end())-numeric_frame));
            }
            ++checked;
        }
        if(maximum_velocity_error>1e-7 || maximum_frame_error>1e-8)throw std::runtime_error("Motion velocity/frame parity failed");
        rows.push_back(Json{{"level",level},{"passed",true},{"maximum_velocity_relative_error",maximum_velocity_error},
                           {"maximum_symbolic_frame_error",maximum_frame_error}});
    }
    std::ofstream(argv[2])<<Json{{"passed",true},{"windows_checked",checked},{"rows",rows}}.dump(2)<<'\n';
    std::cout<<"Verified rotated motion, interval bounds, velocity and frame expressions for "<<checked<<" windows\n";
 }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
