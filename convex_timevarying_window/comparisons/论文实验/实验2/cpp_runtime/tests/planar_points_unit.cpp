#include "native_minco_pvaj.hpp"
#include "restored_kernels.hpp"
#include "mesh.hpp"
#include <filesystem>
using casadi::DM;using casadi::MX;using casadi::Function;
void require(bool x,const std::string& reason){if(!x)throw std::runtime_error(reason);}
Json read(const std::string& p){Json j;std::ifstream(p)>>j;return j;}
std::vector<double> initial(const Json& meta) {
    auto x=meta["initials"][0]["original"].get<std::vector<double>>();const int n=meta["n"],vs=meta["velocity_start"],nv=meta["nvar"];
    for(int k=0;k<n;++k){const double t=std::exp(x[k]);x[k]=t>1.?std::sqrt(2*t-1)-1:1-std::sqrt(2/t-1);}
    if(meta["free_points"].get<bool>())for(int k=n;k<vs;++k)x[k]/=meta["point_scale_m"][(k-n)/2][(k-n)%2].get<double>();
    for(int k=vs;k<nv;++k)x[k]/=meta["state_scale"][k-vs].get<double>();
    x.resize(nv+meta["slack_scale"].size(),0.);return x;
}
int main(int argc,char**argv) {
 try {
    if(argc!=5)throw std::runtime_error("Usage: planar_points_unit FREE_INPUT CENTER_INPUT SCENE OUTPUT");
    auto free=read(std::string(argv[1])+"/manifest.json"),center=read(std::string(argv[2])+"/manifest.json"),scene=read(argv[3]);
    const auto dynamic_meta=read(std::string(argv[1])+"/exchange_dynamics.json");
    const auto dynamics=restored_kernels::dynamic_node(dynamic_meta);
    free["dynamic_residual_scale"]=dynamic_meta["residual_scale"];center["dynamic_residual_scale"]=dynamic_meta["residual_scale"];
    require(free["nvar"]==85 && center["nvar"]==71,"point dimensions must be 14 or zero");
    require(free["point_domain_rows"]==19 && center["point_domain_rows"]==0,"four halfspaces per quad plus one disk row per circle");
    require(free["point_gauge_rows"]==0 && center["point_gauge_rows"]==0,"gauge must be absent");
    std::vector<Window> windows;int circles=0,quads=0;
    for(auto& row:scene["windows"]) {
        const Window w=load_window(row);windows.push_back(w);circles+=w.circle;quads+=!w.circle;
        const auto domain=planar_points::domain(w,planar_points::erosion_radius(free));
        const Eigen::Vector2d boundary(w.circle?domain.safe_radius:domain.upper.x(),0.);
        require(std::abs(w.distance_squared(Vector(boundary.x(),boundary.y(),0.))-
            std::pow(free["finite_radius"].get<double>(),2))<1e-10,
            "physical aperture erosion and frame centerline safety must encode the SAME sphere");
        require(domain.scale.minCoeff()>0. && domain.lower.maxCoeff()<0. && domain.upper.minCoeff()>0.,"center must be inside eroded opening");
        Mesh mesh(row["mesh"]);require(mesh.distance(Vector::Zero())>.74,"physical mesh aperture does not match main gate size");
        require(mesh.distance(Vector(w.circle?w.opening_radius+.085:w.frame[0].x(),0.,0.))<.09,"frame physical mesh missing");
    }
    require(circles==3 && quads==4,"main shape inventory must be 4+3");
    native_minco::Graph gf(free,scene,windows,dynamics,1000.),gc(center,scene,windows,dynamics,1000.);
    auto xf=initial(free),xc=initial(center);const auto params=free["initials"][0]["parameters"].get<std::vector<double>>();
    auto a=gf.trajectory(std::vector<DM>{DM(xf),DM(params)}),b=gc.trajectory(std::vector<DM>{DM(xc),DM(params)});
    double coefficient_error=0.;auto ca=a[2].get_elements(),cb=b[2].get_elements();
    for(size_t i=0;i<ca.size();++i)coefficient_error=std::max(coefficient_error,std::abs(ca[i]-cb[i]));
    require(coefficient_error<1e-9,"free center seed must preserve center trajectory");
    const int prefix=free["workspace_rows"].get<int>()+7+8;
    Function domains("main_domains",{gf.expressions[0],gf.expressions[1]},
        {gf.expressions[6](casadi::Slice(prefix,prefix+19))});
    auto domain_values=domains(std::vector<DM>{DM(xf),DM(params)})[0].get_elements();
    require(*std::min_element(domain_values.begin(),domain_values.end())>0.,"center point domains must be strictly feasible");
    auto diagonal=xf;const double safe=.762-planar_points::erosion_radius(free);
    diagonal[10]=safe/.762;diagonal[11]=safe/.762;
    auto dv=domains(std::vector<DM>{DM(diagonal),DM(params)})[0].get_elements();
    require(dv[4]<0.,"disk constraint must reject bounding-box corner");
    // The Jacobian of world crossing points with respect to plane coordinates has rank two
    // even at the circle center: there is no polar or radial-map singularity.
    const MX z=gf.expressions[0],p=gf.expressions[1];const auto output=gf.trajectory(std::vector<MX>{z,p});
    Function point_jac("main_point_jacobian",{z,p},{MX::jacobian(output[3],z)});
    const auto jac=point_jac(std::vector<DM>{DM(xf),DM(params)})[0];
    for(int gate=0;gate<7;++gate)for(int axis=0;axis<2;++axis) {
        double squared=0.;for(int world=0;world<3;++world)squared+=std::pow(static_cast<double>(jac(3*gate+world,8+2*gate+axis)),2);
        const double scale=free["point_scale_m"][gate][axis];require(std::abs(squared-scale*scale)<1e-10,"xy crossing point Jacobian must retain full rank");
    }
    Function jacobian("main_coefficient_jacobian",{z,p},{MX::jacobian(output[2],z)});
    const auto analytic=jacobian(std::vector<DM>{DM(xf),DM(params)})[0];
    double gradient_error=0.;const int nv=free["nvar"];
    for(int k=0;k<nv;++k) {
        auto plus=xf,minus=xf;const double h=1e-5*(1.+std::abs(xf[k]));plus[k]+=h;minus[k]-=h;
        const auto up=gf.trajectory(std::vector<DM>{DM(plus),DM(params)})[2].get_elements();
        const auto down=gf.trajectory(std::vector<DM>{DM(minus),DM(params)})[2].get_elements();
        for(size_t row=0;row<up.size();++row){double exact=static_cast<double>(analytic(row,k)),numeric=(up[row]-down[row])/(2*h);gradient_error=std::max(gradient_error,std::abs(exact-numeric)/(1.+std::abs(exact)));}
    }
    require(gradient_error<2e-4,"MINCO/xy/time coefficient Jacobian finite-difference failure");
    auto stale=free;stale["point_erosion_radius_m"]=free["finite_radius"];bool rejected=false;
    try{planar_points::validate(stale);}catch(const std::runtime_error&){rejected=true;}
    require(rejected,"centerline-radius-as-erosion stale inputs must be rejected");
    Json result{{"passed",true},{"point_erosion_radius_m",planar_points::erosion_radius(free)},
        {"aperture_and_centerline_sphere_equivalent",true},{"stale_radius_input_rejected",true},{"quad_count",quads},{"circle_count",circles},{"free_variables",85},{"center_variables",71},{"domain_rows",19},
        {"center_embedding_coefficient_error",coefficient_error},{"coefficient_jacobian_relative_error",gradient_error},{"circle_origin_point_jacobian_full_rank",true}};
    std::ofstream(argv[4])<<result.dump(2)<<'\n';std::cout<<result.dump()<<'\n';
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}return 0;
}
