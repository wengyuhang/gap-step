#pragma once
#include "geometry.hpp"

// Physical coordinates in the moving gate's two-dimensional frame, in metres.
// The solver uses q = diag(scale) * z solely for numerical units, not a shape map.
namespace planar_points {
struct Domain {
    Eigen::Vector2d scale, lower, upper;
    std::vector<Eigen::Vector2d> normals;
    std::vector<double> limits;
    double safe_radius = 0.;
};
inline Domain domain(const Window& w,double radius) {
    Domain out;
    if(w.circle) {
        out.safe_radius=w.opening_radius-radius;
        if(!(out.safe_radius>0.))throw std::runtime_error("Empty eroded circular opening");
        out.scale.setConstant(w.opening_radius);
        out.lower.setConstant(-out.safe_radius);out.upper.setConstant(out.safe_radius);
        return out;
    }
    if(w.physical.size()!=4)throw std::runtime_error("Main experiment requires four-sided or circular gates");
    double area=0.;
    for(size_t i=0;i<4;++i) {
        const auto& p=w.physical[i];const auto& q=w.physical[(i+1)%4];
        area+=p.x()*q.y()-p.y()*q.x();
    }
    if(std::abs(area)<1e-12)throw std::runtime_error("Degenerate quadrilateral");
    out.scale.setZero();
    for(const auto& vertex:w.physical)out.scale=out.scale.cwiseMax(vertex.cwiseAbs());
    if(out.scale.minCoeff()<=0.)throw std::runtime_error("Degenerate point scaling");
    for(size_t i=0;i<4;++i) {
        const auto edge=(w.physical[(i+1)%4]-w.physical[i]).eval();
        if(edge.norm()<1e-12)throw std::runtime_error("Zero-length quadrilateral edge");
        Eigen::Vector2d normal(edge.y(),-edge.x());normal*= (area>0.?1.:-1.)/edge.norm();
        const double limit=normal.dot(w.physical[i])-radius;
        if(limit<=0.)throw std::runtime_error("Center outside eroded opening");
        out.normals.push_back(normal);out.limits.push_back(limit);
    }
    out.lower.setConstant(INFINITY);out.upper.setConstant(-INFINITY);
    for(size_t i=0;i<4;++i) {
        const size_t j=(i+3)%4;
        Eigen::Matrix2d pair;pair.row(0)=out.normals[j];pair.row(1)=out.normals[i];
        if(std::abs(pair.determinant())<1e-12)throw std::runtime_error("Parallel adjacent quadrilateral edges");
        const Eigen::Vector2d vertex=pair.inverse()*Eigen::Vector2d(out.limits[j],out.limits[i]);
        for(size_t k=0;k<4;++k)if(out.normals[k].dot(vertex)>out.limits[k]+1e-10)
            throw std::runtime_error("Nonconvex or empty eroded quadrilateral");
        out.lower=out.lower.cwiseMin(vertex);out.upper=out.upper.cwiseMax(vertex);
    }
    return out;
}
// finite_radius is a distance to the frame tube CENTERLINE. The point domain
// starts from the physical aperture boundary, so the tube radius is counted once.
inline double frame_tube_radius(const Json& metadata) {
    const double tube=metadata.at("config").at("frame_thickness");
    if(!std::isfinite(tube) || std::abs(tube-.085)>1e-12)
        throw std::runtime_error("Point-domain frame tube radius must match the geometry model");
    return tube;
}
inline double erosion_radius(const Json& metadata) {
    const double radius=metadata.at("finite_radius").get<double>()-frame_tube_radius(metadata);
    if(!std::isfinite(radius) || radius<=0.)throw std::runtime_error("Invalid finite safety sphere radius");
    return radius;
}
inline void validate(const Json& metadata) {
    const double radius=erosion_radius(metadata);
    if(!metadata.contains("point_erosion_radius_m") ||
       std::abs(metadata.at("point_erosion_radius_m").get<double>()-radius)>1e-12)
        throw std::runtime_error("Stale explicit-xy input: regenerate using main_course_inputs; point erosion must use the safety sphere, not centerline clearance");
    const int m=metadata.at("m");
    for(int gate=0;gate<m;++gate) {
        const auto box=domain(load_window(metadata.at("windows").at(gate)),radius);
        for(int axis=0;axis<2;++axis)if(
            std::abs(metadata.at("point_scale_m").at(gate).at(axis).get<double>()-box.scale[axis])>1e-12 ||
            std::abs(metadata.at("point_lower_m").at(gate).at(axis).get<double>()-box.lower[axis])>1e-12 ||
            std::abs(metadata.at("point_upper_m").at(gate).at(axis).get<double>()-box.upper[axis])>1e-12)
            throw std::runtime_error("Stale explicit-xy bounds: regenerate inputs after changing the safety sphere");
    }
}
inline void configure(Json& metadata) {
    if(metadata.value("point_coordinates",std::string())=="scaled_xy"){validate(metadata);return;}
    const int n=metadata.at("n"),m=metadata.at("m"),old_vs=metadata.at("velocity_start");
    const bool free=metadata.at("free_points");
    const int old_nv=metadata.at("nvar"),order=metadata.value("minco_derivative_order",3);
    const int new_vs=n+(free?2*m:0),new_nv=new_vs+3*order*m;
    int old_gauges=0;
    if(free && metadata.value("point_coordinates",std::string())!="direct_polar_polygon")
        for(const auto& slice:metadata.at("slices"))if(!slice.at("circle").get<bool>())++old_gauges;
    const int workspace=metadata.at("workspace_rows");
    const int prefix=workspace+m+(metadata.value("restore_historical_domain",false)?n:0);
    const int old_domain=metadata.value("point_domain_rows",0);
    int count=0;Json slices=Json::array(),scales=Json::array(),low=Json::array(),high=Json::array();
    for(int i=0;i<m;++i) {
        const Window w=load_window(metadata.at("windows").at(i));const auto box=domain(w,erosion_radius(metadata));
        scales.push_back({box.scale.x(),box.scale.y()});
        low.push_back({box.lower.x(),box.lower.y()});high.push_back({box.upper.x(),box.upper.y()});
        if(free){slices.push_back(Json{{"begin",n+2*i},{"end",n+2*i+2},{"circle",w.circle}});count+=w.circle?1:4;}
    }
    for(const auto& key:{"lower_g","upper_g"}) {
        const auto old=metadata.at(key);Json bounds=Json::array();
        for(int i=0;i<prefix;++i)bounds.push_back(old.at(i));
        for(int i=0;i<count;++i)bounds.push_back(std::string(key)=="lower_g"?Json(0.):Json(nullptr));
        for(size_t i=prefix+old_gauges+old_domain;i<old.size();++i)bounds.push_back(old.at(i));
        metadata[key]=bounds;
    }
    for(auto& seed:metadata["initials"]) {
        const auto old=seed.at("original").get<std::vector<double>>();
        if(static_cast<int>(old.size())!=old_nv)throw std::runtime_error("Archived initial dimension mismatch");
        std::vector<double> physical(new_nv,0.);
        std::copy(old.begin(),old.begin()+n,physical.begin());
        std::copy(old.begin()+old_vs,old.begin()+old_vs+3*order*m,physical.begin()+new_vs);
        std::vector<double> finite(new_nv+metadata.at("slack_scale").size(),0.);
        seed["original"]=physical;seed["finite"]=finite;
    }
    metadata["point_erosion_radius_m"]=erosion_radius(metadata);
    metadata["finite_safety_sphere_radius_m"]=erosion_radius(metadata);
    metadata["verification_safety_sphere_radius_m"]=metadata.at("verification_radius").get<double>()-frame_tube_radius(metadata);
    metadata["frame_tube_radius_m"]=frame_tube_radius(metadata);
    metadata["body_sphere_radius_m"]=metadata.at("config").at("radius");
    metadata["point_radius_reference"]="physical aperture boundary; centerline threshold minus frame tube radius";
    metadata["point_coordinates"]="scaled_xy";metadata["point_initialization"]="center";
    metadata["point_scale_m"]=scales;metadata["point_lower_m"]=low;metadata["point_upper_m"]=high;
    metadata["point_domain_rows"]=count;metadata["point_gauge_rows"]=0;
    metadata["slices"]=slices;metadata["velocity_start"]=new_vs;metadata["nvar"]=new_nv;
}
} // namespace planar_points
