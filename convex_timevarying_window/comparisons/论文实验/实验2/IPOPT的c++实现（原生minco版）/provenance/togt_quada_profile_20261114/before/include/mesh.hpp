#pragma once
#include "trajectory.hpp"
#include <chrono>

inline Vector closest_triangle(const Vector& p,const std::array<Vector,3>& triangle) {
    const Vector a=triangle[0],b=triangle[1],c=triangle[2],ab=b-a,ac=c-a,ap=p-a;
    const double d1=ab.dot(ap),d2=ac.dot(ap);
    if(d1<=0. && d2<=0.)return a;
    const Vector bp=p-b;const double d3=ab.dot(bp),d4=ac.dot(bp);
    if(d3>=0. && d4<=d3)return b;
    const double vc=d1*d4-d3*d2;
    if(vc<=0. && d1>=0. && d3<=0.)return a+d1/(d1-d3)*ab;
    const Vector cp=p-c;const double d5=ab.dot(cp),d6=ac.dot(cp);
    if(d6>=0. && d5<=d6)return c;
    const double vb=d5*d2-d1*d6;
    if(vb<=0. && d2>=0. && d6<=0.)return a+d2/(d2-d6)*ac;
    const double va=d3*d6-d5*d4;
    if(va<=0. && d4-d3>=0. && d5-d6>=0.)return b+(d4-d3)/((d4-d3)+(d5-d6))*(c-b);
    const double inverse=1./(va+vb+vc);return a+(vb*ab+vc*ac)*inverse;
}

struct Mesh {
    std::vector<std::array<Vector,3>> triangles;
    Vector low=Vector::Constant(INFINITY),high=Vector::Constant(-INFINITY);
    explicit Mesh(const Json& rows) {
        for(const auto& row:rows) {
            std::array<Vector,3> triangle;
            for(int k=0;k<3;++k) {
                triangle[k]=Vector(row[k][0],row[k][1],row[k][2]);
                low=low.cwiseMin(triangle[k]);high=high.cwiseMax(triangle[k]);
            }
            triangles.push_back(triangle);
        }
    }
    double distance(const Vector& point)const {
        double squared=INFINITY;
        for(const auto& triangle:triangles)squared=std::min(squared,(point-closest_triangle(point,triangle)).squaredNorm());
        return std::sqrt(squared);
    }
};

inline Json mesh_piece(const Curve& coefficients,double duration,double start,const Window& window,const Mesh& mesh,
    const std::chrono::steady_clock::time_point& deadline) {
    constexpr double radius=.3859447197013662,guard=1e-8;
    Curve local=coefficients*window.rotation;local.row(0)-=window.center.transpose()*window.rotation;
    struct Interval {double a,b;Curve controls;};
    std::vector<Interval> stack{{0.,1.,bernstein(local)}};int boxes=0;double minimum=INFINITY,lower_bound=INFINITY;
    Json witness=nullptr;
    while(!stack.empty()) {
        if(boxes>=32768 || std::chrono::steady_clock::now()>deadline)
            return Json{{"outcome","unresolved"},{"reason","check_budget"},{"boxes",boxes},{"witness",witness}};
        const auto item=stack.back();stack.pop_back();++boxes;
        auto motion=window.bounds(start+item.a*duration,start+item.b*duration);
        Vector low=item.controls.colwise().minCoeff().transpose()-motion.second-Vector::Constant(guard);
        Vector high=item.controls.colwise().maxCoeff().transpose()-motion.first+Vector::Constant(guard);
        const Vector gap=(mesh.low-high).cwiseMax(low-mesh.high).cwiseMax(Vector::Zero());
        double bound=gap.norm();
        if(bound>radius+guard){lower_bound=std::min(lower_bound,bound-radius);continue;}
        const Vector center=(low+high)/2.,half=(high-low)/2.;
        bound=std::max(bound,mesh.distance(center)-half.norm());
        if(bound>radius+guard){lower_bound=std::min(lower_bound,bound-radius);continue;}
        for(double u:{item.a,(item.a+item.b)/2.,item.b}) {
            const double at=start+u*duration;const Vector q=evaluate(local,u)-window.motion(at);
            const double clearance=mesh.distance(q)-radius;
            if(clearance<minimum) {
                minimum=clearance;witness=Json{{"time_s",at},{"u",u},{"local_position",{q.x(),q.y(),q.z()}},{"clearance_m",clearance}};
            }
            if(clearance < -guard)return Json{{"outcome","collision"},{"boxes",boxes},{"minimum_sample_clearance_m",minimum},{"witness",witness}};
        }
        if((item.b-item.a)*duration<=1e-7)
            return Json{{"outcome","unresolved"},{"reason","near_tangency"},{"boxes",boxes},{"witness",witness}};
        const auto halves=split(item.controls,.5);const double middle=(item.a+item.b)/2.;
        stack.push_back({middle,item.b,halves.second});stack.push_back({item.a,middle,halves.first});
    }
    return Json{{"outcome","clear"},{"boxes",boxes},{"clearance_lower_bound_m",std::isfinite(lower_bound)?Json(lower_bound):Json(nullptr)},
        {"minimum_sample_clearance_m",std::isfinite(minimum)?Json(minimum):Json(nullptr)},{"witness",witness}};
}

inline Json mesh_evaluate(const Trajectory& trajectory,const Json& scene) {
    const auto began=std::chrono::steady_clock::now();const auto deadline=began+std::chrono::seconds(120);
    Json checks=Json::array(),events=Json::array(),ground=Json::array();bool collision=false,unresolved=false,ordered=true;
    double previous=-INFINITY;
    for(const auto& row:scene["windows"]) {
        const auto window=load_window(row);Mesh mesh(row["mesh"]);double start=0.;Json crossings=Json::array();bool degenerate=false;
        for(size_t piece=0;piece<trajectory.durations.size();++piece) {
            Json check=mesh_piece(trajectory.coefficients[piece],trajectory.durations[piece],start,window,mesh,deadline);
            check["gate"]=row["name"];check["piece"]=piece;checks.push_back(check);
            collision=collision || check["outcome"]=="collision";unresolved=unresolved || check["outcome"]=="unresolved";
            Curve local=trajectory.coefficients[piece]*window.rotation;local.row(0)-=window.center.transpose()*window.rotation;
            if(local.col(2).cwiseAbs().maxCoeff()<1e-10)degenerate=true;
            else for(double root:roots(local.col(2))) {
                if(root < -1e-8 || root > 1.+1e-8)continue;
                const double u=std::clamp(root,0.,1.),at=start+u*trajectory.durations[piece];
                const Vector q=evaluate(local,u)-window.motion(at);if(!window.contains(q.head<2>()))continue;
                bool duplicate=false;for(const auto& event:crossings)duplicate=duplicate || std::abs(event["time_s"].get<double>()-at)<1e-6;
                if(duplicate)continue;
                const double rate=trajectory.sample(at,1).dot(window.rotation.col(2));
                crossings.push_back(Json{{"time_s",at},{"forward",row["sign"].get<double>()*rate < -1e-8},
                    {"piece",piece},{"local_point",{q.x(),q.y()}}});
            }
            start+=trajectory.durations[piece];
        }
        std::sort(crossings.begin(),crossings.end(),[](const Json& a,const Json& b){return a["time_s"].get<double>()<b["time_s"].get<double>();});
        const bool single=crossings.size()==1 && crossings[0]["forward"].get<bool>() && !degenerate;
        if(!single)ordered=false;
        else {const double at=crossings[0]["time_s"];ordered=ordered && at>previous;previous=at;}
        events.push_back(Json{{"gate",row["name"]},{"events",crossings},{"degenerate_plane_piece",degenerate}});
    }
    double start=0.;
    for(size_t piece=0;piece<trajectory.durations.size();++piece) {
        Eigen::VectorXd derivative(7);for(int k=1;k<8;++k)derivative[k-1]=k*trajectory.coefficients[piece](k,2);
        std::vector<double> candidates{0.,1.};for(double root:roots(derivative))if(root>0. && root<1.)candidates.push_back(root);
        double minimum=INFINITY,witness=0.;for(double u:candidates) {
            const double value=evaluate(trajectory.coefficients[piece],u).z()-.3859447197013662;
            if(value<minimum){minimum=value;witness=u;}
        }
        const std::string outcome=minimum < -1e-8?"collision":minimum>1e-8?"clear":"unresolved";
        ground.push_back(Json{{"piece",piece},{"outcome",outcome},{"minimum_clearance_m",minimum},{"time_s",start+witness*trajectory.durations[piece]}});
        start+=trajectory.durations[piece];
    }
    const Vector start_position(scene["start"][0],scene["start"][1],scene["start"][2]);
    const Vector goal(scene["goal"][0],scene["goal"][1],scene["goal"][2]);
    double error=std::max((trajectory.sample(0.)-start_position).norm(),(trajectory.sample(trajectory.end())-goal).norm());
    for(int d=1;d<4;++d)error=std::max({error,trajectory.sample(0.,d).norm(),trajectory.sample(trajectory.end(),d).norm()});
    const bool endpoints=error<1e-5;
    const std::string verdict=collision?"nominal_collision":!(ordered && endpoints)?"nominal_task_invalid":unresolved?"nominal_unresolved":"nominal_safe_complete";
    return Json{{"verdict",verdict},{"passed",verdict=="nominal_safe_complete"},{"collision_detected",collision},
        {"checks",checks},{"ground",ground},{"opening_events",events},{"ordered_exactly_once",ordered},{"endpoints_passed",endpoints},
        {"endpoint_max_error",error},{"full_trajectory_time_s",trajectory.end()},
        {"check_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()},
        {"evidence","floating adaptive Bernstein/analytic-motion boxes and STL triangle distance"},{"formal_interval_certificate",false}};
}
