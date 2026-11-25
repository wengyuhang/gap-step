#include "planar_points.hpp"
#include <fstream>
#include <iostream>
Json read(const std::string&p){Json j;std::ifstream(p)>>j;return j;}
int main(int argc,char**argv){try{
 if(argc!=5)throw std::runtime_error("RESULT SCENE EROSION OUTPUT");auto r=read(argv[1]),s=read(argv[2]);const double erosion=std::stod(argv[3]);
 const auto times=r.at("durations").get<std::vector<double>>();double at=0.,length=0.,mean_offset=0.,max_offset=0.;int boundary=0;Json gates=Json::array();
 for(int piece=0;piece<times.size();++piece){Curve c;for(int k=0;k<8;++k)for(int axis=0;axis<3;++axis)c(k,axis)=r["coefficients"][piece][k][axis];
  auto value=[&](double u,int order){Vector out=Vector::Zero();for(int k=7;k>=order;--k){double factor=1.;for(int j=0;j<order;++j)factor*=k-j;out=out*u+factor*c.row(k).transpose();}return (out/std::pow(times[piece],order)).eval();};
  const int steps=std::max(1000,int(std::ceil(times[piece]/.001)));double part=0.;for(int i=0;i<=steps;++i)part+=(i==0||i==steps?.5:1.)*value(double(i)/steps,1).norm();length+=part*times[piece]/steps;at+=times[piece];
  if(piece<s["windows"].size()){auto w=load_window(s["windows"][piece]);Vector q=w.rotation.transpose()*(value(1.,0)-w.center)-w.motion(at);auto domain=planar_points::domain(w,erosion);double use=0.;if(w.circle)use=q.head<2>().norm()/domain.safe_radius;else for(int edge=0;edge<domain.normals.size();++edge)use=std::max(use,domain.normals[edge].dot(q.head<2>())/domain.limits[edge]);
   auto velocity=value(1.,1);const double normal=std::abs(w.rotation.col(2).dot(velocity));double clearance=std::sqrt(w.distance_squared(q))-.085;
   mean_offset+=q.head<2>().norm();max_offset=std::max(max_offset,q.head<2>().norm());boundary+=use>.95;
   gates.push_back({{"gate",piece+1},{"at",at},{"offset_m",{q.x(),q.y()}},{"offset_norm_m",q.head<2>().norm()},{"domain_utilization",use},{"crossing_surface_clearance_m",clearance},{"normal_speed_mps",normal},{"window_speed_mps",w.motion_velocity(at).norm()},{"passage_scale_2r_over_vn_s",normal>0?Json(.3/normal):Json(nullptr)}});
  }
 }
 Json out{{"time_s",at},{"distance_m",length},{"mean_offset_m",mean_offset/gates.size()},{"max_offset_m",max_offset},{"gates_near_domain_boundary",boundary},{"gate_count",gates.size()},{"gates",gates},{"clearance_at_crossing_only_not_safety_verdict",true}};
 std::ofstream(argv[4])<<out.dump(2)<<'\n';std::cout<<out.dump(2)<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
