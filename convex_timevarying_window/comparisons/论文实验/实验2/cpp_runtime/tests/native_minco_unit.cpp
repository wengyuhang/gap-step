#include "native_minco_pvaj.hpp"
#include <filesystem>
int main(int argc,char** argv) {
 try {
  if(argc!=4)throw std::runtime_error("Usage: native_minco_test GRAPH SCENE OUTPUT");
  Json m,s;std::ifstream(std::string(argv[1])+"/manifest.json")>>m;std::ifstream(argv[2])>>s;
  std::vector<Window> windows;for(auto& w:m["windows"])windows.push_back(load_window(w));
  native_minco::Graph graph(m,s,windows);
  using casadi::DM;using casadi::MX;using casadi::Function;
  const int n=m["n"],nv=m["nvar"],vs=m["velocity_start"];
  const auto original=m["initials"][0]["original"].get<std::vector<double>>();
  const auto parameters=m["initials"][0]["parameters"].get<std::vector<double>>();
  const auto scale=m["state_scale"].get<std::vector<double>>();
  auto z=original;
  for(int k=0;k<n;++k){double t=std::exp(z[k]);z[k]=t>1.?std::sqrt(2.*t-1.)-1.:1.-std::sqrt(2./t-1.);}
  for(int k=vs;k<nv;++k)z[k]/=scale[k-vs];
  const auto reference=Function::load(std::string(argv[1])+"/physical_curve.casadi");
  const auto serialized=[](const std::string& path){std::ifstream f(path);return std::string((std::istreambuf_iterator<char>(f)),{});};
  const auto old=casadi::StringDeserializer(serialized(std::string(argv[1])+"/symbolic.mx")).unpack_mx_vector();
  std::vector<MX> old_outputs{old[6]};for(size_t k=7;k<old.size();++k)old_outputs.push_back(old[k]);
  const Function old_geometry("audit_old_geometry",{old[0],old[1]},old_outputs);
  std::vector<MX> new_outputs{graph.expressions[6]};for(size_t k=7;k<graph.expressions.size();++k)new_outputs.push_back(graph.expressions[k]);
  const Function new_geometry("audit_native_geometry",{graph.expressions[0],graph.expressions[1]},new_outputs);
  const auto old_value=old_geometry(std::vector<DM>{DM(m["initials"][0]["finite"].get<std::vector<double>>()),DM(parameters)});
  const auto new_value=new_geometry(std::vector<DM>{DM(z),DM(parameters)});
  double geometry_error=0.;
  const int old_workspace=m["workspace_rows"],gates=m["m"];
  for(size_t k=0;k<new_value.size();++k) {
   const auto a=new_value[k].get_elements(),b=old_value[k].get_elements();
   for(size_t j=0;j<a.size();++j) {
    const size_t old_index=k==0 ? (j<static_cast<size_t>(gates) ? old_workspace+j : old_workspace+n+j) : j;
    geometry_error=std::max(geometry_error,std::abs(a[j]-b[old_index]));
   }
  }
  Json unbounded=m;native_minco::remove_domain_bounds(unbounded);
  const int gauges=m["free_points"].get<bool>() ? std::count_if(windows.begin(),windows.end(),[](const Window& w){return !w.circle;}) : 0;
  const bool domain_rows_removed=graph.expressions[6].numel()==gates+gauges &&
      unbounded["workspace_rows"]==0 && unbounded["config"]["workspace_lower"].is_null() &&
      unbounded["lower_g"].size()==m["lower_g"].size()-old_workspace-n;
  const MX weight=MX::sym("coefficient_weights",24*n);
  const MX objective=MX::dot(weight,graph.expressions[4]);
  const Function derivative("audit_native_gradient",{graph.expressions[0],graph.expressions[1],weight},
   {MX::gradient(objective,graph.expressions[0])});
  double coefficient_error=0.,direction_error=0.;
  std::vector<double> weights(24*n);for(size_t i=0;i<weights.size();++i)weights[i]=.01*std::sin(.31*i+.7);
  for(int trial=0;trial<12;++trial){
   auto x=z;for(int k=0;k<nv;++k)if(trial)x[k]+=.005*std::sin(.3*trial+.9*k);
   const auto actual=graph.trajectory(std::vector<DM>{DM(x),DM(parameters)});
   const auto expected=reference(std::vector<DM>{actual[0]})[0].get_elements();
   const auto coefficients=actual[2].get_elements();
   for(size_t k=0;k<expected.size();++k)coefficient_error=std::max(coefficient_error,std::abs(coefficients[k]-expected[k])/(1.+std::abs(expected[k])));
   const auto gradient=derivative(std::vector<DM>{DM(x),DM(parameters),DM(weights)})[0].get_elements();
   for(int group=0;group<3;++group){
    auto plus=x,minus=x;double automatic=0.;const double h=1e-6;
    for(int k=0;k<nv;++k){double d=std::sin(.23*k+.7*trial+.9*group);if((group==0 && k>=n)||(group==1 && (k<n||k>=vs))||(group==2&&k<vs))d=0.;plus[k]+=h*d;minus[k]-=h*d;automatic+=gradient[k]*d;}
    const auto high=graph.trajectory(std::vector<DM>{DM(plus),DM(parameters)})[2].get_elements();
    const auto low=graph.trajectory(std::vector<DM>{DM(minus),DM(parameters)})[2].get_elements();double finite=0.;
    for(size_t k=0;k<weights.size();++k)finite+=weights[k]*(high[k]-low[k])/(2*h);
    direction_error=std::max(direction_error,std::abs(finite-automatic)/(1.+std::abs(automatic)));
   }
  }
  Json dynamics_metadata;std::ifstream(std::string(argv[1])+"/exchange_dynamics.json")>>dynamics_metadata;
  m["dynamic_residual_scale"]=dynamics_metadata["residual_scale"];
  const auto dynamics=Function::load(std::string(argv[1])+"/exchange_dynamic_node.casadi");
  native_minco::Graph soft_graph(m,s,windows,dynamics);
  auto soft_x=z;soft_x.resize(nv+n*33*6,0.);
  const auto normalized_residuals=soft_graph.residuals(std::vector<DM>{DM(soft_x),DM(parameters)})[0].get_elements();
  const auto slack_scale=m["slack_scale"].get<std::vector<double>>();
  for(size_t k=0;k<normalized_residuals.size();++k)soft_x[nv+k]=std::max(normalized_residuals[k],0.)/slack_scale[k]+.3;
  auto legacy_x=m["initials"][0]["finite"].get<std::vector<double>>();
  std::copy(soft_x.begin()+nv,soft_x.end(),legacy_x.begin()+nv);
  const Function soft_objective("audit_soft_objective",{soft_graph.expressions[0],soft_graph.expressions[1]},
      {soft_graph.expressions[5]});
  const auto legacy_base=Function::load(std::string(argv[1])+"/base.casadi");
  const double legacy_objective=static_cast<double>(legacy_base(std::vector<DM>{DM(legacy_x),DM(parameters)})[0])*parameters[0];
  double total=0.;for(int piece=0;piece<n;++piece)total+=std::exp(original[piece]);
  const double expected_soft=total+(legacy_objective-total)/1000.;
  const double actual_soft=static_cast<double>(soft_objective(std::vector<DM>{DM(soft_x),DM(parameters)})[0])*parameters[0];
  const double soft_value_error=std::abs(actual_soft-expected_soft)/(1.+std::abs(expected_soft));
  std::vector<double> probe_weights(soft_graph.expressions[6].numel());
  for(size_t k=0;k<probe_weights.size();++k)probe_weights[k]=.001*std::sin(.11*k+.2);
  const MX soft_probe=soft_graph.expressions[5]+MX::dot(DM(probe_weights),soft_graph.expressions[6]);
  const Function soft_probe_function("audit_soft_gradient",{soft_graph.expressions[0],soft_graph.expressions[1]},
      {soft_probe,MX::gradient(soft_probe,soft_graph.expressions[0])});
  const auto gradient=soft_probe_function(std::vector<DM>{DM(soft_x),DM(parameters)})[1].get_elements();
  double soft_direction_error=0.;
  for(int group=0;group<4;++group) {
    auto plus=soft_x,minus=soft_x;double automatic=0.;constexpr double h=1e-6;
    for(size_t k=0;k<soft_x.size();++k) {
      bool selected=(group==0 && k<static_cast<size_t>(n)) ||
          (group==1 && k>=static_cast<size_t>(n) && k<static_cast<size_t>(vs)) ||
          (group==2 && k>=static_cast<size_t>(vs) && k<static_cast<size_t>(nv)) ||
          (group==3 && k>=static_cast<size_t>(nv));
      const double direction=selected ? std::sin(.23*k+.7*group) : 0.;
      plus[k]+=h*direction;minus[k]-=h*direction;automatic+=gradient[k]*direction;
    }
    const double high=static_cast<double>(soft_probe_function(std::vector<DM>{DM(plus),DM(parameters)})[0]);
    const double low=static_cast<double>(soft_probe_function(std::vector<DM>{DM(minus),DM(parameters)})[0]);
    soft_direction_error=std::max(soft_direction_error,std::abs((high-low)/(2*h)-automatic)/(1.+std::abs(automatic)));
  }
  native_minco::Graph weighted_graph(m,s,windows,dynamics,1000.);
  const Function weighted_objective("audit_weight1000",{weighted_graph.expressions[0],weighted_graph.expressions[1]},
      {weighted_graph.expressions[5]});
  const double weighted_value=static_cast<double>(weighted_objective(std::vector<DM>{DM(soft_x),DM(parameters)})[0])*parameters[0];
  const double weighted_error=std::abs(weighted_value-legacy_objective)/(1.+std::abs(legacy_objective));
  const auto coefficient_jacobian=MX::jacobian(graph.expressions[4],graph.expressions[0]);
  const int declared_nonzeros=coefficient_jacobian.nnz();
  const bool passed=domain_rows_removed && weighted_error<1e-9 && soft_value_error<1e-9 && soft_direction_error<2e-6 && declared_nonzeros<coefficient_jacobian.numel()/2 && geometry_error<1e-7 && coefficient_error<1e-8 && direction_error<2e-6;
  const Json result{{"passed",passed},{"domain_rows_removed",domain_rows_removed},{"remaining_geometric_base_rows",graph.expressions[6].numel()},{"cases",12},{"direction_tests",36},{"derivative_order",3},{"variables",nv},{"coefficient_jacobian_declared_nonzeros",declared_nonzeros},{"coefficient_jacobian_dense_entries",24*n*nv},
   {"weight1000_objective_relative_error",weighted_error},{"soft_objective_relative_error",soft_value_error},{"soft_objective_constraint_gradient_error",soft_direction_error},{"soft_trajectory_variables",nv},{"soft_auxiliary_variables",n*33*6},
   {"initial_geometry_absolute_error",geometry_error},{"normalized_coefficient_relative_error",coefficient_error},{"directional_gradient_relative_error",direction_error}};
  std::ofstream(argv[3])<<result.dump(2)<<'\n';std::cout<<result.dump(2)<<'\n';return passed?0:1;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
