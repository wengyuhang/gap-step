// SPDX-License-Identifier: GPL-3.0-only
// C++ adaptation of uzh-rpg/rpg_time_optimal (Foehn, Romero, Scaramuzza).
// Original source and license retained under
// 复现/cpc_dynamic/vendor/rpg_time_optimal.
#include "model.hpp"
#include <iostream>
int main(int argc, char **argv) {
  try {
    if (argc < 4)
      throw std::runtime_error("planner SCENE VEHICLE OUTPUT [N=400]");
    auto scene = read(argv[1]), vehicle = read(argv[2]);
    fs::path out = argv[3];
    if (fs::exists(out / "result.json"))
      throw std::runtime_error("Refusing overwrite");
    fs::create_directories(out);
    auto start_time = std::chrono::steady_clock::now();
    Model model(vehicle);
    int N = argc > 4 ? std::stoi(argv[4]) : 400, W = scene["windows"].size();
    std::vector<Window> windows;
    std::vector<Vector> points;
    Vector first(scene["start"][0], scene["start"][1], scene["start"][2]),
        goal(scene["goal"][0], scene["goal"][1], scene["goal"][2]);
    for (auto &w : scene["windows"]) {
      windows.push_back(load_window(w));
      points.push_back(windows.back().center);
    }
    points.push_back(goal);
    std::vector<double> dist;
    double length = 0;
    Vector last = first;
    for (auto &p : points) {
      length += (p - last).norm();
      dist.push_back(length);
      last = p;
    }
    std::vector<int> sw;
    for (double d : dist)
      sw.push_back(int(N * d / length));
    if (sw[0] == 0)
      throw std::runtime_error("Insufficient initial grid");
    std::vector<MX> vars, cons;
    std::vector<double> guess, low, high;
    std::vector<size_t> complement_rows;
    auto variable = [&](const std::string &name, const std::vector<double> &g) {
      MX v = MX::sym(name, g.size());
      vars.push_back(v);
      guess.insert(guess.end(), g.begin(), g.end());
      return v;
    };
    auto constraint = [&](const MX &v, double l, double h) {
      cons.push_back(v);
      low.insert(low.end(), v.numel(), l);
      high.insert(high.end(), v.numel(), h);
    };
    auto equal = [&](const MX &v, const DM &target) {
      cons.push_back(v);
      auto t = target.get_elements();
      low.insert(low.end(), t.begin(), t.end());
      high.insert(high.end(), t.begin(), t.end());
    };
    MX fidelity=MX::sym("fidelity");
    MX T = variable("T", {length / 3.});
    constraint(T, .1, 150.);
    std::vector<double> x0(13, 0);
    for (int k = 0; k < 3; ++k)
      x0[k] = first[k];
    x0[6] = 1;
    MX x = variable("initial_state", x0);
    equal(x, DM(x0));
    MX mu = variable("initial_progress", std::vector<double>(W, 1));
    constraint(mu, 1, 1);
    MX ufirst, ulast;
    int wp = 0;
    double fmin = vehicle["rotor_min_n"], fmax = vehicle["rotor_max_n"];
    double rateXY = std::sqrt(vehicle["rate_squared_limits"][0].get<double>()),
           rateZ = std::sqrt(vehicle["rate_squared_limits"][1].get<double>());
    for (int i = 0; i < N; ++i) {
      MX u = variable("u" + std::to_string(i), std::vector<double>(4, fmax));
      if (i == 0)
        ufirst = u;
      ulast = u;
      constraint(u, fmin, fmax);
      MX prediction=MX::if_else(fidelity>0.5,model.step(std::vector<MX>{x,u,T/N})[0],model.rawstep(std::vector<MX>{x,u,T/N})[0]);
      if (i > sw[wp])
        ++wp;
      Vector a = wp ? points[wp - 1] : first, b = points[wp];
      double fraction = wp ? double(i - sw[wp - 1]) / (sw[wp] - sw[wp - 1])
                           : double(i) / sw[0];
      Vector pos = (1 - fraction) * a + fraction * b,
             vel = 3 * (b - a).normalized();
      std::vector<double> gx(13, 0);
      for (int k = 0; k < 3; ++k) {
        gx[k] = pos[k];
        gx[3 + k] = vel[k];
      }
      gx[6] = 1;
      x = variable("x" + std::to_string(i), gx);
      constraint(x - prediction, 0, 0);
      std::vector<double> gl(W, 0);
      if (wp < W && ((wp == 0 && i + 1 >= sw[0]) ||
                     (wp > 0 && i + 1 - sw[wp - 1] >= sw[wp])))
        gl[wp] = 1;
      MX lam = variable("lambda" + std::to_string(i), gl);
      constraint(lam, 0, 1);
      for (int j = 0; j < W; ++j) {
        MX delta = x(casadi::Slice(0, 3)) - center(windows[j], (i + 1) * T / N);
        complement_rows.push_back(low.size());
        constraint(lam(j) * (MX::dot(delta, delta) - .09), -INFINITY, .01);
      }
      std::vector<double> gm(W, 1);
      for (int j = 0; j < W; ++j)
        if (i + 1 >= sw[j])
          gm[j] = 0;
      MX oldmu = mu;
      mu = variable("mu" + std::to_string(i), gm);
      constraint(oldmu - lam - mu, 0, 0);
      constraint(x(casadi::Slice(10, 12)), -rateXY, rateXY);
      constraint(x(12), -rateZ, rateZ);
      constraint(x(2), .5, 100);
      for (int j = 0; j < W - 1; ++j)
        constraint(mu(j + 1) - mu(j), 0, 1);
    }
    constraint(mu, 0, 0);
    constraint(x(casadi::Slice(3, 6)), 0, 0);
    constraint(x(casadi::Slice(10, 13)), 0, 0);
    equal(x(casadi::Slice(0, 3)), dm(goal));
    constraint(x(casadi::Slice(7, 10)), 0, 0);
    equal(ufirst, DM(std::vector<double>(4, model.mass * model.gravity / 4)));
    equal(ulast, DM(std::vector<double>(4, model.mass * model.gravity / 4)));
    MX X = MX::vertcat(vars), G = MX::vertcat(cons);
    Function constraints("constraints", {X,fidelity}, {G});
    save(out / "protocol.json",
         {{"nodes", N},
          {"waypoints", W},
          {"tolerance_m", .3},
          {"complementarity_relaxation", .01},
          {"integration_substeps", Model::integration_substeps},
          {"initial_time_s", length / 3.},
          {"initialization",
           "author chord/progress, velocity guess3; single start"},
          {"variables", X.numel()},
          {"constraints", G.numel()},
          {"vehicle", vehicle},
          {"maximum_solver_calls", 7}, {"complementarity_schedule", {.01,.001,1e-4,1e-5,1e-6,1e-7,1e-8}},
          {"max_iterations", 10000},
          {"scene", fs::absolute(argv[1]).string()},
          {"endpoint", "fixed position, zero velocity/bodyrate, level yaw0, "
                       "endpoint hover inputs"}});
    save(out / "initial.json", guess);
    casadi::Dict opts;
    opts["ipopt.max_iter"] = 10000;
    opts["ipopt.print_level"] = 5;
    opts["ipopt.constr_viol_tol"] = 1e-9;
    opts["ipopt.bound_relax_factor"] = 0.;
    opts["ipopt.warm_start_init_point"]="yes";
    opts["ipopt.warm_start_bound_push"]=1e-9;
    opts["ipopt.warm_start_mult_bound_push"]=1e-9;
    opts["ipopt.warm_start_slack_bound_push"]=1e-9;
    opts["ipopt.mu_strategy"] = "adaptive";
    Function solver = casadi::nlpsol(
        "solver", "ipopt", casadi::MXDict{{"x", X}, {"f", T}, {"g", G}, {"p",fidelity}}, opts);
    casadi::Dict coarse_opts;coarse_opts["ipopt.max_iter"]=10000;coarse_opts["ipopt.print_level"]=5;
    Function coarse_solver=casadi::nlpsol("coarse_solver","ipopt",casadi::MXDict{{"x",X},{"f",T},{"g", MX::substitute(G,fidelity,MX(1))},{"p",fidelity}},coarse_opts);
    auto built = std::chrono::steady_clock::now();
    casadi::DMDict solution;
    DM initial(guess);
    Json stages=Json::array();
    int stage_index=0;
    std::vector<double> schedule{.01,.001,1e-4,1e-5,1e-6,1e-7,1e-8};
    for(;stage_index<int(schedule.size());) {
      double epsilon=schedule[stage_index];
      for(size_t k:complement_rows) high[k]=epsilon;
      auto stage_begin=std::chrono::steady_clock::now();
      Function active=stage_index==0?coarse_solver:solver;
      casadi::DMDict arguments{{"x0",initial},{"lbg",DM(low)},{"ubg",DM(high)},{"p",DM(1)}};
      if(stage_index>0){arguments["lam_g0"]=solution.at("lam_g");arguments["lam_x0"]=solution.at("lam_x");}
      solution=active(arguments);
      auto st=active.stats();
      stages.push_back({{"epsilon",epsilon}, {"normalized_rotation",true},{"status",st.at("return_status").to_string()},{"iterations",st.at("iter_count").to_int()},{"wall_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-stage_begin).count()}});
      save(out/"stages.json",stages);
      save(out/("stage_"+std::to_string(stage_index)+"_optimizer.json"),{{"x",solution.at("x").get_elements()},{"lam_g",solution.at("lam_g").get_elements()},{"lam_x",solution.at("lam_x").get_elements()}});
      initial=solution.at("x");
      ++stage_index;
      if(!st.at("success").to_bool()) break;
    }
    auto ended = std::chrono::steady_clock::now();
    auto values = solution.at("x").get_elements(),
         g = constraints(std::vector<DM>{solution.at("x"),DM(1)})[0].get_elements();
    double violation = 0;
    bool finite = true;
    for (size_t k = 0; k < g.size(); ++k) {
      finite = finite && std::isfinite(g[k]);
      violation = std::max({violation, low[k] - g[k], g[k] - high[k]});
    }
    for (double a : values)
      finite = finite && std::isfinite(a);
    auto stats = stage_index==1?coarse_solver.stats():solver.stats();
    std::string status = stats.at("return_status").to_string();
    bool success = stats.at("success").to_bool();
    Json states = Json::array(), controls = Json::array(),
         progress = Json::array();
    states.push_back(
        std::vector<double>(values.begin() + 1, values.begin() + 14));
    progress.push_back(
        std::vector<double>(values.begin() + 14, values.begin() + 14 + W));
    int offset = 14 + W, width = 17 + 2 * W;
    for (int i = 0; i < N; ++i) {
      int k = offset + i * width;
      controls.push_back(
          std::vector<double>(values.begin() + k, values.begin() + k + 4));
      states.push_back(
          std::vector<double>(values.begin() + k + 4, values.begin() + k + 17));
      progress.push_back(std::vector<double>(values.begin() + k + 17 + W,
                                             values.begin() + k + width));
    }
    int total_iterations=0; for(auto &st:stages) total_iterations+=st["iterations"].get<int>();
    Json result = {
        {"found", finite && violation <= 1e-7 && stages.back()["epsilon"].get<double>() <= 1e-8}, {"stages",stages}, {"integration_substeps",Model::integration_substeps},
        {"optimizer_converged", success},
        {"status", status},
        {"iterations", total_iterations}, {"last_stage_iterations",stats.at("iter_count").to_int()},
        {"constraint_violation", violation},
        {"nominal_time_s", values[0]},
        {"nodes", N},
        {"states", states},
        {"controls", controls},
        {"progress", progress},
        {"setup_s", std::chrono::duration<double>(built - start_time).count()},
        {"solve_s", std::chrono::duration<double>(ended - built).count()},
        {"total_compute_s",
         std::chrono::duration<double>(ended - start_time).count()},
        {"method", "dynamic_cpc_center_neighborhood"},
        {"vehicle", vehicle},
        {"scene_source", fs::absolute(argv[1]).string()},
        {"planning_only", true},
        {"runtime", "C++ CasADi IPOPT; author CPC equations with dynamic "
                    "center substitution"}};
    save(out / "result.json", result);
    save(out / "optimizer.json",
         {{"x", values},
          {"lam_g", solution.at("lam_g").get_elements()},
          {"lam_x", solution.at("lam_x").get_elements()}});
    std::cout << "CPC_RESULT " << result["found"] << " " << status << " "
              << values[0] << std::endl;
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
