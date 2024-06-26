#include "dynobench/bar_integrator2_2d.hpp"
#include "Bar_integrator2_2d_dynamics.hpp" 
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/capsule.h>
#include <fcl/geometry/shape/sphere.h>

namespace dynobench {

void Bar_integrator2_2d_params::read_from_yaml(YAML::Node &node) {
  set_from_yaml(node, VAR_WITH_NAME(max_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_acc));
  set_from_yaml(node, VAR_WITH_NAME(filename));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights));
  set_from_yaml(node, VAR_WITH_NAME(size));
  set_from_yaml(node, VAR_WITH_NAME(shape));
  set_from_yaml(node, VAR_WITH_NAME(l));
  set_from_yaml(node, VAR_WITH_NAME(u_ub));
  set_from_yaml(node, VAR_WITH_NAME(u_lb));
  set_from_yaml(node, VAR_WITH_NAME(dt));

}

void Bar_integrator2_2d_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}



Bar_integrator2_2d::Bar_integrator2_2d(const Bar_integrator2_2d_params &params, 
                                     const Eigen::VectorXd &p_lb,
                                     const Eigen::VectorXd &p_ub)
    : Model_robot(std::make_shared<Rn>(6), 4), params(params) {

  // description of state and control
  x_desc = {"p0x[m]", "p0y[m]", "th[rad]", "dp0x[m/s]", "dp0y[m/s]", "dth[rad/s]",};
  u_desc = {"ddp1x[m/s^2]", "ddp1y[m/s^2]", "ddp2x[m/s^2]", "ddp2y[m/s^2]"};

  const double RM_max__ = std::sqrt(std::numeric_limits<double>::max());
  const double RM_low__ = -RM_max__;

  using Vxd = Eigen::VectorXd;

  std::cout << "Robot name " << name << std::endl;
  std::cout << "Parameters" << std::endl;
  this->params.write(std::cout);
  std::cout << "***" << std::endl;

  name = "Bar_integrator2_2d";
  translation_invariance = 2;
  invariance_reuse_col_shape = false;
  nx_col = 6; 
  nx_pr = 4;
  is_2d = true;
  
  u_weight.resize(4);
  u_weight.setConstant(1);
  x_weightb = Vxd::Ones(nx);
  x_weightb.setConstant(100);

  // state_weights = Vxd::Ones(nx);
  // state_ref = Vxd::Ones(nx);

  ref_dt = params.dt;
  distance_weights = params.distance_weights;
  // bound on state and control
  u_lb << -params.max_acc, -params.max_acc, -params.max_acc, -params.max_acc;
  u_ub << params.max_acc, params.max_acc, params.max_acc, params.max_acc;

  x_lb << low__, low__, -M_PI, -params.max_vel, -params.max_vel, -params.max_angular_vel;
  x_ub << max__, max__, M_PI,  params.max_vel, params.max_vel, params.max_angular_vel;

  // add bounds on position if provided
  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }

  // COLLISIONS
  collision_geometries.clear();

  double rate_box_size =
      .2; // we use a shorter collision body for the
          // cables to avoid self collision against payload or robot!
  collision_geometries.emplace_back(
      std::make_shared<fcl::Boxd>(rate_box_size * params.l, params.size(1), 1.0));


  collision_geometries.emplace_back(
        std::make_shared<fcl::Sphered>(params.radius));

  collision_geometries.emplace_back(
        std::make_shared<fcl::Sphered>(params.radius));

  ts_data.resize(3);
  col_outs.resize(3);


  for (auto &c : collision_geometries) {
    collision_objects.emplace_back(std::make_unique<fcl::CollisionObjectd>(c));
  }
  col_mng_robots_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
  col_mng_robots_->setup();
}

void Bar_integrator2_2d::transformation_collision_geometries(
  const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {
    // shapes:
    // payload, robot 1, robot 2 

    //payload
    Eigen::Vector2d pos_p;
    get_payload_pos(x, pos_p);
    double th;
    get_th(x, th);
    fcl::Transform3d result_p;
    result_p = Eigen::Translation<double, 3>(pos_p(0), pos_p(1), 0.0);
    result_p.rotate(
      Eigen::AngleAxisd(th, Eigen::Vector3d::UnitZ())
    );
    ts.at(0) = result_p;

    // robot 1 and 2 
    Eigen::Vector2d robot1;
    Eigen::Vector2d robot2;
    get_robot1_pos(x, robot1);
    get_robot2_pos(x, robot2);

    fcl::Transform3d result_r1;
    fcl::Transform3d result_r2;
    // std::cout << "rob pos: " << robot1 << std::endl;
    // std::cout << "rob pos: " << robot2 << std::endl;
    result_r1 = Eigen::Translation<double, 3>(robot1(0), robot1(1), 0.0);
    result_r1.rotate(
      Eigen::AngleAxisd(0., Eigen::Vector3d::UnitZ())
    );
    ts.at(1) = result_r1;

    result_r2 = Eigen::Translation<double, 3>(robot2(0), robot2(1), 0.0);
    result_r2.rotate(
      Eigen::AngleAxisd(0., Eigen::Vector3d::UnitZ())
    );
    ts.at(2) = result_r2;
}

void Bar_integrator2_2d::collision_distance(
    const Eigen::Ref<const Eigen::VectorXd> &x, CollisionOut &cout) {
  if (env && env->size()) {
    // agains environment
    Model_robot::collision_distance(x, cout);
    // std::end
  } else {
    cout.distance = max__;
  }

  if (check_inter_r_col) {
    // inner robots

    transformation_collision_geometries(x, ts_data);
    // Update the collision objects
    for (size_t i = 0; i < collision_geometries.size(); i++) {
      fcl::Transform3d &result = ts_data[i];
      assert(collision_objects.at(i));
      auto &co = *collision_objects.at(i);
      co.setTranslation(result.translation());
      co.setRotation(result.rotation());
      co.computeAABB();
    }

    std::vector<fcl::CollisionObjectd *> collision_objects_ptrs;
    collision_objects_ptrs.reserve(collision_objects.size());
    std::transform(collision_objects.begin(), collision_objects.end(),
                   std::back_inserter(collision_objects_ptrs),
                   [](auto &c) { return c.get(); });

    col_mng_robots_->clear();
    col_mng_robots_->registerObjects(collision_objects_ptrs);
    fcl::DefaultDistanceData<double> inter_robot_distance_data;
    inter_robot_distance_data.request.enable_signed_distance = true;

    col_mng_robots_->distance(&inter_robot_distance_data,
                              fcl::DefaultDistanceFunction<double>);

    double inter_robot_distance = inter_robot_distance_data.result.min_distance;
    // std::cout << "condition:" << inter_robot_distance << ","<< cout.distance << std::endl;
    // if (inter_robot_distance < cout.distance) {
    //   cout.distance = inter_robot_distance;
    //   cout.p1 = inter_robot_distance_data.result.nearest_points[0];
    //   cout.p2 = inter_robot_distance_data.result.nearest_points[1];
    //   std::cout << "inter_robot_distance: " << inter_robot_distance
    //             << std::endl;
    //   cout.write(std::cout);
    // }
    
  }
}


void Bar_integrator2_2d::calcV(Eigen::Ref<Eigen::VectorXd> ff,
                                  const Eigen::Ref<const Eigen::VectorXd> &x,
                                  const Eigen::Ref<const Eigen::VectorXd> &u) {

  // Call a function in the autogenerated file

  auto apply_fun = [&](auto &fun) {
    fun(ff.data(), params.l, x.data(), u.data());
  };
  apply_fun(calcV_Bar_integrator2_2d);

}

void Bar_integrator2_2d::calcDiffV(
    Eigen::Ref<Eigen::MatrixXd> Jv_x, Eigen::Ref<Eigen::MatrixXd> Jv_u,
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {

  // Call a function in the autogenerated file
  // NOT_IMPLEMENTED_TODO;

  auto apply_fun = [&](auto &fun) {
    fun(Jv_x.data(), Jv_u.data(), params.l, x.data(), u.data());
  };

  apply_fun(calcJ_Bar_integrator2_2d);

}

void Bar_integrator2_2d::step(Eigen::Ref<Eigen::VectorXd> xnext,
                                 const Eigen::Ref<const Eigen::VectorXd> &x,
                                 const Eigen::Ref<const Eigen::VectorXd> &u,
                                 double dt) {

  // Call a function in the autogenerated file
  std::cout << "THIS xnext:::" << xnext.data() << std::endl;
  std::cout << "THIS xnext:::" << xnext << std::endl;
  auto apply_fun = [&](auto &fun) {
    fun(xnext.data(), params.l, x.data(), u.data(), dt);
  };
  apply_fun(calcStep_Bar_integrator2_2d);
  // ensure(xnext);
}

void Bar_integrator2_2d::stepDiff(Eigen::Ref<Eigen::MatrixXd> Fx,
                                     Eigen::Ref<Eigen::MatrixXd> Fu,
                                     const Eigen::Ref<const Eigen::VectorXd> &x,
                                     const Eigen::Ref<const Eigen::VectorXd> &u,
                                     double dt) {

  // Call a function in the autogenerated file
  auto apply_fun = [&](auto &fun) {
    fun(Fx.data(), Fu.data(), params.l, x.data(), u.data(), dt);
  };

  Fx.setZero();
  Fu.setZero();
  apply_fun(calcF_Bar_integrator2_2d);

}

};