#pragma once

#include "dynobench/dyno_macros.hpp"
#include "dynobench/for_each_macro.hpp"
#include "dynobench/robot_models_base.hpp"

namespace dynobench {

struct Bar_integrator2_2d_params {
  Bar_integrator2_2d_params(const char *file) { read_from_yaml(file); }
  Bar_integrator2_2d_params() = default;

  double max_vel = 1.;         // m/s
  double max_acc = 1.;         // m/s^2
  double max_angular_vel = 2.; // rad/s
  double dt = .1;

  // For computing distance between states
  Eigen::Vector4d distance_weights = Eigen::Vector4d(1, 1);
  Eigen::Vector4d u_ub;
  Eigen::Vector4d u_lb;

  // for collision shape
  std::string shape = "box";
  double l = 0.5;
  double radius = 0.1;
  Eigen::Vector2d size = Eigen::Vector2d(.5, .25);

  // filename used to load the paratemers, it is set by read_from_yaml
  std::string filename = "";

  void read_from_yaml(YAML::Node &node);
  void read_from_yaml(const char *file);

  void write(std::ostream &out) {
    const std::string be = "";
    const std::string af = ": ";

    out << be << STR(filename, af) << std::endl;
    out << be << STR(max_vel, af) << std::endl;
    out << be << STR(max_acc, af) << std::endl;
    out << be << STR(max_angular_vel, af) << std::endl;
    out << be << STR(l, af) << std::endl;
    out << be << STR_VV(distance_weights, af) << std::endl;
    out << be << STR_VV(size, af) << std::endl;
    out << be << STR_VV(u_lb, af) << std::endl;
    out << be << STR_VV(u_ub, af) << std::endl;
  }
};

struct Bar_integrator2_2d : Model_robot {

  virtual ~Bar_integrator2_2d() = default;

  Bar_integrator2_2d_params params;

  // Eigen::VectorXd state_weights;
  // Eigen::VectorXd state_ref;

  std::vector<std::unique_ptr<fcl::CollisionObjectd>> collision_objects;
  bool check_inter_r_col = true;

  Eigen::VectorXd ff;

  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots_;

  Bar_integrator2_2d(const Bar_integrator2_2d &) = default;

  // Bar_integrator2_2d(const char *file,
  //                       const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
  //                       const Eigen::VectorXd &p_ub = Eigen::VectorXd())
  //     : Bar_integrator2_2d(Bar_integrator2_2d_params(file), p_lb, p_ub) {}

  Bar_integrator2_2d(
      const Bar_integrator2_2d_params &params = Bar_integrator2_2d_params(),
      const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
      const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual void write_params(std::ostream &out) override { params.write(out); }

  virtual void ensure(Eigen::Ref<Eigen::VectorXd> xout) override {
    xout(2) = wrap_angle(xout(2));
  }

  void get_payload_pos(const Eigen::Ref<const Eigen::VectorXd> &x,
                       Eigen::Ref<Eigen::Vector2d> out) {
    out = x.head<2>();
  }

  void get_payload_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                       Eigen::Ref<Eigen::Vector2d> out) {
    out = x.segment(3, 2);
  }

  void get_robot1_pos(const Eigen::Ref<const Eigen::VectorXd> &x,
                      Eigen::Ref<Eigen::Vector2d> out) {
    Eigen::Vector2d payload_pos;
    get_payload_pos(x, payload_pos);
    double th;
    get_th(x, th);

    out[0] = payload_pos[0] + (params.l / 2) * cos(th);
    out[1] = payload_pos[1] + (params.l / 2) * sin(th);
  }

  void get_robot2_pos(const Eigen::Ref<const Eigen::VectorXd> &x,
                      Eigen::Ref<Eigen::Vector2d> out) {
    Eigen::Vector2d payload_pos;
    get_payload_pos(x, payload_pos);
    double th;
    get_th(x, th);

    out[0] = payload_pos[0] - (params.l / 2) * cos(th);
    out[1] = payload_pos[1] - (params.l / 2) * sin(th);
  }

  void get_th(const Eigen::Ref<const Eigen::VectorXd> &x, double &out) {

    out = x(2);
    wrap_angle(out);
  }

  virtual void calcV(Eigen::Ref<Eigen::VectorXd> f,
                     const Eigen::Ref<const Eigen::VectorXd> &x,
                     const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                         Eigen::Ref<Eigen::MatrixXd> Jv_u,
                         const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void step(Eigen::Ref<Eigen::VectorXd> xnext,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u,
                    double dt) override;

  virtual void stepDiff(Eigen::Ref<Eigen::MatrixXd> Fx,
                        Eigen::Ref<Eigen::MatrixXd> Fu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u,
                        double dt) override;

  // Collisions
  // This updates the position of the collisions shape(s) of the robot.
  // The collision distance/check  is implemented  in the base class.

  virtual void transformation_collision_geometries(
      const Eigen::Ref<const Eigen::VectorXd> &x,
      std::vector<Transform3d> &ts) override;

  virtual void collision_distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  CollisionOut &cout) override;
};

} // namespace dynobench
