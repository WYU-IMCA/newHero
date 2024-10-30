#include "green_solver/green_solver.hpp"
// std
#include <cmath>
#include <cstddef>
#include <stdexcept>
// project
#include "green_solver/green_solver_node.hpp"
#include "rm_utils/math/utils.hpp"

namespace imca::auto_aim {
Solver::Solver(std::weak_ptr<rclcpp::Node> n) : node_(n) {
  auto node = node_.lock();

  std::string compenstator_type = node->declare_parameter("solver.compensator_type", "ideal");
  trajectory_compensator_ = CompensatorFactory::createCompensator(compenstator_type);
  trajectory_compensator_->iteration_times = node->declare_parameter("solver.iteration_times", 20);
  trajectory_compensator_->velocity = node->declare_parameter("solver.bullet_speed", 20.0);
  trajectory_compensator_->gravity = node->declare_parameter("solver.gravity", 9.8);
  trajectory_compensator_->resistance = node->declare_parameter("solver.resistance", 0.001);

  node.reset();
}

rm_interfaces::msg::GimbalCmd Solver::solve(const rm_interfaces::msg::Green &green_msg) {
    Eigen::Vector3d p;
    p<<green_msg.pose.position.x,green_msg.pose.position.y,green_msg.pose.position.z;

    // Initialize gimbal_cmd
    rm_interfaces::msg::GimbalCmd gimbal_cmd;
    double yaw,pitch;
    calcYawAndPitch(p,yaw,pitch);
    gimbal_cmd.yaw = yaw;
    gimbal_cmd.pitch = pitch;
    return gimbal_cmd;
}

void Solver::calcYawAndPitch(const Eigen::Vector3d &p,
                             double &yaw,
                             double &pitch) const noexcept {
  // Calculate yaw and pitch
  yaw = atan2(p.y(), p.x());
  pitch = atan2(p.z(), p.head(2).norm());

  if (double temp_pitch = pitch; trajectory_compensator_->compensate(p, temp_pitch)) {
    pitch = temp_pitch;
  }
}

} //namespace imca::auto_aim