
#ifndef ARMOR_SOLVER_SOLVER_HPP_
#define ARMOR_SOLVER_SOLVER_HPP_


// std
#include <memory>
// ros2
#include <tf2_ros/buffer.h>

#include <rclcpp/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// 3rd party
#include <Eigen/Dense>
// project
#include "rm_interfaces/msg/green.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"

namespace imca::auto_aim {
class Solver {
public:
  explicit Solver(std::weak_ptr<rclcpp::Node> node);
  ~Solver() = default;

  rm_interfaces::msg::GimbalCmd solve(const rm_interfaces::msg::Green &green_msg);

  std::vector<std::pair<double, double>> getTrajectory(double distance,
                                                       double angle) const noexcept {
    return trajectory_compensator_->getTrajectory(distance, angle);
  }

private:

  void calcYawAndPitch(const Eigen::Vector3d &p,
                       double &yaw,
                       double &pitch) const noexcept;

  std::unique_ptr<TrajectoryCompensator> trajectory_compensator_;

  std::weak_ptr<rclcpp::Node> node_;

};

}//namespace imca::auto_aim

#endif
