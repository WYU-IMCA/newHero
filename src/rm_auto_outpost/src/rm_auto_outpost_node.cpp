#include"rm_auto_outpost/rm_auto_outpost_node.hpp"
// ros2
#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>

#include <rclcpp/qos.hpp>
// third party
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

namespace fyt::auto_outpost{
AutoOutpostNode::AutoOutpostNode(const rclcpp::NodeOptions &options)
: Node("auto_outpost", options){
  FYT_REGISTER_LOGGER("outpost_solver", "~/fyt2024-log", INFO);
  FYT_INFO("outpost_solver", "Starting OutpostSolverNode!");

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(this, "armor_detector/armors", rmw_qos_profile_sensor_data);
  tf2_filter_ = std::make_shared<tf2_filter>(armors_sub_,
                                             *tf2_buffer_,
                                             "odom",
                                             10,
                                             this->get_node_logging_interface(),
                                             this->get_node_clock_interface(),
                                             std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when
  // transforms are available
  tf2_filter_->registerCallback(&AutoOutpostNode::armorsCallback, this);

  enable_ = false;

  // Enable/Disable Outpost Solver
  set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
  "outpost_solver/set_mode",
  std::bind(
      &AutoOutpostNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

  gimbal_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>("outpost_solver/cmd_gimbal",
                                                                    rclcpp::SensorDataQoS());

  serial_receive_data_sub_ = this->create_subscription<rm_interfaces::msg::SerialReceiveData>
  ("serial/receive",rclcpp::SensorDataQoS(),[this](rm_interfaces::msg::SerialReceiveData serial_info){
    IMU_yaw = serial_info.yaw;
    IMU_pitch = serial_info.pitch;
  });

}

void AutoOutpostNode::armorsCallback(const rm_interfaces::msg::Armors::SharedPtr armors_msg){
  // Return if not enable
  if (!enable_) {
    if(solver_ != nullptr){
      solver_->Init();
    }
    return;
  }

  // Lazy initialize solver owing to weak_from_this() can't be called in constructor
  if (solver_ == nullptr) {
    solver_ = std::make_unique<OutpostSolver>();
    RCLCPP_INFO(rclcpp::get_logger("outpost_solver"),"CREATED solver");
  }

  // Tranform armor position from image frame to world coordinate
  for (auto &armor : armors_msg->armors) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try {
      armor.pose = tf2_buffer_->transform(ps, "odom").pose;
    } catch (const tf2::TransformException &ex) {
      FYT_ERROR("outpost_solver", "Transform error: {}", ex.what());
      return;
    }
  }

  // Filter abnormal armors
  // armors_msg->armors.erase(std::remove_if(armors_msg->armors.begin(),
  //                                         armors_msg->armors.end(),
  //                                         [this](const rm_interfaces::msg::Armor &armor) {
  //                                           return abs(armor.pose.position.z) < 0.6 || Eigen::Vector2d(armor.pose.position.x,armor.pose.position.y).norm()>10;
  //                                         }),
  //                          armors_msg->armors.end());

  rm_interfaces::msg::GimbalCmd control_msg;

  control_msg = solver_->solve(armors_msg,IMU_pitch,IMU_yaw);

  // control_msg = solver->old_solve(armors_msg);
  
  gimbal_pub_->publish(control_msg);
}

void AutoOutpostNode::setModeCallback(
  const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
  response->success = true;

  VisionMode mode = static_cast<VisionMode>(request->mode);
  std::string mode_name = visionModeToString(mode);
  if (mode_name == "UNKNOWN") {
    return;
  }

  switch (mode) {
    case VisionMode::AUTO_OUTPOST: {
      enable_ = true;
      break;
    }
    default: {
      enable_ = false;
      break;
    }
  }

}


}// namespace fyt::outpost

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_outpost::AutoOutpostNode)