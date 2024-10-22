// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "armor_solver/armor_tracker.hpp"
// std
#include <cfloat>
#include <memory>
#include <string>
// ros2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// third party
#include <angles/angles.h>
// project
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim
{
  Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
      : tracker_state(LOST), tracked_id(std::string("")), measurement(Eigen::VectorXd::Zero(4)), target_state(Eigen::VectorXd::Zero(9)), max_match_distance_(max_match_distance), max_match_yaw_diff_(max_match_yaw_diff) {}

  void Tracker::init(const Armors::SharedPtr &armors_msg) noexcept
  {
    if (armors_msg->armors.empty())
    {
      return;
    }

    // Simply choose the armor that is closest to image center
    double min_distance = DBL_MAX;
    tracked_armor = armors_msg->armors[0];
    for (const auto &armor : armors_msg->armors)
    {
      if (armor.distance_to_image_center < min_distance)
      {
        min_distance = armor.distance_to_image_center;
        tracked_armor = armor;
      }
    }

    initEKF(tracked_armor);
    FYT_INFO("armor_solver", "Init EKF!");

    tracked_id = tracked_armor.number;
    tracker_state = DETECTING;

    updateArmorsNum(tracked_armor);
  }

  void Tracker::update(const Armors::SharedPtr &armors_msg) noexcept
  {
    // KF predict
    Eigen::VectorXd ekf_prediction = ekf.predict();

    bool matched = false;
    // Use KF prediction as default target state if no matched armor is found
    target_state = ekf_prediction;

    if (!armors_msg->armors.empty())
    {
      // Find the closest armor with the same id
      Armor same_id_armor;
      int same_id_armors_count = 0;
      auto predicted_position = getArmorPositionFromState(ekf_prediction);
      double min_position_diff = DBL_MAX;
      double yaw_diff = DBL_MAX;
      for (const auto &armor : armors_msg->armors)
      {
        // Only consider armors with the same id
        if (armor.number == tracked_id)
        {
          same_id_armor = armor;
          same_id_armors_count++;
          // Calculate the difference between the predicted position and the
          // current armor position
          auto p = armor.pose.position;
          Eigen::Vector3d position_vec(p.x, p.y, p.z);
          double position_diff = (predicted_position - position_vec).norm();
          if (position_diff < min_position_diff)
          {
            // Find the closest armor
            min_position_diff = position_diff;
            yaw_diff = abs(orientationToYaw(armor.pose.orientation) - ekf_prediction(6));
            tracked_armor = armor;
            // Update tracked armor type
            if (tracked_armor.type == "large" &&
                (tracked_id == "3" || tracked_id == "4" || tracked_id == "5"))
            {
              tracked_armors_num = ArmorsNum::BALANCE_2;
            }
            else if (tracked_id == "outpost")
            {
              tracked_armors_num = ArmorsNum::OUTPOST_3;
            }
            else
            {
              tracked_armors_num = ArmorsNum::NORMAL_4;
            }
          }
        }
      }

      // Store tracker info
      info_position_diff = min_position_diff;
      info_yaw_diff = yaw_diff;

      // Check if the distance and yaw difference of closest armor are within the
      // threshold
      // RCLCPP_INFO(rclcpp::get_logger("armor_tracker"),"min_position_diff:%f yaw_diff:%f same_id_armors_count:%d",min_position_diff,yaw_diff,same_id_armors_count);

      if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_)
      {
        // Matched armor found
        matched = true;
        auto p = tracked_armor.pose.position;
        // Update EKF
        double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
        measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
        target_state = ekf.update(measurement);
        //debug
        //RCLCPP_INFO(rclcpp::get_logger("armor_tracker"),"updated x:%f y:%f z:%f",target_state(0),target_state(2),target_state(4));
      }
      else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_)
      {
        // Matched armor not found, but there is only one armor with the same id
        // and yaw has jumped, take this case as the target is spinning and armor
        // jumped
        handleArmorJump(same_id_armor);
      }
      else
      {
        // No matched armor found
        FYT_WARN("armor_solver", "No matched armor found!");
      }
    }

    // Prevent radius from spreading
    if (target_state(8) < 0.12)
    {
      target_state(8) = 0.12;
      ekf.setState(target_state);
    }
    else if (target_state(8) > 0.4)
    {
      target_state(8) = 0.4;
      ekf.setState(target_state);
    }

    // Tracking state machine
    if (tracker_state == DETECTING)
    {
      if (matched)
      {
        detect_count_++;
        if (detect_count_ > tracking_thres)
        {
          detect_count_ = 0;
          tracker_state = TRACKING;
          FYT_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
        }
      }
      else
      {
        detect_count_ = 0;
        tracker_state = LOST;
        FYT_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
      }
    }
    else if (tracker_state == TRACKING)
    {
      if (!matched)
      {
        tracker_state = TEMP_LOST;
        lost_count_++;
        FYT_DEBUG("armor_solver", "Tracker state: TEMP_LOST {}", tracked_id);
      }
    }
    else if (tracker_state == TEMP_LOST)
    {
      if (!matched)
      {
        lost_count_++;
        if (lost_count_ > lost_thres)
        {
          lost_count_ = 0;
          tracker_state = LOST;
          FYT_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
        }
      }
      else
      {
        tracker_state = TRACKING;
        lost_count_ = 0;
        FYT_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
      }
    }
  }

  void Tracker::initEKF(const Armor &a) noexcept
  {
    double xa = a.pose.position.x;
    double ya = a.pose.position.y;
    double za = a.pose.position.z;
    last_yaw_ = 0;
    double yaw = orientationToYaw(a.pose.orientation);

    // Set initial position at 0.2m behind the target
    target_state = Eigen::VectorXd::Zero(9);
    double r = 0.26;
    double xc = xa + r * cos(yaw);
    double yc = ya + r * sin(yaw);
    dz = 0, another_r = r;
    target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;

    ekf.setState(target_state);
  }

  void Tracker::handleArmorJump(const Armor &current_armor) noexcept
  {
    // double last_yaw = target_state(6);
    // double yaw = orientationToYaw(current_armor.pose.orientation);

    // if (abs(yaw - last_yaw) > 0.4) {
    //   // Armor angle also jumped, take this case as target spinning
    //   target_state(6) = yaw;
    //   // Only 4 armors has 2 radius and height
    //   if (tracked_armors_num == ArmorsNum::NORMAL_4) {
    //     dz = target_state(4) - current_armor.pose.position.z;
    //     target_state(4) = current_armor.pose.position.z;
    //     std::swap(target_state(8), another_r);
    //   }
    //   FYT_DEBUG("armor_solver", "Armor Jump!");
    // }
    double yaw = orientationToYaw(current_armor.pose.orientation);
    target_state(6) = yaw;
    updateArmorsNum(current_armor);
    // Only 4 armors has 2 radius and height
    if (tracked_armors_num == ArmorsNum::NORMAL_4)
    {
      dz = target_state(4) - current_armor.pose.position.z;
      target_state(4) = current_armor.pose.position.z;
      std::swap(target_state(8), another_r);
    }
    RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Armor jump!");

    // If position difference is larger than max_match_distance_,
    // take this case as the ekf diverged, reset the state
    auto p = current_armor.pose.position;
    Eigen::Vector3d current_p(p.x, p.y, p.z);
    Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

    if ((current_p - infer_p).norm() > max_match_distance_)
    {
      // If the distance between the current armor and the inferred armor is too
      // large, the state is wrong, reset center position and velocity in the
      // state
      double r = target_state(8);
      target_state(0) = p.x + r * cos(yaw); // xc
      target_state(1) = 0;                  // vxc
      target_state(2) = p.y + r * sin(yaw); // yc
      target_state(3) = 0;                  // vyc
      target_state(4) = p.z;                // xz
      target_state(5) = 0;                  // vza
      FYT_WARN("armor_solver", "State wrong!");
    }

    ekf.setState(target_state);
  }

  double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion &q) noexcept
  {
    // Get armor yaw
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    // Make yaw change continuous (-pi~pi to -inf~inf)
    yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
    last_yaw_ = yaw;
    return yaw;
  }

  Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd &x) noexcept
  {
    // Calculate predicted position of the current armor
    double xc = x(0), yc = x(2), za = x(4);
    double yaw = x(6), r = x(8);
    double xa = xc - r * cos(yaw);
    double ya = yc - r * sin(yaw);
    return Eigen::Vector3d(xa, ya, za);
  }
  void Tracker::updateArmorsNum(const Armor &armor)
  {
    if (armor.type == "large" && (tracked_id == "3" || tracked_id == "4" || tracked_id == "5"))
    {
      tracked_armors_num = ArmorsNum::BALANCE_2;
    }
    else if (tracked_id == "outpost")
    {
      tracked_armors_num = ArmorsNum::OUTPOST_3;
    }
    else
    {
      tracked_armors_num = ArmorsNum::NORMAL_4;
    }
  }

} // namespace fyt::auto_aim
