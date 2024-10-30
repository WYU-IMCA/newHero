// std
#include <algorithm>
#include <deque>
#include <iostream>
#include <rm_utils/heartbeat.hpp>
#include <string>
#include <tuple>
#include <vector>
// ros2
#include <geometry_msgs/msg/point_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// third party
#include <opencv2/opencv.hpp>
// project
#include "rm_utils/heartbeat.hpp"
#include "rm_utils/common.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_auto_outpost/outpost_solver.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"

namespace imca::auto_outpost
{
    using tf2_filter = tf2_ros::MessageFilter<rm_interfaces::msg::Armors>;
    class AutoOutpostNode : public rclcpp::Node
    {
    public:
        explicit AutoOutpostNode(const rclcpp::NodeOptions &options);

    private:
        void armorsCallback(const rm_interfaces::msg::Armors::SharedPtr armors_ptr);

        void setModeCallback(const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
                             std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);

        // Outpost Solver
        std::unique_ptr<OutpostSolver> solver_;

        // Enable/Disable Outpost Solver
        bool enable_;
        rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;

        // Subscriber with tf2 message_filter
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
        message_filters::Subscriber<rm_interfaces::msg::Armors> armors_sub_;
        std::shared_ptr<tf2_filter> tf2_filter_;

        rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_receive_data_sub_;

        // Publisher
        rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;

        double IMU_yaw,IMU_pitch;
    };

} // namespace imca::outpost