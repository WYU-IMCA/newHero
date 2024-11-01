#include <memory>
#include <string>
#include <chrono>
#include <iostream>
// 3rd party
#include <Eigen/Dense>
#include "opencv2/core/core.hpp"
#include <opencv2/core/eigen.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
// project
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"
// ros2 
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace imca::auto_outpost
{

    class ArmorBlob{
    public:

        double x, y, z;
        // 重载小于号，用于set，按(x,y,z)到(0,0,0)的距离排序
        bool operator < (const ArmorBlob& a) const
        {
            return x * x + y * y + z * z < a.x * a.x + a.y * a.y + a.z * a.z;
        }

    };

    class OutpostSolver
    {
    public:
        explicit OutpostSolver();

        ~OutpostSolver() = default;

        rm_interfaces::msg::GimbalCmd solve(const rm_interfaces::msg::Armors::SharedPtr armors_msg,const double IMU_pitch,const double IMU_yaw);

        rm_interfaces::msg::GimbalCmd old_solve(const rm_interfaces::msg::Armors::SharedPtr armors_msg);

        void Init();

    private:
        double outpost_interval;
        double flying_time;
        bool have_detecte;
        bool get_center;
        bool markLast;
        bool waitingToshut;
        int lostInShootZone;
        double last_sleep_time;

        std::vector<ArmorBlob>armors_set;

        std::vector<std::chrono::steady_clock::time_point> time;
        std::vector<std::chrono::milliseconds> timeInZone;

        std::chrono::steady_clock::time_point start, end;

        std::unique_ptr<TrajectoryCompensator> trajectory_compensator_;

        double orientationToYaw(const geometry_msgs::msg::Quaternion &q) noexcept;

        void calcCameraYawAndPitch(const Eigen::Vector3d &p, double &yaw, double &pitch) const noexcept;

        float calculateDistanceToCenter(const cv::Point2f &image_point) const noexcept ;

        void calcYawAndPitch(const Eigen::Vector3d &p,
                             double &yaw,
                             double &pitch) const noexcept;

        bool AutoFireControl(const double TimeDetecteToNow);

        cv::Mat camera_matrix;
    };

}