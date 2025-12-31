
#include <cmath>
#include <memory>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"

using namespace std::chrono_literals;

class Rotation2 : public plansys2::ActionExecutorClient {
    public:
        Rotation2(): plansys2::ActionExecutorClient("rotation_2", 100ms) {
            /*odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 
                100, 
                std::bind(&Rotation2::odom_callback, this, std::placeholders::_1)
            );
            markers_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
                "/aruco_detections",
                100,
                std::bind(&Rotation2::marker_detection, this, std::placeholders::_1)
            );*/

            //vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            progress_ = 0.0;
            //yaw = std::numeric_limits<float>::quiet_NaN();  
            
            /*auto args = get_arguments();
            if (args.size() < 2) {
                RCLCPP_ERROR(get_logger(), "Not enough arguments for move action");
                finish(false, 0.0, "Insufficient arguments");
                return;
            }
            waypoint = args[1];
            std::cout << waypoint << std::endl;*/
        }

    private:
        float progress_;
        float yaw;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
        rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr markers_sub_;
        //std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;
        std::string waypoint;

        /*void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            const auto & q = msg->pose.pose.orientation;
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w);
        }

        double quaternion_to_yaw(double x, double y, double z, double w) {
            return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        }

        double normalize_angle(double angle) {
            return std::atan2(std::sin(angle), std::cos(angle));
        }*/

        /*void marker_detection(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg) {
            if (msg->markers.empty()) {
                return;
            }

            for (const auto & marker : msg->markers) {
                int marker_id = static_cast<int>(marker.marker_id);

                double px = marker.pose.position.x;
                double py = marker.pose.position.y;
                double pz = marker.pose.position.z;

                constexpr double EPS = 1e-6;
                if (std::abs(px) < EPS && std::abs(py) < EPS && std::abs(pz) < EPS) {
                    continue;
                }
            }
        }*/
        
        void do_work() override {
            /*static bool rotating = false;               // STATIC fa in modo di far rimanere la variabile tra una chiamata e l'altra
            static float cumulative_rotation = 0.0;
            static float last_yaw = 0.0;

            if (std::isnan(yaw)) {
                RCLCPP_INFO(this->get_logger(), "Waiting for odom");
                send_feedback(0.0, "Waiting for odom");
                return;
            }

            if (!rotating) {
                rotating = true;
                cumulative_rotation = 0.0;
                last_yaw = yaw;
                RCLCPP_INFO(this->get_logger(), "Rotation2 started");
            }

            geometry_msgs::msg::Twist twist;
            twist.angular.z = 0.5;
            vel_pub->publish(twist);

            float delta = normalize_angle(yaw - last_yaw);
            cumulative_rotation += std::abs(delta);
            last_yaw = yaw;

            progress_ = std::min(1.0f, cumulative_rotation / float(2.0 * M_PI));
            progress_ = std::round(progress_ * 100.0f) / 100.0f;
            send_feedback(progress_, "Rotating");
            if (cumulative_rotation >= 2.0 * M_PI) {
                twist.angular.z = 0.0;
                vel_pub->publish(twist);

                finish(true, 1.0, "Rotation2 completed");
                rotating = false;
                cumulative_rotation = 0.0;
                progress_ = 0.0;

                RCLCPP_INFO(this->get_logger(), "Rotation2 COMPLETED!");
            }*/
            progress_ = 1.0;
            send_feedback(progress_, "Rotating");
            finish(true, 1.0, "Rotation2 completed");
        }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Rotation2>();

    node->set_parameter(rclcpp::Parameter("action_name", "rotation_2"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
