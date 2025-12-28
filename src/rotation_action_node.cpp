
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;

class Rotation : public plansys2::ActionExecutorClient {
    public:
        Rotation(): plansys2::ActionExecutorClient("rotation", 1s) {
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10,
                std::bind(&Rotation::odom_callback, this, std::placeholders::_1)
            );

            vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

            progress_ = 0.0;
        }

    private:
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            const auto & q = msg->pose.pose.orientation;
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w);
        }
        
        void do_work() override {
            /*geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.0;
            twist.angular.z = 0.25;
            double cumulative_rotation = 0.0;

            if (std::isnan(yaw)) {
                RCLCPP_WARN(get_logger(), "Waiting for odometry...");
                return;
            }

            double last_yaw = yaw;
                
            rclcpp::Rate rate(20);  // ~0.05 sec
            try {
                while (rclcpp::ok() && cumulative_rotation < (2.0 * M_PI)) {
                    vel_pub->publish(twist);
                    rclcpp::spin_some(this->get_node_base_interface());

                    if (!std::isnan(yaw)) {
                        double delta = normalize_angle(yaw - last_yaw);
                        cumulative_rotation += std::abs(delta);
                        last_yaw = yaw;
                    }

                    progress_ = std::min(1.0, cumulative_rotation / (2.0 * M_PI));
                    send_feedback(progress_, "Rotating");
                    rate.sleep();
                }
            }
            catch (const std::exception & e) {
                RCLCPP_WARN(this->get_logger(), "Interrupted during rotation.");
                return;
            }

            twist.angular.z = 0.0;
            vel_pub->publish(twist);*/

            finish(true, 1.0, "Rotation completed");
            progress_ = 0.0;

            RCLCPP_INFO(this->get_logger(), "SCANNING COMPLETED!");

            /*if (progress_ < 1.0) {
            progress_ += 0.05;
            send_feedback(progress_, "Rotation running");
            } else {
            finish(true, 1.0, "Rotation completed");

            progress_ = 0.0;
            std::cout << std::endl;
            }

            std::cout << "\r\e[K" << std::flush;
            std::cout << "Requesting for charging ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
            std::flush;*/
        }

        double quaternion_to_yaw(double x, double y, double z, double w) {
            return std::atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z));
        }

        double normalize_angle(double angle) {
            return std::atan2(std::sin(angle), std::cos(angle));
        }

        float progress_;
        double yaw = 0.0;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Rotation>();

  node->set_parameter(rclcpp::Parameter("action_name", "rotation"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);


  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
