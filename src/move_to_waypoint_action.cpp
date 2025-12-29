#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include <memory>
#include <chrono>
#include <string>
#include <algorithm>
#include <cmath>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient {
    public:
        MoveAction() : plansys2::ActionExecutorClient("move_to_waypoint", 250ms) {
            odom = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 
                100,
                std::bind(&MoveAction::odom_callback, this, std::placeholders::_1)
            );
            nav2_node_ = rclcpp::Node::make_shared("move_action_nav2_client");
            nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(nav2_node_, "navigate_to_pose");

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            progress_ = 0.0;
            goal_sent_ = false;
        }

    private:
        float progress_;
        bool goal_sent_;
        double start_x_ = 0.0, start_y_ = 0.0;
        double current_x_ = 0.0, current_y_ = 0.0;

        rclcpp::Node::SharedPtr nav2_node_;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        void do_work() override {
            auto args = get_arguments();

            if (args.size() < 3) {
                RCLCPP_ERROR(get_logger(), "Not enough arguments for move action");
                finish(false, 0.0, "Insufficient arguments");
                return;
            }

            std::string wp_to_navigate = args[2];
            double goal_x, goal_y;
            if (wp_to_navigate == "wp1") {
                goal_x = -6.0;
                goal_y = -6.0;
            } else if (wp_to_navigate == "wp2") {
                goal_x = -6.0;
                goal_y = 6.0;
            } else if (wp_to_navigate == "wp3") {
                goal_x = 6.0;
                goal_y = -6.0;
            } else if (wp_to_navigate == "wp4") {
                goal_x = 6.0;
                goal_y = 6.0;
            } else {
                RCLCPP_ERROR(get_logger(), "Unknown waypoint: %s", wp_to_navigate.c_str());
                finish(false, 0.0, "Unknown waypoint");
                return;
            }

            if (!goal_sent_) {
                if (!nav2_client_->wait_for_action_server(1s)) {
                    RCLCPP_WARN(get_logger(), "NavigateToPose server not ready");
                    return;
                }

                geometry_msgs::msg::PoseStamped goal_pose;
                goal_pose.header.frame_id = "map";
                goal_pose.pose.position.x = goal_x;
                goal_pose.pose.position.y = goal_y;
                goal_pose.pose.orientation.w = 1.0;

                auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
                goal_msg.pose = goal_pose;

                rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;
                send_goal_options.result_callback =
                    [this, wp_to_navigate](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
                    {
                        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
                            RCLCPP_ERROR(get_logger(), "Navigation failed: %s", wp_to_navigate.c_str());
                            finish(true, 1.0, "Move failed");
                        } else if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                            goal_sent_= false;
                            progress_ = 1.0;
                            send_feedback(progress_, "Moving to " + wp_to_navigate);
                            RCLCPP_INFO(get_logger(), "Reached waypoint: %s", wp_to_navigate.c_str());
                            finish(true, 1.0, "Move completed");
                        }
                    };

                nav2_client_->async_send_goal(goal_msg, send_goal_options);
                goal_sent_ = true;

                //start_x_ = current_x_;
                //start_y_ = current_y_;
                get_current_pose_in_map(start_x_, start_y_);
            }

            std::cout << " GOAL " << goal_x <<  " Y " << goal_y << std::endl;
            std::cout << " CURRENT " << current_x_ <<  " Y " << current_y_ << std::endl;

            if (!get_current_pose_in_map(current_x_, current_y_)) {
                RCLCPP_WARN(get_logger(), "Cannot get current pose in map frame");
                return;
            }

            double total_dist = std::hypot(goal_x - start_x_, goal_y - start_y_);
            double rem_dist   = std::hypot(goal_x - current_x_, goal_y - current_y_);
            std::cout << " DISTANCE " << total_dist <<  " REM " << rem_dist << std::endl;
            progress_ = total_dist > 0.0 ? 1.0 - std::min(rem_dist / total_dist, 1.0) : 1.0;

            send_feedback(progress_, "Moving to " + wp_to_navigate);

            if (rem_dist < 0.2) {
                goal_sent_= false;
                progress_ = 1.0;
                send_feedback(progress_, "Moving to " + wp_to_navigate);
                RCLCPP_INFO(get_logger(), "Reached waypoint: %s", wp_to_navigate.c_str());
                finish(true, 1.0, "Move completed");
            }
            rclcpp::spin_some(nav2_node_);
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_x_ = msg->pose.pose.position.x;
            current_y_ = msg->pose.pose.position.y;
        }

        bool get_current_pose_in_map(double &x, double &y) {
            try {
                auto transformStamped = tf_buffer_->lookupTransform(
                    "map",       // target frame
                    "base_link", // source frame
                    rclcpp::Time(0),
                    1000ms
                );
                x = transformStamped.transform.translation.x;
                y = transformStamped.transform.translation.y;
                return true;
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(get_logger(), "Could not transform base_link to map: %s", ex.what());
                return false;
            }
        }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveAction>();

    node->set_parameter(rclcpp::Parameter("action_name", "move_to_waypoint"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;
}