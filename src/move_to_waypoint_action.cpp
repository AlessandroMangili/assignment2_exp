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

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MoveAction : public plansys2::ActionExecutorClient {
    public:
        MoveAction() : plansys2::ActionExecutorClient("move_to_waypoint", 250ms) {
            nav2_node = rclcpp::Node::make_shared("move_action_nav2_client");
            nav2_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(nav2_node, "navigate_to_pose");
            
            progress_ = 0.0;
            goal_sent = false;
            
            /*odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 
                100,
                std::bind(&MoveAction::odom_callback, this, _1)
            );*/

            pos_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/amcl_pose",
                1000,
                std::bind(&MoveAction::current_pos_callback, this, _1)
            );
        }

    private:
        float progress_;
        bool goal_sent;
        double start_x = 0.0, start_y = 0.0;
        //double current_x_ = 0.0, current_y_ = 0.0;

        rclcpp::Node::SharedPtr nav2_node;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client;
        
        //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub;
        geometry_msgs::msg::Pose current_pos;
        std::string wp_to_navigate;

        void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            current_pos = msg->pose.pose;
        }

        /*void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_x_ = msg->pose.pose.position.x;
            current_y_ = msg->pose.pose.position.y;
        }*/

        void do_work() override {
            auto args = get_arguments();
            if (args.size() < 3) {
                RCLCPP_ERROR(get_logger(), "Not enough arguments for move action");
                finish(false, 0.0, "Insufficient arguments");
                return;
            }

            static double goal_x, goal_y;
            if (!goal_sent) {
                wp_to_navigate = args[2];
                
                if (!nav2_client->wait_for_action_server(1s)) {
                    RCLCPP_WARN(get_logger(), "NavigateToPose server not ready");
                    return;
                }

                if (wp_to_navigate == "wp1") {
                    goal_x = -6.8;
                    goal_y = -8.0;
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

                geometry_msgs::msg::PoseStamped goal_pose;
                goal_pose.header.frame_id = "map";
                goal_pose.pose.position.x = goal_x;
                goal_pose.pose.position.y = goal_y;
                goal_pose.pose.orientation.w = 1.0;

                RCLCPP_INFO(get_logger(), "Moving to waypoint: %s", wp_to_navigate.c_str());

                auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
                goal_msg.pose = goal_pose;

                rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;
                send_goal_options.result_callback =
                    [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
                    {
                        switch (result.code) {
                            case rclcpp_action::ResultCode::SUCCEEDED:
                                goal_sent = false;
                                progress_ = 1.0;
                                send_feedback(progress_, "Moving to " + wp_to_navigate);
                                RCLCPP_INFO(get_logger(), "Reached waypoint: %s", this->wp_to_navigate.c_str());
                                finish(true, 1.0, "Navigation completed");
                                break;
                            case rclcpp_action::ResultCode::ABORTED:
                                RCLCPP_WARN(get_logger(), "Nav2 aborted goal, waiting distance check");
                                goal_sent = false;
                                RCLCPP_ERROR(get_logger(), "Navigation failed: %s", this->wp_to_navigate.c_str());
                                finish(false, 0.0, "Navigation failed");
                                break;
                            default:
                                RCLCPP_WARN(get_logger(), "Nav2 returned unknown state");
                                goal_sent = false;
                                RCLCPP_ERROR(get_logger(), "Navigation failed: %s", this->wp_to_navigate.c_str());
                                finish(false, 0.0, "Navigation failed");
                                break;
                        }
                        return;   
                    };

                nav2_client->async_send_goal(goal_msg, send_goal_options);
                goal_sent = true;

                //start_x = current_x_;
                //start_y = current_y_;
                start_x = current_pos.position.x;
                start_y = current_pos.position.y;
            }

            double total_dist = std::hypot(goal_x - start_x, goal_y - start_y);
            double rem_dist   = std::hypot(goal_x - current_pos.position.x, goal_y - current_pos.position.y);
            progress_ = total_dist > 0.0 ? 1.0 - std::min(rem_dist / total_dist, 1.0) : 1.0;
            send_feedback(progress_, "Moving to " + wp_to_navigate);

            /*if (rem_dist < 0.25) {
                goal_sent= false;
                progress_ = 1.0;
                send_feedback(progress_, "Moving to " + wp_to_navigate);
                RCLCPP_INFO(get_logger(), "Reached waypoint: %s", wp_to_navigate.c_str());
                finish(true, 1.0, "Move completed");
            }*/
            rclcpp::spin_some(nav2_node);
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