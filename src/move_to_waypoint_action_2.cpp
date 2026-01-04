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
#include <map>

#include "marker_service_pkg/srv/store_markers.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

const bool DEBUG = false;

class MoveAction : public plansys2::ActionExecutorClient {
    public:
        MoveAction() : plansys2::ActionExecutorClient("move_to_waypoint_2", 250ms) {
            nav2_node = rclcpp::Node::make_shared("move_action_nav2_client");
            nav2_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(nav2_node, "navigate_to_pose");
            progress_ = 0.0;
            goal_sent = false;
            markers.clear();

            pos_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/amcl_pose",
                1000,
                std::bind(&MoveAction::current_pos_callback, this, _1)
            );

            store_client = this->create_client<marker_service_pkg::srv::StoreMarkers>(
                "/store_markers"
            );
        }
        
    private:
        float progress_;
        bool goal_sent;
        double start_x = 0.0, start_y = 0.0;
        std::map<int32_t, std::string> markers;
        rclcpp::Node::SharedPtr nav2_node;
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub;
        geometry_msgs::msg::Pose current_pos;
        rclcpp::Client<marker_service_pkg::srv::StoreMarkers>::SharedPtr store_client;
        std::string wp_to_navigate;

        void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            current_pos = msg->pose.pose;
        }

        void do_work() override {
            //cv::waitKey(1);
            auto args = get_arguments();
            if (args.size() < 3) {
                RCLCPP_WARN(get_logger(), "Not enough arguments for move action");
                finish(false, 0.0, "Insufficient arguments");
                return;
            }

            static bool get_markers = false;
            if (!get_markers) {
                if (!store_client->wait_for_service(2s)) {
                    RCLCPP_ERROR(get_logger(), "Service /store_markers not available");
                    return;
                }

                auto request = std::make_shared<marker_service_pkg::srv::StoreMarkers::Request>();
                request->marker_id = -1;
                store_client->async_send_request(request,
                    [this](rclcpp::Client<marker_service_pkg::srv::StoreMarkers>::SharedFuture future) {
                        auto response = future.get();
                        for (size_t i = 0; i < response->markers_id.size(); ++i) {
                            markers[response->markers_id[i]] = response->markers[i];
                        }
                        RCLCPP_INFO(get_logger(), "Service's response received successfully");
                        return;
                    });
                get_markers = true;
            }

            if (markers.empty()) {
                if (DEBUG) RCLCPP_WARN(get_logger(), "Failed to get the lowest marker id due to an empty waypoint list");
                send_feedback(0.0, "Waiting for service response");
                return;
            }

            static double goal_x, goal_y;   
            if (!goal_sent) {
                auto item = markers.begin(); 
                RCLCPP_INFO(get_logger(), "Moving to waypoint %s - %d", item->second.c_str(), item->first);
                wp_to_navigate = item->second;

                if (!nav2_client->wait_for_action_server(1s)) {
                    RCLCPP_WARN(get_logger(), "NavigateToPose server not ready");
                    send_feedback(0.0, "Waiting for NavigateToPose server");
                    return;
                }

                if (wp_to_navigate == "wp1") {
                    goal_x = -6.8;
                    goal_y = -8.0;
                } else if (wp_to_navigate == "wp2") {
                    goal_x = -6.0;
                    goal_y = 6.0;
                } else if (wp_to_navigate == "wp3") {
                    goal_x = 6.5;
                    goal_y = -8.0;
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

                auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
                goal_msg.pose = goal_pose;

                rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;
                send_goal_options.result_callback =
                    [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
                        switch (result.code) {
                            case rclcpp_action::ResultCode::SUCCEEDED:
                                goal_sent = false;
                                progress_ = 1.0;
                                markers.erase(markers.begin());
                                send_feedback(progress_, "Moving to " + wp_to_navigate);
                                RCLCPP_INFO(get_logger(), "Waypoint %s reached", this->wp_to_navigate.c_str());
                                finish(true, 1.0, "Navigation completed");
                                break;
                            case rclcpp_action::ResultCode::ABORTED:
                                goal_sent = false;
                                RCLCPP_ERROR(get_logger(), "Navigation to %s failed", this->wp_to_navigate.c_str());
                                finish(false, 0.0, "Navigation failed");
                                break;
                            default:
                                goal_sent = false;
                                RCLCPP_ERROR(get_logger(), "Navigation returned unknown state");
                                finish(false, 0.0, "Navigation returned unknown state");
                                break;
                        }
                        return;                  
                    };

                nav2_client->async_send_goal(goal_msg, send_goal_options);
                goal_sent = true;

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
                markers.erase(markers.begin());
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

    node->set_parameter(rclcpp::Parameter("action_name", "move_to_waypoint_2"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    //node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();

    return 0;
}