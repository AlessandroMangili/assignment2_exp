
#include <cmath>
#include <memory>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"

#include "marker_service_pkg/srv/store_markers.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

double quaternion_to_yaw(double x, double y, double z, double w) {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

class Rotation : public plansys2::ActionExecutorClient {
    public:
        Rotation(): plansys2::ActionExecutorClient("rotation", 100ms) {
            progress_ = 0.0;
            yaw = std::numeric_limits<float>::quiet_NaN();
            request_sent = true;
            
            vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 
                100, 
                std::bind(&Rotation::odom_callback, this, _1)
            );
            markers_sub = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
                "/aruco_detections",
                100,
                std::bind(&Rotation::marker_detection, this, _1)
            );
            store_client = this->create_client<marker_service_pkg::srv::StoreMarkers>(
                "/store_markers"
            );
        }

    private:
        float progress_;
        float yaw;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
        rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr markers_sub;
        std::string waypoint;
        
        rclcpp::Client<marker_service_pkg::srv::StoreMarkers>::SharedPtr store_client;
        bool request_sent;


        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            const auto & q = msg->pose.pose.orientation;
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w);
        }

        void marker_detection(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg) {
            if (msg->markers.empty() || waypoint.empty()) {
                return;
            }

            for (const auto & marker : msg->markers) {
                int marker_id = static_cast<int>(marker.marker_id);

                //RCLCPP_INFO(this->get_logger(), "Marker id %d", marker_id);    

                if (!this->request_sent) {
                    auto request = std::make_shared<marker_service_pkg::srv::StoreMarkers::Request>();
                    request->marker_id = marker_id;
                    request->marker = waypoint;

                    try {
                        this->request_sent = true;
                        auto result_future = store_client->async_send_request(request,
                            [this](rclcpp::Client<marker_service_pkg::srv::StoreMarkers>::SharedFuture response) {
                                RCLCPP_INFO(this->get_logger(), "Service request received");
                            });
                    } catch (const std::exception & e) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to call store service: %s", e.what());
                        this->request_sent = false;
                    }   
                }
            }
        }
        
        void do_work() override {   
            static bool rotating = false;               // STATIC fa in modo di far rimanere la variabile tra una chiamata e l'altra
            static float cumulative_rotation = 0.0;
            static float last_yaw = 0.0;
            static geometry_msgs::msg::Twist twist;
            
            auto args = get_arguments();
            if (args.size() < 2) {
                RCLCPP_ERROR(get_logger(), "Not enough arguments for move action");
                finish(false, 0.0, "Insufficient arguments");
                return;
            }
            // To reset and allow to send the makrker id each time the robot approches a new marker
            if (this->waypoint != args[1]) {
                this->request_sent = false;
            }
            this->waypoint = args[1];

            if (std::isnan(yaw)) {
                RCLCPP_INFO(this->get_logger(), "Waiting for odom");
                send_feedback(0.0, "Waiting for odom");
                return;
            }

            if (!rotating) {
                rotating = true;
                cumulative_rotation = 0.0;
                last_yaw = yaw;
                twist.angular.z = 0.5;
                RCLCPP_INFO(this->get_logger(), "Rotation over waypoint %s started", this->waypoint.c_str());
            }
            vel_pub->publish(twist);

            float delta = normalize_angle(yaw - last_yaw);
            cumulative_rotation += std::abs(delta);
            last_yaw = yaw;

            progress_ = std::min(1.0f, cumulative_rotation / float(2.0 * M_PI));
            //progress_ = std::round(progress_ * 100.0f) / 100.0f;
            send_feedback(progress_, "Rotating for id scanning over the waypoint");
            if (cumulative_rotation >= 2.0 * M_PI) {
                twist.angular.z = 0.0;
                vel_pub->publish(twist);

                rotating = false;
                cumulative_rotation = 0.0;
                progress_ = 0.0;

                if (!this->request_sent) {
                    RCLCPP_INFO(this->get_logger(), "Rotation over the waypoint %s for id scanning not completed", waypoint.c_str());
                    finish(false, 0.0, "Rotation for id scanning not completed");
                    return;
                }
                
                RCLCPP_INFO(this->get_logger(), "Rotation over the waypoint %s for id scanning completed", waypoint.c_str());
                finish(true, 1.0, "Rotation completed");
            }
        }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Rotation>();

    node->set_parameter(rclcpp::Parameter("action_name", "rotation"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    //node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
