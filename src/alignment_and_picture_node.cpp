
#include <cmath>
#include <memory>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "marker_service_pkg/srv/store_markers.hpp"
#include <map>

using namespace std::chrono_literals;

using std::placeholders::_1;

double quaternion_to_yaw(double x, double y, double z, double w) {
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

bool isCenteredPixel(const cv::Point2f &center, const cv::Mat &img, int pixel_thresh = 9) {
    int cx_img = img.cols / 2;
    float dx = center.x - cx_img;
    return std::abs(dx) <= pixel_thresh;
}

class Alignment : public plansys2::ActionExecutorClient {
    public:
        Alignment(): plansys2::ActionExecutorClient("alignment_and_picture", 100ms) {
            progress_ = 0.0;
            yaw = std::numeric_limits<float>::quiet_NaN();  
            get_markers = false;

            vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            image_pub = this->create_publisher<sensor_msgs::msg::Image>("/circled_markers", 10);
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 
                100, 
                std::bind(&Alignment::odom_callback, this, _1)
            );
            image_sub = this->create_subscription<sensor_msgs::msg::Image>(
                "camera/image",
                100,
                std::bind(&Alignment::image_callback, this, _1)
            );
            store_client = this->create_client<marker_service_pkg::srv::StoreMarkers>(
                "/store_markers"
            );
        }

    private:
        float progress_;
        float yaw;
        const int REQUIRED_CONSECUTIVE = 2;
        bool get_markers;
        std::map<int, int> centered_counts;
        cv::Mat cv_image;
        std_msgs::msg::Header header;
        std::map<int32_t, std::string> markers;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
        rclcpp::Client<marker_service_pkg::srv::StoreMarkers>::SharedPtr store_client;


        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            const auto & q = msg->pose.pose.orientation;
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w);
        }

        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            try {
                cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
                header = msg->header;
            } catch (const cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Exception raised by the image conversion: %s", e.what());
            }
        }

        void drawCircle(int marker_id) {
            if (cv_image.empty()) {
                RCLCPP_WARN(this->get_logger(), "No image has been saved");
                return;
            }

            try {
                cv::namedWindow("Marker-" + std::to_string(marker_id), cv::WINDOW_AUTOSIZE);

                // ArUco dictionary e parametri
                cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
                cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

                // detectMarkers
                std::vector<std::vector<cv::Point2f>> corners, rejected;
                std::vector<int> ids;
                cv::aruco::detectMarkers(cv_image, aruco_dict, corners, ids, parameters, rejected);

                if (ids.empty()) {
                    RCLCPP_WARN(this->get_logger(), "No aruco markers detected!");
                } else {
                    size_t n = std::min(ids.size(), corners.size());
                    for (size_t i = 0; i < n; ++i) {
                        int current_marker_id = ids[i];

                        cv::Point2f center(0,0);
                        for (const auto &p : corners[i]) center += p;
                        center *= (1.0f / corners[i].size());

                        float radius_f = 0.0f;
                        for (const auto &p : corners[i]) {
                            float dist = cv::norm(p - center);
                            if (dist > radius_f) radius_f = dist;
                        }
                        int radius = static_cast<int>(std::ceil(radius_f));
                        cv::Point center_int(static_cast<int>(std::round(center.x)), static_cast<int>(std::round(center.y)));

                        cv::circle(cv_image, center_int, radius, cv::Scalar(0,0,255), 2);
                        cv::circle(cv_image, center_int, 3, cv::Scalar(0,0,255), -1);
                        cv::putText(cv_image, "marker_id=" + std::to_string(current_marker_id),
                                    cv::Point(center_int.x, center_int.y + 40),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2, cv::LINE_AA);
                    }
                }
                cv::imshow("Marker-" + std::to_string(marker_id), cv_image);
                cv::waitKey(20);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Exception raised by drawCircle: %s", e.what());
                return;
            }

            try {
                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image).toImageMsg();
                msg->header = header;
                image_pub->publish(*msg);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Exception while converting image: %s", e.what());
                return;
            }
        }

        bool centerMarker(int marker_id) {
            try {
                cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
                cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

                std::vector<std::vector<cv::Point2f>> corners;
                std::vector<int> ids;
                std::vector<std::vector<cv::Point2f>> rejected;

                cv::aruco::detectMarkers(this->cv_image, aruco_dict, corners, ids, parameters, rejected);

                if (ids.empty()) {
                    RCLCPP_WARN(this->get_logger(), "No corners have been detected");
                    return false;
                }

                size_t n = std::min(ids.size(), corners.size());
                for (size_t i = 0; i < n; ++i) {
                    int actual_marker_id = ids[i];
                    if (marker_id != actual_marker_id) {
                        RCLCPP_WARN(this->get_logger(), "Marker id %d does not match target %d", actual_marker_id, marker_id);
                        continue;
                    }

                    const auto &pts = corners[i];
                    if (pts.size() < 4) {
                        RCLCPP_WARN(this->get_logger(), "Invalid corner size for marker %d", actual_marker_id);
                        continue;
                    }

                    cv::Point2f center(0.0f, 0.0f);
                    for (const auto &p : pts) center += p;
                    center *= (1.0f / static_cast<float>(pts.size()));

                    bool centered_px = isCenteredPixel(center, this->cv_image, 9);
                    if (centered_px) {
                        this->centered_counts[actual_marker_id] = this->centered_counts[actual_marker_id] + 1;
                        RCLCPP_INFO(this->get_logger(), "Frames %d/%d centered for marker: %d", this->centered_counts[actual_marker_id], this->REQUIRED_CONSECUTIVE, actual_marker_id);
                    } else {
                        this->centered_counts[actual_marker_id] = 0;
                        RCLCPP_WARN(this->get_logger(), "Frame not centered for marker: %d", actual_marker_id);
                    }

                    if (this->centered_counts[actual_marker_id] >= this->REQUIRED_CONSECUTIVE) {
                        RCLCPP_INFO(this->get_logger(), "Marker %d CENTERED", actual_marker_id);
                        cv::namedWindow(std::string("Marker-") + std::to_string(actual_marker_id) + "-p", cv::WINDOW_AUTOSIZE);
                        cv::imshow(std::string("Marker-") + std::to_string(actual_marker_id) + "-p", this->cv_image);
                        cv::waitKey(20);
                        return true;
                    }
                }
                return false;
            } catch (const std::exception &e) {
                RCLCPP_WARN(this->get_logger(), "Exception raised by the set marker center: %s", e.what());
                return false;
            }
        }
        
        void do_work() override {
            cv::waitKey(1);
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
                        RCLCPP_INFO(get_logger(), "Store markers completato");
                        return;
                    });
                get_markers = true;
            }

            auto args = get_arguments();
            if (args.size() < 2) {
                RCLCPP_ERROR(get_logger(), "Not enough arguments for alignment");
                finish(false, 0.0, "Insufficient arguments");
                return;
            }

            if (markers.empty()) {
                RCLCPP_WARN(get_logger(), "Failed to get the lowest marker id");
                return;
            }

            static bool rotating = false;               // STATIC fa in modo di far rimanere la variabile tra una chiamata e l'altra
            static float cumulative_rotation = 0.0;
            static float last_yaw = 0.0;
            static geometry_msgs::msg::Twist twist;
            static int marker_id = -1;

            if (std::isnan(this->yaw)) {
                RCLCPP_INFO(this->get_logger(), "Waiting for odom");
                send_feedback(0.0, "Waiting for odom");
                return;
            }

            if (!rotating) {
                auto item = markers.begin(); 
                marker_id = item->first;
                RCLCPP_INFO(this->get_logger(), "MMARKER ID:%d", marker_id);
                rotating = true;
                cumulative_rotation = 0.0;
                last_yaw = this->yaw;
                RCLCPP_INFO(this->get_logger(), "Alignment started");
            }
            twist.angular.z = 0.25;
            vel_pub->publish(twist);

            float delta = normalize_angle(this->yaw - last_yaw);
            cumulative_rotation += std::abs(delta);
            last_yaw = this->yaw;

            progress_ = std::min(1.0f, cumulative_rotation / float(2.0 * M_PI));
            send_feedback(progress_, "Rotating");

            if (centerMarker(marker_id)) {
                twist.angular.z = 0.0;
                vel_pub->publish(twist);

                drawCircle(marker_id);
                
                markers.erase(markers.begin());
                rotating = false;
                cumulative_rotation = 0.0;
                progress_ = 0.0;
                
                RCLCPP_INFO(this->get_logger(), "Alignment completed!");
                finish(true, 1.0, "Alignment completed");
            }
        }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Alignment>();

    node->set_parameter(rclcpp::Parameter("action_name", "alignment_and_picture"));
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
