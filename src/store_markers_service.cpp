#include <memory>
#include <vector>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "marker_service_pkg/srv/store_markers.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class MarkerService : public rclcpp::Node {
    public:
        MarkerService() : Node("store_markers_service")
        {
            service_ = this->create_service<marker_service_pkg::srv::StoreMarkers>(
            "store_markers",
            std::bind(&MarkerService::handle_service, this, _1, _2)
            );

            RCLCPP_INFO(this->get_logger(), "Store marker service is ready");
        }

    private:
    std::map<int32_t, std::string> markers;

    rclcpp::Service<marker_service_pkg::srv::StoreMarkers>::SharedPtr service_;

    void handle_service( const std::shared_ptr<marker_service_pkg::srv::StoreMarkers::Request> request, 
                        std::shared_ptr<marker_service_pkg::srv::StoreMarkers::Response> response) {
        auto result = markers.insert({request->marker_id, request->marker});

        if (!result.second) {
            RCLCPP_WARN(this->get_logger(), "Marker ID %d already exists. Ignoring request.", request->marker_id);
        } else {
            RCLCPP_INFO(this->get_logger(), "Stored marker_id=%d marker='%s' | Total stored=%zu", request->marker_id, request->marker.c_str(), markers.size());
        }

        response->markers_id.clear();
        response->markers.clear();

        for (const auto & pair : markers) {
            response->markers_id.push_back(pair.first);
            response->markers.push_back(pair.second);
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarkerService>());
    rclcpp::shutdown();
    return 0;
}
