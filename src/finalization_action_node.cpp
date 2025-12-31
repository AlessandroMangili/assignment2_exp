
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;

class Finalization : public plansys2::ActionExecutorClient {
    public:
        Finalization(): plansys2::ActionExecutorClient("finalization", 1s) {
            progress_ = 0.0;
        }

    private:
        void do_work() override {
            finish(true, 1.0, "Checked completed");
            progress_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "END COMPLETED!");
        }

        float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Finalization>();

  node->set_parameter(rclcpp::Parameter("action_name", "finalization"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
