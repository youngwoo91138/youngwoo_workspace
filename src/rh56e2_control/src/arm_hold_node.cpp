#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class ArmHoldNode : public rclcpp::Node
{
public:
  ArmHoldNode() : Node("arm_hold_node")
  {
    positions_ = declare_parameter<std::vector<double>>(
      "positions", {0.0, -0.51145, 0.50894, 0.0, 0.0, 1.53058});
    if (positions_.size() != 6)
    {
      throw std::invalid_argument("Arm hold requires six joint positions.");
    }
    for (double position : positions_)
    {
      if (!std::isfinite(position))
      {
        throw std::invalid_argument("Arm hold positions must be finite.");
      }
    }
    publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/ur5_hold_controller/commands", 10);
    timer_ = create_wall_timer(100ms, [this]() {
      if (publisher_->get_subscription_count() == 0)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Waiting for ur5_hold_controller. Check controller activation.");
        return;
      }
      std_msgs::msg::Float64MultiArray message;
      message.data = positions_;
      publisher_->publish(message);
    });
  }

private:
  std::vector<double> positions_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmHoldNode>());
  rclcpp::shutdown();
  return 0;
}
