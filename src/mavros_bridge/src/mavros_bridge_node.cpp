#include "rclcpp/rclcpp.hpp"
#include "mavros_bridge_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<uosm::mavros::MavrosBridgeComponent>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}