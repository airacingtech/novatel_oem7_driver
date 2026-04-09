////////////////////////////////////////////////////////////////////////////////
//
// Standalone executable for novatel_oem7_driver.
// Constructs the node directly (not through rclcpp_components class_loader)
// to avoid the self-loading pluginlib issue where the node's internal
// ClassLoader can't find factories from its own library.
//
////////////////////////////////////////////////////////////////////////////////

#include <memory>
#include "rclcpp/rclcpp.hpp"

// Factory defined in oem7_message_node.cpp, avoids needing full class header
extern std::shared_ptr<rclcpp::Node> novatel_oem7_driver_create_node(
  const rclcpp::NodeOptions& options);

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = novatel_oem7_driver_create_node(rclcpp::NodeOptions{});

  static const size_t THREAD_NUM = 3;
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, THREAD_NUM);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
