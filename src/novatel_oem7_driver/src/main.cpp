////////////////////////////////////////////////////////////////////////////////
// Standalone executable for novatel_oem7_driver.
// Uses MultiThreadedExecutor (3 threads) matching original driver requirements.
////////////////////////////////////////////////////////////////////////////////

#include <memory>
#include "rclcpp/rclcpp.hpp"

// Defined in oem7_message_node.cpp, linked via the node component library.
namespace novatel_oem7_driver { class Oem7MessageNode; }

// Forward-declare the factory so we don't need the full node header.
extern std::shared_ptr<rclcpp::Node> create_oem7_node(const rclcpp::NodeOptions& options);

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto oem7 = create_oem7_node(rclcpp::NodeOptions{});

  static const size_t THREAD_NUM = 3; // Default + Receive and blocking command service.
  rclcpp::ExecutorOptions defaultOptions;
  rclcpp::executors::MultiThreadedExecutor executor(defaultOptions, THREAD_NUM);
  executor.add_node(oem7);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
