////////////////////////////////////////////////////////////////////////////////
// Custom component container with a 3-thread MultiThreadedExecutor.
// The default component_container_mt uses 1 thread per CPU core which causes
// excessive idle CPU usage from executor polling.
////////////////////////////////////////////////////////////////////////////////

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  static const size_t THREAD_NUM = 3;
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions{}, THREAD_NUM);

  auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);
  exec->add_node(node);
  exec->spin();
  rclcpp::shutdown();
  return 0;
}
