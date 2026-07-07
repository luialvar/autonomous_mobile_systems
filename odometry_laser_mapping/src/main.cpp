#include "rclcpp/rclcpp.hpp"
#include "map_sim/map_sim.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<map_sim>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
