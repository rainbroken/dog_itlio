#include "dog_driver/dog_driver_node.hpp"

#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dog_driver::DogDriverNode>());
  rclcpp::shutdown();
  return 0;
}
