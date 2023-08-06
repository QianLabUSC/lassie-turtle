#include <cstdio>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
// #include "can_driver/can_service.hpp"
#include "can_driver/can_publisher.hpp"
#include "can_driver/can_suber.hpp"


int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  // auto server = std::make_shared<CanService>();
  // exec.add_node(server);

  auto publisher = std::make_shared<CanPublisher>();
  exec.add_node(publisher);

  auto suber = std::make_shared<CanSuber>();
  exec.add_node(suber);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
