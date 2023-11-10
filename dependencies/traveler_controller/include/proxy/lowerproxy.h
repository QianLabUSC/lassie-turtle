/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-21 22:01:05 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2022-02-02 18:14:21
 */

#ifndef LOWERPROXY_H_
#define LOWERPROXY_H_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "proxy/control_data.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "traveler_msgs/msg/odrive_status.hpp"
#include "traveler_msgs/msg/set_input_position.hpp"
// #include "travelermsgs/msg/robot_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
namespace turtle_namespace{
namespace control{

class lowerproxy:public rclcpp::Node{
  public:
    lowerproxy(std::string name = "lower_proxy");
    void PublishControlCommand(turtle &);
    void terrain_estimation();
    void calculate_position(turtle &);
    void Estop();
    void UpdateJoystickStatus(turtle &);
    float fmodf_mpi_pi(float);
  

  private:

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  joint0_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  joint1_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  servo0_publisher_dd;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  servo1_publisher_dd;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  servo0_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  servo1_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  servo2_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  servo3_publisher;
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr
                                                  Leg1_subscriber;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  controller_state_publisher; 
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr 
                                                  controller_state_publisher2;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
                                                  GUI_subscriber;                                                                                               
    rclcpp::TimerBase::SharedPtr _timer;
    float _count;
    turtle turtle_inter_;
    // void handle_joint_state(const control_msgs::msg::DynamicJointState::SharedPtr msg);
    void handle_gui(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

} //namespace control
} //namespace turtle_namespace



#endif
