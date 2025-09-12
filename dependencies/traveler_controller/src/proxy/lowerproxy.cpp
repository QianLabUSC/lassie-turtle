/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-21 21:58:00 
 * @Last Modified by: 
 * @Last Modified time: 2025-09-12
 */

#include "proxy/lowerproxy.h"
#include "controller/inverse_kinematics.h"

#include "proxy/lowerproxy.h"
#include "controller/inverse_kinematics.h"
#include <fstream>
#include <iostream>
#include <iomanip>

namespace turtle_namespace {
namespace control {

lowerproxy::lowerproxy(std::string name) : Node(name){
    std::cout<<"Start to create the ros node subscriber and publisher"
                <<std::endl;
   
    controller_state_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/robot_state", 10);
    

    _count = 0;

    RCLCPP_INFO(this->get_logger(), "Publisher created!!");                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
}



void lowerproxy::calculate_position(turtle &turtle_) {
    // float target_x = t.traj_data.desired_x;  
    // float target_y = t.traj_data.desired_y;

    // float theta, gamma;
    // physicalToAbstract(target_x, target_y, theta, gamma, true);

    // float m0 = theta - gamma;
    // float m1 = theta + gamma;

    // t.turtle_control.Leg_lf.axis0.motor_control_position = m0; // radians
    // t.turtle_control.Leg_lf.axis1.motor_control_position = m1;
}

void lowerproxy::UpdateJoystickStatus(turtle &turtle_) {
    // Mirror app state (what upper layers commanded + any sensed values)
    // back to the GUI. No ODrive access here.
    turtle_inter_ = turtle_;

    auto robot_state = std_msgs::msg::Float64MultiArray();
    robot_state.data.reserve(13);

    // State flag / gait state
    robot_state.data.push_back(turtle_inter_.turtle_chassis.gait_state);

    // Motor feedback (positions / torque setpoints as available in your struct)
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_adduction.pos_estimate);
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_sweeping.pos_estimate);
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_adduction.pos_estimate);
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_sweeping.pos_estimate);

    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_adduction.iq_setpoint);
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_sweeping.iq_setpoint);
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_adduction.iq_setpoint);
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_sweeping.iq_setpoint);

    // What we are commanding (in “turns”, per your existing interface)
    robot_state.data.push_back(turtle_inter_.turtle_control.left_adduction.set_input_position_radian.input_position);
    robot_state.data.push_back(turtle_inter_.turtle_control.left_sweeping.set_input_position_radian.input_position);
    robot_state.data.push_back(turtle_inter_.turtle_control.right_adduction.set_input_position_radian.input_position);
    robot_state.data.push_back(turtle_inter_.turtle_control.right_sweeping.set_input_position_radian.input_position);

    controller_state_publisher->publish(robot_state);
}

void lowerproxy::Estop() {
  
}

} // namespace control
} // namespace turtle_namespace
