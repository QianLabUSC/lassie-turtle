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



   void lowerproxy::calculate_position(turtle &turtle_ )
    {
        /**
         * The motor positions must be converted from Radians to Turns
         * 
         * The controller interprets angular position in radians, but the ODrive
         * uses turns as its angular unit.
        */
        float axis0_pos = (-1.0f * turtle_.turtle_control.Leg_lf.axis0.motor_control_position + M0_OFFSET + (M_PI / 2)) / (2 * M_PI);
        float axis1_pos = (turtle_.turtle_control.Leg_lf.axis1.motor_control_position - (M_PI / 2) + M1_OFFSET) / (2 * M_PI);

        // clamp both motor angles to within 1 turn;
        // axis0_pos = fmodf(axis0_pos, 1.0f);
        // axis1_pos = fmodf(axis1_pos, 1.0f);
        
        // _count = _count + 1;
        //int sign = 0;

        // auto message_channel_0 = turtle_msgs::msg::SetInputPosition();
        // message_channel_0.can_channel = 0;
        // message_channel_0.axis = 0;
        // message_channel_0.input_position = axis0_pos;
        // //message_channel_0.input_position = sign ;
        // message_channel_0.vel_ff = 0;
        // message_channel_0.torque_ff = 0;

        auto message_channel_1 = turtle_msgs::msg::SetInputPosition();
        message_channel_1.can_channel = 1;
        message_channel_1.axis = 0;
        message_channel_1.input_position = axis1_pos;
        message_channel_1.vel_ff = 0;
        message_channel_1.torque_ff = 0;
        // instead of publiishing to ros topics, publish to local class
        turtle_.turtle_control.Leg_lf.axis0.set_input_position = message_channel_0;
        turtle_.turtle_control.Leg_lf.axis1.set_input_position = message_channel_1;
        // Position_publisher_channel_0->publish(message_channel_0);
        // Position_publisher_channel_1->publish(message_channel_1);
        
      
    }

} // namespace control
} // namespace turtle_namespace




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
}
} // namespace turtle_namespace
