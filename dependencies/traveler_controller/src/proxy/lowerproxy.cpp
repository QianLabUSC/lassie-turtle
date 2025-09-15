/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-21 21:58:00 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2022-02-02 18:31:48
 */

#include "proxy/lowerproxy.h"
#include "controller/inverse_kinematics.h"
#include <fstream>
#include <iostream>
#include <iomanip>

/**
 * lowerproxy - class to publish control command to webots turtle_namespace or real 
 * agile taur.
 */
const double PI = 3.141592653589793238463;
const double TWO_PI = PI*2;

namespace turtle_namespace{
namespace control{

lowerproxy::lowerproxy(std::string name) : Node(name){
    std::cout<<"Start to create the ros node subscriber and publisher"
                <<std::endl;
   
    controller_state_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/robot_state", 10);

    _count = 0;

    RCLCPP_INFO(this->get_logger(), "Publisher created!!");                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
}

float lowerproxy::fmodf_mpi_pi(float f)
{
  if (f>0)
    return (fmodf(f+PI, TWO_PI) - PI);
  else
    return (fmodf(f-PI, TWO_PI) + PI);
}

/// @brief calculate the motor command using different controller:
///        default: inverse kinematic controller
/// @param turtle
void lowerproxy::calculate_position(turtle &turtle_ )
{
    /**
     * The motor positions must be converted from Radians to Turns
     * 
     * The controller interprets angular position in radians, but the ODrive
     * uses turns as its angular unit.
    */
    
    // Get the motor control positions from the leg structure
    float left_adduction_pos = turtle_.turtle_control.Leg_lf.axis0.motor_control_position;
    float left_sweeping_pos = turtle_.turtle_control.Leg_lf.axis1.motor_control_position;
    
    // Set the input positions for the motor commands
    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = left_adduction_pos;
    turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = left_sweeping_pos;
    
    // Also update the motor_control_position fields in motor_command structures
    turtle_.turtle_control.left_adduction.motor_control_position = left_adduction_pos;
    turtle_.turtle_control.left_sweeping.motor_control_position = left_sweeping_pos;
}

void lowerproxy::goback2desiredangle(turtle& turtle_, float left_adduction, 
                                    float left_sweeping, float right_adduction,
                                    float right_sweeping,  float start_left_adduction, 
                                    float start_left_sweeping, float start_right_adduction,  
                                    float start_right_sweeping,  
                                    float t_decrease_time,float total_time)
{
    // Implementation for smooth transition to desired angles
    left_adduction = left_adduction/360.0f;
    left_sweeping = left_sweeping/TWO_PI;
    right_adduction = right_adduction/360.0f;
    right_sweeping = right_sweeping/TWO_PI;
    total_time = total_time/2.0f;
    
    if( t_decrease_time>total_time)
   {
     t_decrease_time=total_time;
      turtle_.turtle_control.left_adduction.set_input_position_radian.input_position= start_left_adduction+ (left_adduction-start_left_adduction);
      turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position  =start_left_sweeping+ (left_sweeping-start_left_sweeping);
      turtle_.turtle_control.right_adduction.set_input_position_radian.input_position=start_right_adduction + (right_adduction-start_right_adduction);
      turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position= start_right_sweeping+(right_sweeping-start_right_sweeping);
   }
   else
   {
    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position= start_left_adduction+ (left_adduction-start_left_adduction)*(t_decrease_time/total_time);
    turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position  =start_left_sweeping+ (left_sweeping-start_left_sweeping)*(t_decrease_time/total_time);
    turtle_.turtle_control.right_adduction.set_input_position_radian.input_position=start_right_adduction + (right_adduction-start_right_adduction)*(t_decrease_time/total_time);
    turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position= start_right_sweeping+(right_sweeping-start_right_sweeping)*(t_decrease_time/total_time);
   }
}

void lowerproxy::Estop(){
    // Emergency stop implementation
    std::cout << "E-STOP triggered" << std::endl;
}

void lowerproxy::UpdateJoystickStatus(turtle& turtle_){
    // instead of reading message from ros2, directly call the function 
    // in odrivepro drive to get the message
    // use the intermediate structure instead of the raw turtle_ to avoid messy
    turtle_inter_ = turtle_;

    // to publish information back to gui
    auto robot_state = std_msgs::msg::Float64MultiArray();
    robot_state.data.push_back(turtle_inter_.turtle_chassis.gait_state); // state flag
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_adduction.pos_estimate); //left adduction motor position status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_sweeping.pos_estimate); //left sweeping motor position status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_adduction.pos_estimate); //right adduction motor position status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_sweeping.pos_estimate); //right sweeping motor position status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_adduction.iq_setpoint); //left adduction motor torque status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_sweeping.iq_setpoint); //left sweeping motor torque status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_adduction.iq_setpoint); //right sweeping motor torque status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_sweeping.iq_setpoint); //left adduction motor torque status
    robot_state.data.push_back(turtle_inter_.turtle_control.left_adduction.set_input_position_radian.input_position);
    robot_state.data.push_back(turtle_inter_.turtle_control.left_sweeping.set_input_position_radian.input_position);
    robot_state.data.push_back(turtle_inter_.turtle_control.right_adduction.set_input_position_radian.input_position);
    robot_state.data.push_back(turtle_inter_.turtle_control.right_sweeping.set_input_position_radian.input_position);
    
    controller_state_publisher->publish(robot_state);
}

void lowerproxy::terrain_estimation(){
    // Implementation for terrain estimation
}

void lowerproxy::loadPrecomputed(const std::string &csv_path) {
    // Implementation for loading precomputed trajectories
    (void)csv_path; // Suppress unused parameter warning
}

void lowerproxy::handle_gui(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // Implementation for handling GUI messages
    (void)msg; // Suppress unused parameter warning
}

} //namespace control
} //namespace turtle_namespace

