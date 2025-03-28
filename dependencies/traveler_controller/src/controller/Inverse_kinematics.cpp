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
    std::cout<<"Start to create the ros node subscriber and publisher" << std::endl;
   
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


/// @brief estimate terrain in this function
void lowerproxy::terrain_estimation(){
    // No modifications here for now.
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
    //float axis0_pos = (-1.0f * turtle_.turtle_control.Leg_lf.axis0.motor_control_position + M0_OFFSET + (M_PI / 2)) / (2 * M_PI);
    //float axis1_pos = (traveler_.traveler_control.Leg_lf.axis1.motor_control_position - (M_PI / 2) + M1_OFFSET) / (2 * M_PI);
        
 
   // clamp both motor angles to within 1 turn;
        

    // auto message_channel_0_axis0 = traveler_msgs::msg::SetInputPosition();
    // message_channel_0.can_channel = 0;
    // message_channel_0.axis = 0;
    // message_channel_0.input_position = axis0_pos;
    // //message_channel_0.input_position = sign ;
    // message_channel_0.vel_ff = 0;
    // message_channel_0.torque_ff = 0;

    // auto message_channel_1 = traveler_msgs::msg::SetInputPosition();
    // message_channel_1.can_channel = 1;
    // message_channel_1.axis = 0;
    // message_channel_1.input_position = axis1_pos;
    // message_channel_1.vel_ff = 0;
    // message_channel_1.torque_ff = 0;
    // // instead of publiishing to ros topics, publish to local class
    // turtle_.turtle_control.Leg_lf.axis0.set_input_position = message_channel_0;
    // turtle_.turtle_control.Leg_lf.axis1.set_input_position = message_channel_1;
    // Position_publisher_channel_0->publish(message_channel_0);
    // Position_publisher_channel_1->publish(message_channel_1);

        
        
    if(turtle_.turtle_gui.start_flag==1){
        t2 = std::chrono::high_resolution_clock::now();
        std::chrono::high_resolution_clock::time_point current_time_pre;
        std::chrono::duration<double> deltaTime_pre;

        std::chrono::high_resolution_clock::time_point current_time;
        std::chrono::duration<double> deltaTime;

        std::chrono::high_resolution_clock::time_point current_time1;
        std::chrono::duration<double> deltaTime1;
                
        double pre_t;
        double curr_initial_phase_time;
        double running_t;

        switch (currentState) {
            case ProgramState::FirstIteration:                       
                turtle_.turtle_chassis.gait_state = -1;
                // Additional code for first iteration
                std::cout << "First Interation: " << std::endl;
                starting_time = std::chrono::high_resolution_clock::now();
                currentState = ProgramState::SetToControlAndCalibrate;
                turtle_.turtle_control.if_control = false;
                break;

            case ProgramState::SetToControlAndCalibrate:
                current_time_pre = std::chrono::high_resolution_clock::now();
                deltaTime_pre = current_time_pre - starting_time;
                pre_t = deltaTime_pre.count();
                if(turtle_.turtle_chassis.if_idle_count <= 0 && pre_t > set_close_control_time){
                    currentState = ProgramState::GoToInitialPoint;
                }
                std::cout << "waiting Interation: " << std::endl;
                saved_left_adduction = turtle_.turtle_chassis.left_adduction.pos_estimate;
                saved_left_sweeping = turtle_.turtle_chassis.left_sweeping.pos_estimate;
                saved_right_adduction = turtle_.turtle_chassis.right_adduction.pos_estimate;
                saved_right_sweeping = turtle_.turtle_chassis.right_sweeping.pos_estimate; 
                std::cout << "saved_left_sweeping" << saved_left_sweeping << std::endl;
                std::cout << "saved_left_adduction" << saved_left_adduction << std::endl;
                std::cout << "saved_right_sweeping" << saved_right_sweeping << std::endl;
                std::cout << "saved_right_adduction" << saved_right_adduction << std::endl;
                turtle_.turtle_control.if_control = false;
                break;
                        

            case ProgramState::GoToInitialPoint:
                current_time = std::chrono::high_resolution_clock::now();
                deltaTime = current_time - starting_time;
                curr_initial_phase_time = deltaTime.count() - set_close_control_time;
                turtle_.turtle_chassis.gait_state = 0;
                turtle_.turtle_control.if_control = true;
                // MODIFIED: Interpolate from the current/saved positions to the user-specified start configuration.
                // The user-specified start configuration is given by start_gamma (gamma) and start_theta (theta) in radians.
                if (curr_initial_phase_time < initial_phase_time) {
                    goback2desiredangle(turtle_, 
                        turtle_.traj_data.start_gamma,          // MODIFIED: target gamma (right flipper)
                        turtle_.traj_data.start_theta,            // MODIFIED: target theta (right flipper)
                        turtle_.traj_data.start_gamma,            // using same for both sides if desired
                        turtle_.traj_data.start_theta,
                        saved_left_adduction, saved_left_sweeping,
                        saved_right_adduction, saved_right_sweeping,
                        curr_initial_phase_time, initial_phase_time);
                } else {
                    currentState = ProgramState::Running;
                }
                std::cout << "go to initial point Iteration: " << curr_initial_phase_time << std::endl;
                break;

            case ProgramState::Running:
                current_time1 = std::chrono::high_resolution_clock::now();
                deltaTime1 = current_time1 - starting_time;
                curr_initial_phase_time = deltaTime1.count();
                turtle_.turtle_control.if_control = true;
                running_t = curr_initial_phase_time - initial_phase_time - set_close_control_time;
                // MODIFIED: Call custom_trajectory to execute a cycle: from start to end and then back to start.
                custom_trajectory(turtle_, running_t); // MODIFIED: custom_trajectory function is added below.
                std::cout << "running Iteration: " << running_t << std::endl;
                break;
        }
    }
    else{
        turtle_.turtle_chassis.gait_state = 5;
        currentState = ProgramState::FirstIteration;
        turtle_.turtle_control.if_control = false;
        // probably not used 
        auto current_back_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> deltaTime = current_back_time - t2;
        double back_curr_time = deltaTime.count();
        turtle_.turtle_chassis.step_count = 0;
        // Optionally, call goback2desiredangle to return smoothly.
    }
}

void lowerproxy::goback2desiredangle(turtle& turtle_, float left_adduction, 
                                    float left_sweeping, float right_adduction,
                                    float right_sweeping,  float start_left_adduction, 
                                    float start_left_sweeping, float start_right_adduction,  
                                    float start_right_sweeping,  
                                    float t_decrease_time, float total_time)
{
    // right and left sweeping angle is no longer for use, the desired angle is determined by GUI theta range
    left_adduction = left_adduction/360;
    left_sweeping = left_sweeping/TWO_PI;
    right_adduction = right_adduction/360;
    right_sweeping = right_sweeping/TWO_PI;
    total_time = total_time/2;
    if( t_decrease_time > total_time)
    {
        t_decrease_time = total_time;
        turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = start_left_adduction + (left_adduction - start_left_adduction);
        turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position  = start_left_sweeping + (left_sweeping - start_left_sweeping);
        turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = start_right_adduction + (right_adduction - start_right_adduction);
        turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = start_right_sweeping + (right_sweeping - start_right_sweeping);
    }
    else
    {
        // left_adduction should be in turns (unit)
        turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = start_left_adduction + (left_adduction - start_left_adduction) * (t_decrease_time/total_time);
        turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position  = start_left_sweeping + (left_sweeping - start_left_sweeping) * (t_decrease_time/total_time);
        turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = start_right_adduction + (right_adduction - start_right_adduction) * (t_decrease_time/total_time);
        turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = start_right_sweeping + (right_sweeping - start_right_sweeping) * (t_decrease_time/total_time);
    }
}

void lowerproxy::Estop(){
    // Not implemented; placeholder.
}

void lowerproxy::UpdateJoystickStatus(turtle& turtle_){
    // instead of reading message from ros2, directly call the function 
    // in odrivepro drive to get the message
    // use the intermediate structure instead of the raw turtle_ to avoid messy
    turtle_inter_ = turtle_;

    // get raw encoder estimate from ODrive in unit of turns
    // convert to radians
    // auto odrive0 = turtle_inter_.turtle_chassis.left_adduction;
    // auto odrive1 = turtle_inter_.turtle_chassis.left_sweeping;
    // auto odrive2 = turtle_inter_.turtle_chassis.left_adduction;
    // auto odrive3 = turtle_inter_.turtle_chassis.left_sweeping;
    

    // clamp the position estimate from [0, 2pi]
    // pos_estimate_rad = fmodf_0_2pi(pos_estimate_rad);

    // need to modify here to get the status feedback
    // traveler_leg_.traveler_chassis.Leg_lf.axis0.effort = odrive0.iq_measured * TORQUE_CONST;
    // traveler_leg_.traveler_chassis.Leg_lf.axis0.position = 
    //     -1.0f * (odrive0.pos_estimate) * 2 * M_PI + M0_OFFSET + (M_PI/2);

    // traveler_leg_.traveler_chassis.Leg_lf.axis1.effort = odrive1.iq_measured * TORQUE_CONST;
    // traveler_leg_.traveler_chassis.Leg_lf.axis1.position =
    //     (odrive1.pos_estimate) * 2 * M_PI + (M_PI/2) - M1_OFFSET;


    // to be implemented
    terrain_estimation();

    // to publish information back to GUI.
    auto robot_state = std_msgs::msg::Float64MultiArray();
    robot_state.data.push_back(turtle_inter_.turtle_chassis.gait_state); // state flag
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_adduction.pos_estimate); // left adduction motor position status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_sweeping.pos_estimate);  // left sweeping motor position status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_adduction.pos_estimate); // right adduction motor position status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_sweeping.pos_estimate);  // right sweeping motor position status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_adduction.iq_setpoint);   // left adduction motor torque status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_sweeping.iq_setpoint);    // left sweeping motor torque status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_adduction.iq_setpoint);  // right adduction motor torque status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_sweeping.iq_setpoint);   // right sweeping motor torque status
    robot_state.data.push_back(turtle_inter_.turtle_control.left_adduction.set_input_position_radian.input_position);
    robot_state.data.push_back(turtle_inter_.turtle_control.left_sweeping.set_input_position_radian.input_position);
    robot_state.data.push_back(turtle_inter_.turtle_control.right_adduction.set_input_position_radian.input_position);
    robot_state.data.push_back(turtle_inter_.turtle_control.right_sweeping.set_input_position_radian.input_position);
    
    // add more information related to robot state here
    /*
    something important
    */
    
    controller_state_publisher->publish(robot_state);
}


// MODIFIED: Add custom_trajectory function to support trajectory between two user-specified points.
// This function is integrated with the inverse kinematics so that the flipper travels from
// the start point to the end point and then returns to the start point in one cycle.
void custom_trajectory(turtle& turtle_, float t)
{
    // Define phase durations (in seconds) for each part of the cycle.
    const float T1 = 3.0; // Phase 1: move from current position to the start point.
    const float T2 = 5.0; // Phase 2: move from the start point to the end point.
    const float T3 = 5.0; // Phase 3: move from the end point back to the start point.
    const float T_total = T1 + T2 + T3;
    
    float u;
    double desired_gamma, desired_theta;
    
    if(t < T1) {
        // Phase 1: Interpolate from current position to user-specified start.
        double current_gamma = 0.0;  // For now, assume current position is zero (or use saved values).
        double current_theta = 0.0;
        u = t / T1;
        desired_gamma = current_gamma + u * (turtle_.traj_data.start_gamma - current_gamma);
        desired_theta = current_theta + u * (turtle_.traj_data.start_theta - current_theta);
    }
    else if(t < T1 + T2) {
        // Phase 2: Move from start to end.
        u = (t - T1) / T2;
        desired_gamma = turtle_.traj_data.start_gamma + u * (turtle_.traj_data.end_gamma - turtle_.traj_data.start_gamma);
        desired_theta = turtle_.traj_data.start_theta + u * (turtle_.traj_data.end_theta - turtle_.traj_data.start_theta);
    }
    else if(t < T_total) {
        // Phase 3: Return from end to start.
        u = (t - T1 - T2) / T3;
        desired_gamma = turtle_.traj_data.end_gamma + u * (turtle_.traj_data.start_gamma - turtle_.traj_data.end_gamma);
        desired_theta = turtle_.traj_data.end_theta + u * (turtle_.traj_data.start_theta - turtle_.traj_data.end_theta);
    }
    else {
        // After the cycle, hold at the start point and stop the movement.
        desired_gamma = turtle_.traj_data.start_gamma;
        desired_theta = turtle_.traj_data.start_theta;
        turtle_.turtle_gui.start_flag = 0; // Signal to stop the trajectory.
    }
    
    // Command the right flipper with the computed desired angles.
    // Convert angles from radians to degrees for one interface.
    turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = desired_gamma * 180.0 / M_PI;
    turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = desired_theta * 180.0 / M_PI;
    turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = desired_gamma;
    turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = desired_theta;
    
    // Optionally, set the left flipper to an idle state (or mirror the right flipper if desired).
    turtle_.turtle_control.left_adduction.set_input_position_degree.input_position = 0;
    turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position = 0;
    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = 0;
    turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = 0;
    
    // Update the gait state to indicate custom trajectory execution.
    turtle_.turtle_chassis.gait_state = 9;
}

} // namespace control
} // namespace turtle_namespace



