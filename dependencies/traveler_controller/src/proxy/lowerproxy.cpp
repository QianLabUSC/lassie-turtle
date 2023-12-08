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
    // create a list of low level position command publisher 
    joint0_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/joint0_position_controller/commands", 10);
    joint1_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/joint1_position_controller/commands", 10);
    // replace previous servo topics with current direct drive topic
    servo0_publisher_dd = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/servo0_controller/commands", 10);
    servo1_publisher_dd = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/servo1_controller/commands", 10);

    servo0_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/traveler_servo_big_1", 10);
    servo1_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/traveler_servo_big_2", 10);
    servo2_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/traveler_servo_small_1", 10);
    servo3_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/traveler_servo_small_2", 10);
    controller_state_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/controller", 10);
    // Robot_state_publisher = this->create_publisher<travelermsgs::msg::RobotState>
    //     ("/robot_state", 10);
    // Leg1_subscriber = this->create_subscription
    //                     <control_msgs::msg::DynamicJointState>
    //     ("/dynamic_joint_states", 10, std::bind(&lowerproxy::handle_joint_state, this, _1));


    _count = 0;

    // RCLCPP_INFO(this->get_logger(), "Publisher created!!");                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
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
                t = t + 0.001;
                t2=0;
                std::cout<<"AAAAAAAAAAbbbbA"<<t<<std::endl;
                // turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = 0; 
                // turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = 0; 
                // turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = 0; 
                // turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = 0;
                // update the current state
                 boundingGAIT(turtle_, t);
                //float channel0_axis0_pos = turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position; 
                //float channel0_axis1_pos = turtle_.turtle_control.right_adduction.set_input_position_degree.input_position; 
                //float channel1_axis0_pos = turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position; 
               // float channel1_axis1_pos = turtle_.turtle_control.left_adduction.set_input_position_degree.input_position; 
            
                // cout << theta1 <<" "<< gamma1 <<" "<< beta1 <<" "<< theta2 <<" "<< gamma2 <<" "<< beta2 << endl;


                // turtle_.turtle_control.left_limb_motor.motor_control_position = theta1;
                // turtle_.turtle_control.left_big_servo_command = gamma1;
                // turtle_.turtle_control.left_small_servo_command = beta1;

                // turtle_.turtle_control.right_limb_motor.motor_control_position = theta2;
                // turtle_.turtle_control.right_big_servo_command = gamma2;
                // turtle_.turtle_control.right_small_servo_command = beta2;

                // // update the current state
                // turtle_.turtle_chassis.left_big_servo_pos = gamma1;
                // turtle_.turtle_chassis.left_small_servo_pos = beta1;
                // turtle_.turtle_chassis.right_big_servo_pos = gamma2;
                // turtle_.turtle_chassis.right_small_servo_pos = beta2;

                //update angle when GUI stop
                saved_left_adduction=turtle_.turtle_chassis.left_adduction.pos_estimate;
                saved_left_sweeping= turtle_.turtle_chassis.left_sweeping.pos_estimate;
                saved_right_adduction= turtle_.turtle_chassis.right_adduction.pos_estimate;
                saved_right_sweeping= turtle_.turtle_chassis.right_sweeping.pos_estimate;
            }
            else{
                t = 0;
                t2=t2+0.001;
                std::cout<<"AAAAAAAAAAbbbbAelse"<<t;
                goback2desiredangle(turtle_,0,0,0,0,t2,10);
                //turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = 0; 
               // turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = 0; 
              //  turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = 0; 
                //turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = 0;
                // update the current state
                // turtle_.turtle_chassis.left_big_servo_pos = 0;
                // turtle_.turtle_chassis.left_small_servo_pos = 0;
                // turtle_.turtle_chassis.right_big_servo_pos = 0;
                // turtle_.turtle_chassis.right_small_servo_pos = 0;
            }
        
      
    }

void lowerproxy::goback2desiredangle(turtle& turtle_, float left_adduction, float left_sweeping, float right_adduction, float right_sweeping,float t_decrease_time,float total_time)
{
    if( t_decrease_time>total_time)
   {
     t_decrease_time=total_time;
      turtle_.turtle_control.left_adduction.set_input_position_radian.input_position= saved_left_adduction+ (left_adduction-saved_left_adduction);
      turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position  =saved_left_sweeping+ (left_sweeping-saved_left_sweeping);
      turtle_.turtle_control.right_adduction.set_input_position_radian.input_position=saved_right_adduction + (right_adduction-saved_right_adduction);
         turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position= saved_right_sweeping+(right_sweeping-saved_right_sweeping);
  
   }
   else
   {

    //left_adduciton should be in turns(unit)
    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position= saved_left_adduction+ (left_adduction-saved_left_adduction)*(t_decrease_time/total_time);
      turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position  =saved_left_sweeping+ (left_sweeping-saved_left_sweeping)*(t_decrease_time/total_time);
      turtle_.turtle_control.right_adduction.set_input_position_radian.input_position=saved_right_adduction + (right_adduction-saved_right_adduction)*(t_decrease_time/total_time);
         turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position= saved_right_sweeping+(right_sweeping-saved_right_sweeping)*(t_decrease_time/total_time);
   }
}

void lowerproxy::Estop(){

}

void lowerproxy::UpdateJoystickStatus(turtle& turtle_){
    turtle_.turtle_control = turtle_inter_.turtle_control;
    turtle_.turtle_gui = turtle_inter_.turtle_gui;

    // instead of reading message from ros2, directly call the function 
    // in odrivepro drive to get the message

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

    
    
}
    
} //namespace control
} //namespace turtle_namespace



