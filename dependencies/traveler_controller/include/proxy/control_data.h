/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2022-02-02 16:17:20 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2022-02-02 18:58:03
 */

#ifndef DATA_H_
#define DATA_H_
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "traveler_msgs/msg/odrive_status.hpp"
#include "traveler_msgs/msg/set_input_position.hpp"
#include "traveler_msgs/msg/set_state.hpp"
struct motor_status{
    float error;
    float effort;
    float temperature;
    float position;
    float velocity;
    float toeforce;
    float toespeed;
};


// define turtle leg id vector
// left_adduction: 0
// left_sweeping: 1
// right_adduction: 2
// right_sweeping: 3

struct turtle_status
{
    traveler_msgs::msg::OdriveStatus left_adduction;
    traveler_msgs::msg::OdriveStatus left_sweeping;
    traveler_msgs::msg::OdriveStatus right_adduction;
    traveler_msgs::msg::OdriveStatus right_sweeping;
    // gait state flag
    float gait_state = 0; // 0: prepare, 1: backing, 2: penetrating, 3: penetrate, 4: shear, 5: stop
    // maximum idle/close_loop_control set count
    int if_idle_count = 1;
    
    
};


struct motor_command{
    traveler_msgs::msg::SetInputPosition set_input_position_degree;
    traveler_msgs::msg::SetInputPosition set_input_position_radian;
    traveler_msgs::msg::SetState set_state;
};

struct trutle_command{
    bool if_control;
    motor_command left_adduction;
    motor_command left_sweeping;
    motor_command right_adduction;
    motor_command right_sweeping;
     
};

struct human_interface{
    float drag_traj = 0;

    // Trajectory Start Flag (run state = true or false)
    int start_flag = 0;

    // unused
    bool status_update_flag = false;

    
};
// struct that defines the behavior of trajectories
struct TrajectoryData
{
    // Extrustion Trajectory Parameters
    float lateral_angle_range;      // arc             
    float drag_speed;               //m/s     
    float wiggle_time;                  //s

    float servo_speed;        //s
    float extraction_angle;    //arc     
    float wiggle_frequency;        //hz  
    float insertion_depth;             //arc        
    float wiggle_amptitude;                 //arc     
};

struct turtle{
    turtle_status turtle_chassis;
    trutle_command turtle_control;
    human_interface turtle_gui;
    TrajectoryData traj_data;
};

#endif