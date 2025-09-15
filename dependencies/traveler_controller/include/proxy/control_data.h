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

#include <cmath>

const float L1 = 0.1f; // meters
const float L2 = 0.2f; // meters
const float L3 = 0.05f; //the length of leg extension


const float MIN_EXT = L2-L1+L3+0.005;
const float MAX_EXT = L2+L1+L3-0.01;


struct XY_pair{
    float x;
    float y;
    XY_pair() {
        x = 0.0f;
        y = 0.0f;
    }
    XY_pair(float x_, float y_) {
        x = x_;
        y = y_;
    } 
};

struct Waypoint {
    XY_pair point;
    float vel;
    float delay;

    Waypoint() {
        point = XY_pair();
        vel = 0.0f;
        delay = 0.0f;
    }
    Waypoint(XY_pair point_, float vel_, float delay_) {
        point = point_;
        vel = vel_;
        delay = delay_;
    }
    Waypoint(float x_, float y_, float vel_, float delay_) {
        point = XY_pair(x_, y_);
        vel = vel_;
        delay = delay_;
    }
};

struct Theta_L_pair {
    float theta;
    float L;
};

struct Point_Pair {
    XY_pair A;
    XY_pair B;
};


// // Prints roots of quadratic equation ax*2 + bx + x
// XY_pair findRoots(float a, float b, float c);

// Point_Pair findCircleIntercepts(XY_pair xvals, float m, float b);


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
    int step_count = 0;
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
    float sweeping_range;      // arc             
    float insertion_depth;               // m/s     
    float penetration_velocity;              // s

    float sweeping_velocity;              // s
    float extraction_velocity;         // arc     
    float swing_velocity;         // hz  
    // float insertion_depth;          // arc        
    // float wiggle_amptitude;         // arc  
    float servo_speed; 
    float lateral_angle_range; 
    float extraction_angle; 
    float drag_speed;   

    // MODIFIED: New fields for specifying the trajectory start and end points.
    // These are interpreted as the gamma and theta angles (in radians) for the right flipper.
    float start_gamma;  // Start gamma (in radians)
    float start_theta;  // Start theta (in radians)
    float end_gamma;    // End gamma (in radians)
    float end_theta;    // End theta (in radians)
    
    float curve_angle; // curve offset (in radians)

    int num_waypoints; // number of waypoints
    std::vector<float> waypoints_x; // x coordinates of waypoints
    std::vector<float> waypoints_y; // y coordinates of waypoints
    std::vector<float> waypoints_v; // velocity of waypoints
};

struct turtle{
    turtle_status turtle_chassis;
    trutle_command turtle_control;
    human_interface turtle_gui;
    TrajectoryData traj_data;
};





#endif
