#pragma once
#include <vector>
#include <cmath>
#include "traveler_msgs/msg/set_input_position.hpp"

// Simple (x,y) pair
struct XY_pair {
    float x;
    float y;
    XY_pair() : x(0.0f), y(0.0f) {}
    XY_pair(float x_, float y_) : x(x_), y(y_) {}
};

// Waypoint with velocity
struct Waypoint {
    XY_pair point;
    float vel;
    float delay;
    Waypoint() : point(), vel(0.0f), delay(0.0f) {}
    Waypoint(XY_pair p, float v, float d=0.0f) : point(p), vel(v), delay(d) {}
    Waypoint(float x, float y, float v, float d=0.0f) : point(x,y), vel(v), delay(d) {}
};

// Basic turtle state
struct turtle_status {
    float gait_state = 0;   // 0: idle, 1..N: running phases if you need
    int step_count = 0;
};

// Motor command for one actuator
struct motor_command {
    traveler_msgs::msg::SetInputPosition set_input_position_radian;
};

// Motor command set for both DOF
struct turtle_command {
    bool if_control = false;
    motor_command left_adduction;
    motor_command left_sweeping;
    motor_command right_adduction;
    motor_command right_sweeping;
};

// GUI interface (can be bypassed by hardcoding start_flag=1)
struct human_interface {
    int start_flag = 0;   // 0 = stopped, 1 = run
};

// Waypoint trajectory data
struct TrajectoryData {
    int num_waypoints = 0;
    std::vector<float> waypoints_x;
    std::vector<float> waypoints_y;
    std::vector<float> waypoints_v;
};

// Root turtle struct
struct turtle {
    turtle_status turtle_chassis;
    turtle_command turtle_control;
    human_interface turtle_gui;
    TrajectoryData traj_data;
};
