#pragma once
#include <vector>
#include <cmath>
#include "traveler_msgs/msg/set_input_position.hpp"
#include "traveler_msgs/msg/set_state.hpp"
#include "traveler_msgs/msg/odrive_status.hpp"

// Simple (x,y) pair
struct XY_pair {
    float x;
    float y;
    XY_pair() : x(0.0f), y(0.0f) {}
    XY_pair(float x_, float y_) : x(x_), y(y_) {}
};

// Pair of XY points
struct Point_Pair {
    XY_pair A;
    XY_pair B;
    Point_Pair() : A(), B() {}
    Point_Pair(XY_pair a, XY_pair b) : A(a), B(b) {}
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

// Motor axis control structure
struct motor_axis {
    float motor_control_position = 0.0f;
    float position = 0.0f;
    float velocity = 0.0f;
    float effort = 0.0f;
    traveler_msgs::msg::SetInputPosition set_input_position;
};

// Leg structure with two axes
struct leg_structure {
    motor_axis axis0;
    motor_axis axis1;
    XY_pair toe_position;
};

// Motor command for one actuator
struct motor_command {
    traveler_msgs::msg::SetInputPosition set_input_position_radian;
    float motor_control_position = 0.0f;  // Added missing field
};

// Set state command structure
struct set_state_command {
    traveler_msgs::msg::SetState set_state;
};

// Basic turtle state
struct turtle_status {
    float gait_state = 0;   // 0: idle, 1..N: running phases if you need
    int step_count = 0;
    int if_idle_count = 1;  // Added missing field
    
    // ODrive status messages
    traveler_msgs::msg::OdriveStatus left_adduction;
    traveler_msgs::msg::OdriveStatus left_sweeping;
    traveler_msgs::msg::OdriveStatus right_adduction;
    traveler_msgs::msg::OdriveStatus right_sweeping;
    
    // Leg structures
    leg_structure Leg_lf;  // Left front leg
};

// Motor command set for both DOF
struct turtle_command {
    bool if_control = false;
    motor_command left_adduction;
    motor_command left_sweeping;
    motor_command right_adduction;
    motor_command right_sweeping;
    
    // Set state commands
    set_state_command left_adduction_state;
    set_state_command left_sweeping_state;
    set_state_command right_adduction_state;
    set_state_command right_sweeping_state;
    
    // Leg structures for control
    leg_structure Leg_lf;  // Left front leg
};

// GUI interface (can be bypassed by hardcoding start_flag=1)
struct human_interface {
    int start_flag = 0;   // 0 = stopped, 1 = run
    bool status_update_flag = false;  // Added missing field
};

// Waypoint trajectory data
struct TrajectoryData {
    int num_waypoints = 0;
    std::vector<float> waypoints_x;
    std::vector<float> waypoints_y;
    std::vector<float> waypoints_v;
};

// Program state enum
enum class ProgramState {
    FirstIteration,
    SetToControlAndCalibrate,
    GoToInitialPoint,
    Running,
};

// Root turtle struct
struct turtle {
    turtle_status turtle_chassis;
    turtle_command turtle_control;
    human_interface turtle_gui;
    TrajectoryData traj_data;
};

