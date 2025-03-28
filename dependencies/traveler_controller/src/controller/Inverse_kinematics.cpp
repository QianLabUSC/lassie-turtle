#include "controller/inverse_kinematics.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
using namespace std;

#define DEBUG

/**
 * ! Leg Workspace:
 * Gamma is the rotation angle of the odrive motor and must be within [-0.79, 0.79]
 * Theta is the big servo angle and must be within [-1.05, +1.05] radians
 * Beta is the small servo angle and must be within [-1.05, +1.05] radians
 */


/*
void fixed_insertion_depth_gait_lower_point_version_3_analytic_solution(turtle& turtle_, float t) {
    //
}
*/

// NEW: custom_trajectory function
namespace turtle_namespace {
namespace control {

void custom_trajectory(turtle& turtle_, float t)
{
    // Define phase durations for each segment of the cycle.
    // You may adjust these durations for smooth motion.
    const float T1 = 3.0; // Phase 1: from current (or idle) configuration to user-specified start.
    const float T2 = 5.0; // Phase 2: from start to end.
    const float T3 = 5.0; // Phase 3: from end back to start.
    const float T_total = T1 + T2 + T3;
    
    float u;
    double desired_gamma, desired_theta;
    
    if (t < T1) {
        // Phase 1: Interpolate from current configuration to the user-specified start.
        // (Here we assume the current position is 0; you may use saved current values if available.)
        double current_gamma = 0.0;
        double current_theta = 0.0;
        u = t / T1;
        desired_gamma = current_gamma + u * (turtle_.traj_data.start_gamma - current_gamma);
        desired_theta = current_theta + u * (turtle_.traj_data.start_theta - current_theta);
    }
    else if (t < T1 + T2) {
        // Phase 2: Interpolate from start to end.
        u = (t - T1) / T2;
        desired_gamma = turtle_.traj_data.start_gamma + u * (turtle_.traj_data.end_gamma - turtle_.traj_data.start_gamma);
        desired_theta = turtle_.traj_data.start_theta + u * (turtle_.traj_data.end_theta - turtle_.traj_data.start_theta);
    }
    else if (t < T_total) {
        // Phase 3: Interpolate from end back to start.
        u = (t - T1 - T2) / T3;
        desired_gamma = turtle_.traj_data.end_gamma + u * (turtle_.traj_data.start_gamma - turtle_.traj_data.end_gamma);
        desired_theta = turtle_.traj_data.end_theta + u * (turtle_.traj_data.start_theta - turtle_.traj_data.end_theta);
    }
    else {
        // After the cycle, hold the start configuration and signal to stop the trajectory.
        desired_gamma = turtle_.traj_data.start_gamma;
        desired_theta = turtle_.traj_data.start_theta;
        turtle_.turtle_gui.start_flag = 0;
    }
    
    // Command the right flipper:
    // For one interface, convert to degrees; for another, keep in radians.
    turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = desired_gamma * 180.0 / M_PI;
    turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = desired_theta * 180.0 / M_PI;
    turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = desired_gamma;
    turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = desired_theta;
    
    // Optionally, set the left flipper to idle.
    turtle_.turtle_control.left_adduction.set_input_position_degree.input_position = 0;
    turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position = 0;
    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = 0;
    turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = 0;
    
    // Update the gait state for debugging purposes.
    turtle_.turtle_chassis.gait_state = 9;
}

// For compatibility with your existing state machine, boundingGAIT now simply calls custom_trajectory.
void boundingGAIT(turtle& turtle_, float t)
{
    custom_trajectory(turtle_, t);
}

} // namespace control
} // namespace turtle_namespace



