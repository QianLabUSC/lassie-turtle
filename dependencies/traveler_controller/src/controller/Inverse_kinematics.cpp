#include "controller/inverse_kinematics.h"

#define _USE_MATH_DEFINES
#include <fstream>
#include <iostream>
#include <cmath>
using namespace std;

#define DEBUG

/**
 * ! Leg Workspace:
 * Gamma is the rotation angle of odrive motor and must be within [-0.79 0.79]
 * Theta is the big servo angle and must be within [-1.05, +1.05] radians
 * Beta is the small servo angle and must be within [-1.05, +1.05] radians
 */

/**
 * @brief Finds the motor and servo control command for a given toe position.
 *
 * @param turtle_  The turtle object (contains trajectory data and control handles)
 * @param t        The current time (in seconds)
 *
 * @note This modified function is derived from your original 
 *       fixed_insertion_depth_gait_lower_point_version_3_analytic_solution.
 *       It combines shear and extraction movements into one continuous phase.
 *       // MODIFIED: Now uses user-specified start and end coordinates.
 */
void fixed_insertion_depth_gait_lower_point_version_3_analytic_solution(turtle& turtle_, float t) {

    double l1 = 0.130;         // flipper length (new shorter flipper)
    double turtle_height = 0.079; // height from flipper to ground (e.g., pivot-to-ground)
    double lower_point = 0.055;   // a lower reference point

    // MODIFIED: Compute the difference vector from start to end (user-specified, in meters)
    float dx = turtle_.traj_data.end_x - turtle_.traj_data.start_x;
    float dy = turtle_.traj_data.end_y - turtle_.traj_data.start_y;
    float distance = sqrt(dx * dx + dy * dy);
    // Convert the distance (arc length) to an effective angle (in degrees) using the flipper length:
    float effective_lateral_range = (distance / l1) * (180 / M_PI);
    // If distance is very small, you might want to enforce a minimum effective range
    // effective_lateral_range = (effective_lateral_range < 1.0f) ? 1.0f : effective_lateral_range;

    // MODIFIED: Use the effective lateral range in place of the original lateral_angle_range.
    // (This defines the amplitude of the horizontal movement.)
    Rectangle_Params rectangle_params;
    rectangle_params.period_down = effective_lateral_range * l1 * 2 / turtle_.traj_data.drag_speed;
    rectangle_params.period_up = 0.8;       // customize back phase time
    rectangle_params.period_left = turtle_.traj_data.servo_speed;
    rectangle_params.period_right = turtle_.traj_data.servo_speed;
    rectangle_params.vertical_range = turtle_.traj_data.insertion_depth;
    rectangle_params.horizontal_range = effective_lateral_range;
    rectangle_params.period_waiting_time = 0;

    float hold_time_1 = 3.0; // hold duration for a new phase (unused here)
    float hold_time_2 = 3.0; // hold duration for a new phase (unused here)
    float hold_time_3 = 3.0; // hold duration for a new phase (unused here)
    float end_delay    = 3.0; // delay at the end (unused here)

    float total_period = rectangle_params.period_down + rectangle_params.period_up +
                         rectangle_params.period_left + rectangle_params.period_right +
                         hold_time_1 + hold_time_2 + hold_time_3 + end_delay +
                         rectangle_params.period_waiting_time;

    float t_mod = fmod(t, total_period);
    turtle_.turtle_chassis.step_count = (t - t_mod) / total_period;

    double desierd_insertion_depth = turtle_.traj_data.insertion_depth;
    if (desierd_insertion_depth > 0.07) {
        desierd_insertion_depth = 0.07;
    }
    cout << "desired insertion depth(m): " << desierd_insertion_depth << endl;

    // Fixed insertion depth initial calculation
    double initial_insertion_depth_rad = asin((desierd_insertion_depth + turtle_height) /
        sqrt((l1 * cos((effective_lateral_range) * M_PI / 180)) * (l1 * cos((effective_lateral_range) * M_PI / 180)) + lower_point * lower_point))
        - atan(lower_point / (l1 * cos((effective_lateral_range) * M_PI / 180)));
    double initial_insertion_depth_deg = initial_insertion_depth_rad * 180 / M_PI;
    cout << "TMOD: " << t_mod << endl;

    float corres_t = 0;
    float left_hori_servo = 0;   // original value (e.g., 100; manually tuned to 94)
    float right_hori_servo = 0;
    float extraction_angle = turtle_.traj_data.extraction_angle;

    double gamma1 = 0; // adduction (extraction) angle for left flipper
    double theta1 = 0; // sweeping angle for left flipper
    double gamma2 = 0; // adduction (extraction) angle for right flipper
    double theta2 = 0; // sweeping angle for right flipper

    // MODIFIED: Compute the movement direction based on the start and end points.
    float computed_angle = atan2(dy, dx) * 180 / M_PI;
    float horizontal_angle;
    // If the vertical difference is negligible, use the computed angle (linear trajectory);
    // Otherwise, use the effective lateral range (curved trajectory).
    if (fabs(dy) < 1e-3) {
        horizontal_angle = computed_angle;
    } else {
        horizontal_angle = effective_lateral_range;
    }

    // Combine shear and extraction.
    float curve_offset_deg = 30.0f;  // this value can be adjusted via the GUI if desired

    if(t_mod < rectangle_params.period_down){
        corres_t = t_mod / rectangle_params.period_down;
        // For the right flipper, mirror the sweeping:
        theta2 = horizontal_angle - curve_offset_deg * corres_t;
        // Interpolate the extraction (adduction) angle:
        gamma2 = right_hori_servo - extraction_angle + (initial_insertion_depth_rad * 180 / M_PI + extraction_angle) * corres_t;
    }
    else{
        corres_t = (t_mod - rectangle_params.period_down) / rectangle_params.period_left;
        theta2 = horizontal_angle - curve_offset_deg + (curve_offset_deg * corres_t);
        gamma2 = right_hori_servo + (initial_insertion_depth_rad * 180 / M_PI) - (initial_insertion_depth_rad * 180 / M_PI + extraction_angle) * corres_t;
    }

    cout << "Combined Phase: corres_t=" << corres_t
         << ", theta2=" << theta2 << ", gamma2=" << gamma2 << endl;

    turtle_.turtle_chassis.gait_state = 3;

    // Send commands to servos in degrees.
    turtle_.turtle_control.left_adduction.set_input_position_degree.input_position = gamma1;
    turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position = theta1;
    turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = gamma2;
    turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = theta2;

    // Also, send commands in radians (converted to turns).
    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = -gamma1 / 360;
    turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = -theta1 / 360;
    turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = -gamma2 / 360;
    turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = -theta2 / 360;
}

void boundingGAIT(turtle& turtle_, float t)
{
    fixed_insertion_depth_gait_lower_point_version_3_analytic_solution(turtle_, t);
}
