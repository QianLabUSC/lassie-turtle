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
 *       MODIFIED: It now also uses user-specified start and end coordinates.
 */
void fixed_insertion_depth_gait_lower_point_version_3_analytic_solution(turtle& turtle_, float t) {

    double l1 = 0.130;         // flipper length (new shorter flipper)
    double turtle_height = 0.079; // height from flipper to ground (e.g., pivot-to-ground)
    double lower_point = 0.055;   // a lower reference point

    // MODIFIED: Compute effective horizontal angle.
    // Compute the angle between the user-specified start and end points.
    float computed_angle = atan2(turtle_.traj_data.end_y - turtle_.traj_data.start_y,
                                 turtle_.traj_data.end_x - turtle_.traj_data.start_x) * 180 / M_PI;
    float effective_angle;
    // If the y difference is negligible, use the computed angle (linear trajectory);
    // otherwise, use the originally defined lateral_angle_range (curved trajectory).
    if (fabs(turtle_.traj_data.end_y - turtle_.traj_data.start_y) < 1e-3) { // threshold for "same y"
        effective_angle = computed_angle;
    } else {
        effective_angle = turtle_.traj_data.lateral_angle_range * 180 / M_PI;
    }

    // MODIFIED: Use effective_angle in place of horizontal_angle.
    // float horizontal_angle = turtle_.traj_data.lateral_angle_range * 180 / M_PI;
    float horizontal_angle = effective_angle;

    // Set up timing parameters (same as before)
    Rectangle_Params rectangle_params;
    rectangle_params.period_down = turtle_.traj_data.lateral_angle_range * l1 * 2 / turtle_.traj_data.drag_speed; 
    rectangle_params.period_up = 0.8;       // customize back phase time
    rectangle_params.period_left = turtle_.traj_data.servo_speed;
    rectangle_params.period_right = turtle_.traj_data.servo_speed;
    rectangle_params.vertical_range = turtle_.traj_data.insertion_depth; // depends on insertion depth
    rectangle_params.horizontal_range = turtle_.traj_data.lateral_angle_range * 180 / M_PI;
    rectangle_params.period_waiting_time = 0;

    float hold_time_1 = 3.0; // hold duration for a new phase (unused here)
    float hold_time_2 = 3.0; // hold duration for a new phase (unused here)
    float hold_time_3 = 3.0; // hold duration for a new phase (unused here)
    float end_delay    = 3.0; // delay at the end (unused here)

    // Combined duration for the extraction (and simultaneous shearing) phase.
    float combined_duration = rectangle_params.period_down + rectangle_params.period_left;
    float t_mod = fmod(t, combined_duration);
    turtle_.turtle_chassis.step_count = (t - t_mod) / combined_duration;

    // Desired insertion depth limits.
    double desierd_insertion_depth = turtle_.traj_data.insertion_depth;
    if (desierd_insertion_depth > 0.07) {
        desierd_insertion_depth = 0.07;
    }
    cout << "desired insertion depth(m): " << desierd_insertion_depth << endl;

    // Initial insertion depth calculation.
    double initial_insertion_depth_rad = asin((desierd_insertion_depth + turtle_height) /
        sqrt((l1 * cos(horizontal_angle * M_PI / 180)) * (l1 * cos(horizontal_angle * M_PI / 180)) + lower_point * lower_point))
        - atan(lower_point / (l1 * cos(horizontal_angle * M_PI / 180)));
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

    // Combine shear and extraction.
    float curve_offset_deg = 30.0f;  // This parameter can be adjusted via the GUI (e.g., turtle_.traj_data.curve_angle)

    // For the sweeping angle (theta):
    // We start at the “extracted” (or fully inserted) position, then add a shear offset that increases with time.
    if(t_mod < rectangle_params.period_down){
        corres_t = t_mod / rectangle_params.period_down;
        // For mirrored behavior on the right flipper:
        theta2 = horizontal_angle - curve_offset_deg * corres_t;
        // For extraction (adduction) angle (gamma):
        gamma2 = right_hori_servo - extraction_angle + (initial_insertion_depth_rad * 180 / M_PI + extraction_angle) * corres_t;
    }
    else{
        corres_t = (t_mod - rectangle_params.period_down) / rectangle_params.period_left;
        // Mirror behavior for sweeping:
        theta2 = horizontal_angle - curve_offset_deg + (curve_offset_deg * corres_t);
        // Interpolate extraction (adduction) angle (gamma):
        gamma2 = right_hori_servo + (initial_insertion_depth_rad * 180 / M_PI) - (initial_insertion_depth_rad * 180 / M_PI + extraction_angle) * corres_t;
    }

    cout << "Combined Phase: corres_t=" << corres_t
         << ", theta2=" << theta2 << ", gamma2=" << gamma2 << endl;

    // Set a gait state value indicating that we're in the combined shear+extraction phase.
    turtle_.turtle_chassis.gait_state = 3;

    // Send commands to servos (in degrees)
    turtle_.turtle_control.left_adduction.set_input_position_degree.input_position = gamma1;
    turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position = theta1;
    turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = gamma2;
    turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = theta2;

    // Also, send commands in radians (converted to turns)
    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = -gamma1 / 360;
    turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = -theta1 / 360;
    turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = -gamma2 / 360;
    turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = -theta2 / 360;
}

/**
 * @brief bounding gaits.
 * @param t  The current time (in seconds)
 */
void boundingGAIT(turtle& turtle_, float t)
{
    fixed_insertion_depth_gait_lower_point_version_3_analytic_solution(turtle_, t);
}
