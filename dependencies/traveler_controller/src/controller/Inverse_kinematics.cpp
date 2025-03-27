#include "controller/inverse_kinematics.h"

#define _USE_MATH_DEFINES
#include <fstream>
#include <iostream>
#include <cmath>
using namespace std;

#define DEBUG

/**
 * ! Leg Workspace:
 * Gamma is the rotation angle of the odrive motor and must be within [-0.79, 0.79]
 * Theta is the big servo angle and must be within [-1.05, +1.05] radians
 * Beta is the small servo angle and must be within [-1.05, +1.05] radians
 */

/**
 * @brief This modified function is derived from your original 
 *        fixed_insertion_depth_gait_lower_point_version_3_analytic_solution.
 *
 *        For testing curved trajectories only, we combine the shearing and extraction
 *        movements into one continuous phase. In this version the user specifies any valid 
 *        start and end point (in gamma and theta for the right flipper) which define a target 
 *        trajectory in the workspace. A mid–point is computed from these using a lateral offset 
 *        (curve_offset, in radians). Then, the code smoothly interpolates between start and mid, and 
 *        mid and end during the combined phase.
 *
 * @param turtle_  The turtle object (contains trajectory data and control handles, including user inputs)
 * @param t        The current time (in seconds)
 */
void fixed_insertion_depth_gait_lower_point_version_3_analytic_solution(turtle& turtle_, float t) {
    // Fixed parameters (we keep these for the phase-splitting blueprint)
    double l1 = 0.130;                // flipper length (new shorter flipper)
    double turtle_height = 0.079;       // height from flipper to ground (e.g., pivot-to-ground)
    double lower_point = 0.055;         // a lower reference point

    // Get trajectory data and define timing parameters as before
    float horizontal_angle = turtle_.traj_data.lateral_angle_range * 180 / M_PI; // convert radians to degrees

    Rectangle_Params rectangle_params;
    rectangle_params.period_down = turtle_.traj_data.lateral_angle_range * l1 * 2 / turtle_.traj_data.drag_speed; // (Note: “down” phase
    rectangle_params.period_up = 0.8;       // customize back phase time (unused here)
    rectangle_params.period_left = turtle_.traj_data.servo_speed;  // used for second segment
    rectangle_params.period_right = turtle_.traj_data.servo_speed; // (unused here)
    rectangle_params.vertical_range = turtle_.traj_data.insertion_depth; // (unused here)
    rectangle_params.horizontal_range = turtle_.traj_data.lateral_angle_range * 180 / M_PI;
    rectangle_params.period_waiting_time = 0;

    // We use only two hold times for this combined phase blueprint
    // (the other hold times and period_up are not used in this interpolation)
    // MODIFIED: We retain period_down and period_left to split the combined phase.
    float combined_duration = rectangle_params.period_down + rectangle_params.period_left;
    float t_mod = fmod(t, combined_duration);
    turtle_.turtle_chassis.step_count = (t - t_mod) / combined_duration;

    // MODIFIED: Retrieve the user-specified start and end configurations (in radians)
    // For the right flipper, start: (start_x, start_y) and end: (end_x, end_y)
    double start_gamma = turtle_.traj_data.start_x;  // start adduction (gamma)
    double start_theta = turtle_.traj_data.start_y;  // start sweeping (theta)
    double end_gamma   = turtle_.traj_data.end_x;    // end adduction (gamma)
    double end_theta   = turtle_.traj_data.end_y;    // end sweeping (theta)

    // MODIFIED: Compute differences between start and end (for later use in mid-point computation)
    double d_gamma = end_gamma - start_gamma;
    double d_theta = end_theta - start_theta;
    double norm = sqrt(d_gamma * d_gamma + d_theta * d_theta);

    // MODIFIED: Compute the linear midpoint between start and end
    double mid_gamma = (start_gamma + end_gamma) / 2.0;
    double mid_theta = (start_theta + end_theta) / 2.0;

    // MODIFIED: Apply an offset to the midpoint to achieve a curved trajectory.
    // The offset is perpendicular to the line from start to end.
    double offset = turtle_.traj_data.curve_angle;  // curve offset magnitude (in radians); user-defined
    if (norm > 1e-6) {
        mid_gamma += -d_theta / norm * offset;
        mid_theta += d_gamma / norm * offset;
    }

    // MODIFIED: Interpolate between the points.
    double gamma2 = 0;
    double theta2 = 0;
    double corres_t = 0;
    if (t_mod < rectangle_params.period_down) {
        // First segment: from start to mid-point
        corres_t = t_mod / rectangle_params.period_down;
        // You may optionally apply a smoothing (easing) function here.
        gamma2 = (1.0 - corres_t) * start_gamma + corres_t * mid_gamma;
        theta2 = (1.0 - corres_t) * start_theta + corres_t * mid_theta;
        turtle_.turtle_chassis.gait_state = 1;  // forward segment indicator
    }
    else {
        // Second segment: from mid-point to end
        corres_t = (t_mod - rectangle_params.period_down) / rectangle_params.period_left;
        gamma2 = (1.0 - corres_t) * mid_gamma + corres_t * end_gamma;
        theta2 = (1.0 - corres_t) * mid_theta + corres_t * end_theta;
        turtle_.turtle_chassis.gait_state = 2;  // reverse segment indicator
    }

    // Debug printout
    cout << "Combined Phase: corres_t=" << corres_t
         << ", theta2=" << theta2 << ", gamma2=" << gamma2 << endl;

    // MODIFIED: Set the computed commands for the right flipper.
    // The commands are sent in degrees for one interface and in radians for another.
    turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = gamma2 * 180.0 / M_PI;
    turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = theta2 * 180.0 / M_PI;
    turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = gamma2;
    turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = theta2;

    // Set the left flipper to an idle state (since only the right flipper is used).
    turtle_.turtle_control.left_adduction.set_input_position_degree.input_position = 0;
    turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position = 0;
    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = 0;
    turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = 0;
}

/**
 * @brief Bounding gaits.
 * @param turtle_ The turtle object.
 * @param t       The current time.
 */
void boundingGAIT(turtle& turtle_, float t)
{
    fixed_insertion_depth_gait_lower_point_version_3_analytic_solution(turtle_, t);
}
