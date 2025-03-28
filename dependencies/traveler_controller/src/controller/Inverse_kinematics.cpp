#include "controller/inverse_kinematics.h"
#include <fstream>
#include <iostream>
#include <cmath>
using namespace std;

#define DEBUG

// Select the phase combination based on the difference between endpoints.
PhaseCombination selectPhase(double start_gamma, double start_theta,
                             double end_gamma, double end_theta) {
    const double threshold_gamma = 0.1; 
    const double threshold_theta = 0.2; 

    double delta_gamma = end_gamma - start_gamma;
    double delta_theta = end_theta - start_theta;

    PhaseCombination mode;
    if(fabs(delta_gamma) < threshold_gamma && fabs(delta_theta) < threshold_theta) {
        // Minimal change: default to insertion only.
        mode = INSERTION_ONLY;
    } else if(fabs(delta_gamma) >= threshold_gamma && fabs(delta_theta) < threshold_theta) {
        // Movement mainly in gamma.
        mode = (delta_gamma > 0) ? INSERTION_ONLY : EXTRACTION_ONLY;
    } else if(fabs(delta_gamma) < threshold_gamma && fabs(delta_theta) >= threshold_theta) {
        // Movement mainly in theta.
        mode = (delta_theta > 0) ? SWING_ONLY : STANCE_ONLY;
    } else {
        // Both angles change significantly.
        if(delta_gamma > 0 && delta_theta > 0)
            mode = INSERTION_AND_SWING;
        else if(delta_gamma < 0 && delta_theta > 0)
            mode = SWING_AND_EXTRACTION;
        else if(delta_gamma > 0 && delta_theta < 0)
            mode = STANCE_AND_INSERTION;
        else // (delta_gamma < 0 && delta_theta < 0)
            mode = STANCE_AND_EXTRACTION;
    }
    return mode;
}

// Main function that computes and sends commands based on the selected phase.
void combined_phase_trajectory(turtle& turtle_, float t, PhaseCombination mode) {
    // Common geometry parameters.
    double l1 = 0.130;          // flipper length (meters)
    double turtle_height = 0.079; // height from flipper to ground (meters)
    double lower_point = 0.055;   // reference distance (meters)

    // Convert lateral_angle_range (in radians) to degrees.
    float horizontal_angle = turtle_.traj_data.lateral_angle_range * 180 / M_PI;

    // Set up timing parameters from trajectory data.
    Rectangle_Params rectangle_params;
    rectangle_params.period_down  = turtle_.traj_data.lateral_angle_range * l1 * 2 / turtle_.traj_data.drag_speed;
    rectangle_params.period_up    = 0.8; // fixed back phase time
    rectangle_params.period_left  = turtle_.traj_data.servo_speed;
    rectangle_params.period_right = turtle_.traj_data.servo_speed;
    rectangle_params.vertical_range   = turtle_.traj_data.insertion_depth;
    rectangle_params.horizontal_range = turtle_.traj_data.lateral_angle_range * 180 / M_PI;
    rectangle_params.period_waiting_time = 0;

    // Hold times and delays (used in some phases)
    float hold_time_1 = 3.0;
    float hold_time_2 = 3.0;
    float hold_time_3 = 3.0;
    float end_delay    = 3.0;

    // Baseline servo offsets and extraction parameter.
    float left_hori_servo = 0;
    float right_hori_servo = 0;
    float extraction_angle = turtle_.traj_data.extraction_angle;

    // Variables to store computed angles.
    double gamma1 = 0, theta1 = 0, gamma2 = 0, theta2 = 0;
    float corres_t = 0;

    // For phases that involve insertion depth calculations.
    double desierd_insertion_depth = turtle_.traj_data.insertion_depth;
    if(desierd_insertion_depth > 0.07)
        desierd_insertion_depth = 0.07;

    // Compute the initial insertion depth (used in insertion, stance, extraction phases).
    double initial_insertion_depth_rad = asin((desierd_insertion_depth + turtle_height) /
        sqrt((l1 * cos(horizontal_angle * M_PI / 180)) * (l1 * cos(horizontal_angle * M_PI / 180)) + lower_point * lower_point))
        - atan(lower_point / (l1 * cos(horizontal_angle * M_PI / 180)));
    double initial_insertion_depth_deg = initial_insertion_depth_rad * 180 / M_PI;

    // Switch on the selected mode.
    switch(mode) {
        case SWING_ONLY:
            {
                // Swing only: assume the motion is along theta only.
                float T_total = rectangle_params.period_up;
                float t_mod = fmod(t, T_total);
                corres_t = t_mod / T_total;
                gamma1 = left_hori_servo + extraction_angle;
                theta1 = -horizontal_angle + 2 * horizontal_angle * corres_t;
                gamma2 = right_hori_servo - extraction_angle;
                theta2 = horizontal_angle - 2 * horizontal_angle * corres_t;
                turtle_.turtle_chassis.gait_state = 1;
            }
            break;
        case INSERTION_ONLY:
            {
                // Insertion only: vertical motion.
                float T_total = rectangle_params.period_up + hold_time_3 + rectangle_params.period_right;
                float t_mod = fmod(t, T_total);
                if(t_mod < rectangle_params.period_up + hold_time_3) {
                    // Hold at the initial configuration.
                    theta1 = horizontal_angle;
                    gamma1 = left_hori_servo + extraction_angle;
                    theta2 = -horizontal_angle;
                    gamma2 = right_hori_servo - extraction_angle;
                } else {
                    corres_t = (t_mod - rectangle_params.period_up - hold_time_3) / rectangle_params.period_right;
                    theta1 = horizontal_angle;
                    gamma1 = left_hori_servo + extraction_angle - (((initial_insertion_depth_rad * 180 / M_PI) + extraction_angle) * corres_t);
                    theta2 = -horizontal_angle;
                    gamma2 = right_hori_servo - extraction_angle + (((initial_insertion_depth_rad * 180 / M_PI) + extraction_angle) * corres_t);
                }
                turtle_.turtle_chassis.gait_state = 2;
                cout << "Insertion Only - Initial Adduction: " << gamma1 << endl;
            }
            break;
        case STANCE_ONLY:
            {
                // Stance only: horizontal motion (theta decreasing).
                float T_total = rectangle_params.period_up + hold_time_3 + rectangle_params.period_right + hold_time_1 + rectangle_params.period_down;
                float t_mod = fmod(t, T_total);
                corres_t = (t_mod - rectangle_params.period_up - hold_time_3 - rectangle_params.period_right - hold_time_1) / rectangle_params.period_down;
                theta1 = horizontal_angle - 2 * horizontal_angle * corres_t;
                gamma1 = left_hori_servo - (asin((desierd_insertion_depth + turtle_height) /
                          sqrt((l1 * cos(-theta1 * M_PI / 180)) * (l1 * cos(-theta1 * M_PI / 180)) + lower_point * lower_point)) -
                          atan(lower_point / (l1 * cos(-theta1 * M_PI / 180)))) * 180 / M_PI;
                theta2 = -horizontal_angle + 2 * horizontal_angle * corres_t;
                gamma2 = right_hori_servo + (asin((desierd_insertion_depth + turtle_height) /
                           sqrt((l1 * cos(-theta1 * M_PI / 180)) * (l1 * cos(-theta1 * M_PI / 180)) + lower_point * lower_point)) -
                           atan(lower_point / (l1 * cos(-theta1 * M_PI / 180)))) * 180 / M_PI;
                turtle_.turtle_chassis.gait_state = 3;
                cout << "Stance Only - Phase3 Adduction: " << gamma1 << endl;
            }
            break;
        case EXTRACTION_ONLY:
            {
                // Extraction only: upward motion (reverse of insertion).
                float T_total = rectangle_params.period_up + hold_time_3 + rectangle_params.period_right + hold_time_1 +
                                rectangle_params.period_down + hold_time_2 + rectangle_params.period_left;
                float t_mod = fmod(t, T_total);
                corres_t = (t_mod - rectangle_params.period_up - hold_time_3 - rectangle_params.period_right -
                           hold_time_1 - rectangle_params.period_down - hold_time_2) / rectangle_params.period_left;
                theta1 = -horizontal_angle;
                gamma1 = left_hori_servo - (initial_insertion_depth_rad * 180 / M_PI) +
                         (((initial_insertion_depth_rad * 180 / M_PI) + extraction_angle) * corres_t);
                theta2 = horizontal_angle;
                gamma2 = right_hori_servo + (initial_insertion_depth_rad * 180 / M_PI) -
                         (((initial_insertion_depth_rad * 180 / M_PI) + extraction_angle) * corres_t);
                turtle_.turtle_chassis.gait_state = 4;
            }
            break;
        case INSERTION_AND_SWING:
            {
                // Combined insertion and swing: blend insertion (vertical) and swing (horizontal) simultaneously.
                float combined_duration = rectangle_params.period_down + rectangle_params.period_left;
                float t_mod = fmod(t, combined_duration);
                turtle_.turtle_chassis.step_count = (t - t_mod) / combined_duration;
                double desierd_insertion_depth = turtle_.traj_data.insertion_depth;
                if(desierd_insertion_depth > 0.07)
                    desierd_insertion_depth = 0.07;
                double initial_insertion_depth_rad = asin((desierd_insertion_depth + turtle_height) /
                    sqrt((l1 * cos(horizontal_angle * M_PI / 180)) * (l1 * cos(horizontal_angle * M_PI / 180)) + lower_point * lower_point))
                    - atan(lower_point / (l1 * cos(horizontal_angle * M_PI / 180)));
                float curve_offset_deg = 30.0f;
                if(t_mod < rectangle_params.period_down) {
                    corres_t = t_mod / rectangle_params.period_down;
                    theta2 = horizontal_angle - curve_offset_deg * corres_t;  
                    gamma2 = right_hori_servo - extraction_angle +
                             (((initial_insertion_depth_rad * 180 / M_PI) + extraction_angle) * corres_t);
                } else {
                    corres_t = (t_mod - rectangle_params.period_down) / rectangle_params.period_left;
                    theta2 = horizontal_angle - curve_offset_deg + (curve_offset_deg * corres_t);
                    gamma2 = right_hori_servo + (initial_insertion_depth_rad * 180 / M_PI) -
                             (((initial_insertion_depth_rad * 180 / M_PI) + extraction_angle) * corres_t);
                }
                turtle_.turtle_chassis.gait_state = 10;
            }
            break;
        case SWING_AND_EXTRACTION:
            {
                // Combined swing and extraction: blend swing (increasing theta) with extraction (decreasing gamma).
                float combined_duration = rectangle_params.period_up + rectangle_params.period_left;
                float t_mod = fmod(t, combined_duration);
                turtle_.turtle_chassis.step_count = (t - t_mod) / combined_duration;
                corres_t = t_mod / combined_duration;
                theta2 = horizontal_angle + 2 * horizontal_angle * corres_t;
                gamma2 = right_hori_servo - extraction_angle * corres_t;
                turtle_.turtle_chassis.gait_state = 11;
            }
            break;
        case STANCE_AND_INSERTION:
            {
                // Combined stance and insertion: blend stance (leftward swing) with insertion.
                float combined_duration = rectangle_params.period_down + rectangle_params.period_left;
                float t_mod = fmod(t, combined_duration);
                turtle_.turtle_chassis.step_count = (t - t_mod) / combined_duration;
                double desierd_insertion_depth = turtle_.traj_data.insertion_depth;
                if(desierd_insertion_depth > 0.07)
                    desierd_insertion_depth = 0.07;
                double initial_insertion_depth_rad = asin((desierd_insertion_depth + turtle_height) /
                    sqrt((l1 * cos(horizontal_angle * M_PI / 180)) * (l1 * cos(horizontal_angle * M_PI / 180)) + lower_point * lower_point))
                    - atan(lower_point / (l1 * cos(horizontal_angle * M_PI / 180)));
                float curve_offset_deg = 30.0f;
                if(t_mod < rectangle_params.period_down) {
                    corres_t = t_mod / rectangle_params.period_down;
                    theta2 = horizontal_angle + curve_offset_deg * corres_t;  
                    gamma2 = right_hori_servo - extraction_angle +
                             (((initial_insertion_depth_rad * 180 / M_PI) + extraction_angle) * corres_t);
                } else {
                    corres_t = (t_mod - rectangle_params.period_down) / rectangle_params.period_left;
                    theta2 = horizontal_angle + curve_offset_deg - (curve_offset_deg * corres_t);
                    gamma2 = right_hori_servo + (initial_insertion_depth_rad * 180 / M_PI) -
                             (((initial_insertion_depth_rad * 180 / M_PI) + extraction_angle) * corres_t);
                }
                turtle_.turtle_chassis.gait_state = 12;
            }
            break;
        case STANCE_AND_EXTRACTION:
            {
                // Combined stance and extraction: blend stance (decreasing theta) with extraction (increasing gamma).
                float combined_duration = rectangle_params.period_up + rectangle_params.period_left;
                float t_mod = fmod(t, combined_duration);
                turtle_.turtle_chassis.step_count = (t - t_mod) / combined_duration;
                corres_t = t_mod / combined_duration;
                theta2 = -horizontal_angle - 2 * horizontal_angle * corres_t;
                gamma2 = right_hori_servo + extraction_angle * corres_t;
                turtle_.turtle_chassis.gait_state = 13;
            }
            break;
    }
    
    // Add a print statement to output the computed angles.
    cout << "Phase = " << corres_t 
         << ", gamma1 = " << gamma1 << ", theta1 = " << theta1 
         << ", gamma2 = " << gamma2 << ", theta2 = " << theta2 << endl;
    
    // Send commands to servos.
    turtle_.turtle_control.left_adduction.set_input_position_degree.input_position = gamma1;
    turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position = theta1;
    turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = gamma2;
    turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = theta2;

    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = -gamma1 / 360;
    turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = -theta1 / 360;
    turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = -gamma2 / 360;
    turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = -theta2 / 360;
}


void boundingGAIT(turtle& turtle_, float t, int mode) {
    double start_gamma = turtle_.traj_data.start_gamma;
    double start_theta = turtle_.traj_data.start_theta;
    double end_gamma   = turtle_.traj_data.end_gamma;
    double end_theta   = turtle_.traj_data.end_theta;
    PhaseCombination autoMode = selectPhase(start_gamma, start_theta, end_gamma, end_theta);
    combined_phase_trajectory(turtle_, t, autoMode);
}
