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

const double PI = 3.141592653589793238463;
const double TWO_PI = PI * 2;

namespace turtle_namespace {
namespace control {

lowerproxy::lowerproxy(std::string name) : Node(name) {
    std::cout << "Start to create the ros node subscriber and publisher" << std::endl;

    controller_state_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/robot_state", 10);

    _count = 0;

    RCLCPP_INFO(this->get_logger(), "Publisher created!!");
}

float lowerproxy::fmodf_mpi_pi(float f) {
    if (f > 0)
        return (fmodf(f + PI, TWO_PI) - PI);
    else
        return (fmodf(f - PI, TWO_PI) + PI);
}

void lowerproxy::terrain_estimation() {
    // Implementation of terrain estimation
}

// MODIFIED: Helper function to compute the desired starting servo angles based on user-specified start point.
// We assume that calling the inverse kinematics function with t = 0 yields the correct starting configuration.
void lowerproxy::calculate_initial_angles(turtle &turtle_) {
    fixed_insertion_depth_gait_lower_point_version_3_analytic_solution(turtle_, 0);
}

/// @brief calculate the motor command using different controller:
///        default: inverse kinematic controller
/// @param turtle
void lowerproxy::calculate_position(turtle &turtle_) {
    if (turtle_.turtle_gui.start_flag == 1) {
        t2 = std::chrono::high_resolution_clock::now();
        std::chrono::high_resolution_clock::time_point current_time_pre;
        std::chrono::duration<double> deltaTime_pre;
        std::chrono::high_resolution_clock::time_point current_time;
        std::chrono::duration<double> deltaTime;
        std::chrono::high_resolution_clock::time_point current_time1;
        std::chrono::duration<double> deltaTime1;

        double pre_t;
        double curr_initial_phase_time;
        double running_t;

        switch (currentState) {
            case ProgramState::FirstIteration:
                turtle_.turtle_chassis.gait_state = -1; // Initial state (could be used as a flag)
                std::cout << "First Iteration: " << std::endl;
                starting_time = std::chrono::high_resolution_clock::now();
                currentState = ProgramState::SetToControlAndCalibrate;
                turtle_.turtle_control.if_control = false;
                break;

            case ProgramState::SetToControlAndCalibrate:
                current_time_pre = std::chrono::high_resolution_clock::now();
                deltaTime_pre = current_time_pre - starting_time;
                pre_t = deltaTime_pre.count();
                if (turtle_.turtle_chassis.if_idle_count <= 0 && pre_t > set_close_control_time) {
                    currentState = ProgramState::GoToInitialPoint;
                }
                std::cout << "Waiting Iteration: " << std::endl;
                saved_left_adduction = turtle_.turtle_chassis.left_adduction.pos_estimate;
                saved_left_sweeping = turtle_.turtle_chassis.left_sweeping.pos_estimate;
                saved_right_adduction = turtle_.turtle_chassis.right_adduction.pos_estimate;
                saved_right_sweeping = turtle_.turtle_chassis.right_sweeping.pos_estimate;
                std::cout << "Saved left sweeping: " << saved_left_sweeping << std::endl;
                std::cout << "Saved left adduction: " << saved_left_adduction << std::endl;
                std::cout << "Saved right sweeping: " << saved_right_sweeping << std::endl;
                std::cout << "Saved right adduction: " << saved_right_adduction << std::endl;
                turtle_.turtle_control.if_control = false;
                break;

            case ProgramState::GoToInitialPoint:
                current_time = std::chrono::high_resolution_clock::now();
                deltaTime = current_time - starting_time;
                curr_initial_phase_time = deltaTime.count() - set_close_control_time;
                turtle_.turtle_chassis.gait_state = 0;
                turtle_.turtle_control.if_control = true;
                // MODIFIED: Compute the desired initial angles based on the user-specified start point.
                calculate_initial_angles(turtle_);
                // Now use the computed desired angles (which have been stored in the control fields)
                // as the target for moving from the saved positions.
                if (curr_initial_phase_time < initial_phase_time) {
                    goback2desiredangle(turtle_,
                        turtle_.turtle_control.left_adduction.set_input_position_degree.input_position,   // desired left adduction
                        turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position,      // desired left sweeping
                        turtle_.turtle_control.right_adduction.set_input_position_degree.input_position,     // desired right adduction
                        turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position,      // desired right sweeping
                        saved_left_adduction, saved_left_sweeping,
                        saved_right_adduction, saved_right_sweeping,
                        curr_initial_phase_time, initial_phase_time);
                } else {
                    currentState = ProgramState::Running;
                }
                std::cout << "Go-to-initial point Iteration: " << curr_initial_phase_time << std::endl;
                break;

            case ProgramState::Running:
                current_time1 = std::chrono::high_resolution_clock::now();
                deltaTime1 = current_time1 - starting_time;
                curr_initial_phase_time = deltaTime1.count();
                turtle_.turtle_control.if_control = true;
                running_t = curr_initial_phase_time - initial_phase_time - set_close_control_time;
                {
                    double l1 = 0.130; // flipper length (same as in inverse kinematics)
                    float period_down = turtle_.traj_data.lateral_angle_range * l1 * 2 / turtle_.traj_data.drag_speed;
                    float period_left = turtle_.traj_data.servo_speed;
                    float combined_duration = period_down + period_left;
                    if (running_t >= combined_duration) {
                        currentState = ProgramState::CycleCompleted; // Transition when one cycle is complete
                        std::cout << "Cycle completed. Stopping trajectory." << std::endl;
                    } else {
                        boundingGAIT(turtle_, running_t);
                        std::cout << "Running Iteration: " << running_t << std::endl;
                    }
                }
                break;

            case ProgramState::CycleCompleted:
                turtle_.turtle_chassis.gait_state = 5; // Stop state
                turtle_.turtle_control.if_control = false;
                std::cout << "Cycle Completed state reached. Motion stopped." << std::endl;
                break;
        }
    }
    // The else block for when start_flag is not set is omitted.
}

void lowerproxy::goback2desiredangle(turtle &turtle_, float left_adduction,
                                      float left_sweeping, float right_adduction,
                                      float right_sweeping, float start_left_adduction,
                                      float start_left_sweeping, float start_right_adduction,
                                      float start_right_sweeping,
                                      float t_decrease_time, float total_time) {
    left_adduction = left_adduction / 360;
    left_sweeping = left_sweeping / TWO_PI;
    right_adduction = right_adduction / 360;
    right_sweeping = right_sweeping / TWO_PI;
    total_time = total_time / 2;
    if (t_decrease_time > total_time) {
        t_decrease_time = total_time;
        turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = start_left_adduction + (left_adduction - start_left_adduction);
        turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = start_left_sweeping + (left_sweeping - start_left_sweeping);
        turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = start_right_adduction + (right_adduction - start_right_adduction);
        turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = start_right_sweeping + (right_sweeping - start_right_sweeping);
    } else {
        turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = start_left_adduction + (left_adduction - start_left_adduction) * (t_decrease_time / total_time);
        turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = start_left_sweeping + (left_sweeping - start_left_sweeping) * (t_decrease_time / total_time);
        turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = start_right_adduction + (right_adduction - start_right_adduction) * (t_decrease_time / total_time);
        turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = start_right_sweeping + (right_sweeping - start_right_sweeping) * (t_decrease_time / total_time);
    }
}

void lowerproxy::Estop() {
    // Emergency stop implementation (if needed)
}

void lowerproxy::UpdateJoystickStatus(turtle &turtle_) {
    turtle_inter_ = turtle_;
    terrain_estimation();

    auto robot_state = std_msgs::msg::Float64MultiArray();
    robot_state.data.push_back(turtle_inter_.turtle_chassis.gait_state);
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_adduction.pos_estimate);
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_sweeping.pos_estimate);
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_adduction.pos_estimate);
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_sweeping.pos_estimate);
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_adduction.iq_setpoint);
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_sweeping.iq_setpoint);
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_adduction.iq_setpoint);
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_sweeping.iq_setpoint);
    robot_state.data.push_back(turtle_inter_.turtle_control.left_adduction.set_input_position_radian.input_position);
    robot_state.data.push_back(turtle_inter_.turtle_control.left_sweeping.set_input_position_radian.input_position);
    robot_state.data.push_back(turtle_inter_.turtle_control.right_adduction.set_input_position_radian.input_position);
    robot_state.data.push_back(turtle_inter_.turtle_control.right_sweeping.set_input_position_radian.input_position);

    controller_state_publisher->publish(robot_state);
}

} // namespace control
} // namespace turtle_namespace
