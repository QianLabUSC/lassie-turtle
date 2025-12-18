#include "controller/trajectories_parser.h"
#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
using namespace std;

namespace turtle_namespace {
namespace control {

void TrajectoriesParser::cartesianMotorCommand(turtle &turtle, float target_x, float target_y) {
    physicalToAbstract(target_x, target_y, theta_, gamma_, true);
    float axis_0 = theta_ - gamma_;
    float axis_1 = theta_ + gamma_;


    turtle.turtle_control.Leg_lf.axis0.motor_control_position = axis_0;
    turtle.turtle_control.Leg_lf.axis1.motor_control_position = axis_1;

    
}

void TrajectoriesParser::generateWaypoints(turtle &turtle) {
    waypoints_.clear();
    int trajectory = turtle.turtle_gui.drag_traj; 

    for (int i = 0; i < turtle.traj_data.num_waypoints; i++) {
        float x = turtle.traj_data.waypoints_x[i];
        float y = turtle.traj_data.waypoints_y[i];
        float v = turtle.traj_data.waypoints_v[i];
        waypoints_.push_back(Waypoint(x, y, v, 0.0f));
    }

    printf("Waypoints loaded:\n");
    for (size_t i = 0; i < waypoints_.size(); i++) {
        printf("Waypoint %zu: (%f, %f), vel: %f\n",
               i, waypoints_[i].point.x, waypoints_[i].point.y, waypoints_[i].vel);
    }
}

bool TrajectoriesParser::processWaypoint(turtle &turtle) {
    clock_now_ = chrono::steady_clock::now();
    t_ = chrono::duration<float>(clock_now_ - clock_start_).count();

    bool segment_complete = linearTraj(t_, curr_waypoint_.vel, 
                                       prev_waypoint_.point, curr_waypoint_.point, 
                                       gamma_, theta_);

    // Store the calculated values
    // turtle.turtle_control.right_adduction.set_input_position_radian.input_position = gamma_;
    // turtle.turtle_control.right_sweeping.set_input_position_radian.input_position = theta_;
    // Store the calculated values and convert to motor units (turns)
    turtle.turtle_control.right_adduction.set_input_position_radian.input_position = -gamma_ / (2 * M_PI);
    turtle.turtle_control.right_sweeping.set_input_position_radian.input_position = -theta_ / (2 * M_PI);

    // Return whether we've reached the waypoint
    return segment_complete;
}


bool TrajectoriesParser::goToInitialPoint(turtle &turtle) {
    if (waypoints_.empty()) {
        return true;
    }

    // Use a conservative velocity for the move-to-start segment
    const float v = std::min(goto_start_vel_, waypoints_[0].vel);

    clock_now_ = chrono::steady_clock::now();
    t_ = chrono::duration<float>(clock_now_ - clock_start_).count();

    const bool segment_complete = linearTraj(t_, v,
                                            prev_waypoint_.point, waypoints_[0].point,
                                            gamma_, theta_);

    // Command motors (ODrive input_pos is in turns)
    turtle.turtle_control.right_adduction.set_input_position_radian.input_position = -gamma_ / (2 * M_PI);
    turtle.turtle_control.right_sweeping.set_input_position_radian.input_position  = -theta_ / (2 * M_PI);

    return segment_complete;
}



bool TrajectoriesParser::waypointTrajectory(turtle &turtle) {
    if (first_iteration) {
    generateWaypoints(turtle);

    turtle.turtle_control.if_control = 1;

    // Pre-position: start from the *current* measured motor positions (turns -> radians)
    // ODrive pos_estimate is in turns. Your waypoint points are treated as radians (gamma/theta).
    const float curr_add_turns = turtle.turtle_chassis.right_adduction.pos_estimate;
    const float curr_swp_turns = turtle.turtle_chassis.right_sweeping.pos_estimate;

    const float curr_gamma_rad = -curr_add_turns * (2 * M_PI);
    const float curr_theta_rad = -curr_swp_turns * (2 * M_PI);

    // Reuse prev_waypoint_ as the "current state" start point for the pre-position segment
    prev_waypoint_ = Waypoint(curr_gamma_rad, curr_theta_rad, 0.0f, 0.0f);

    prog_state_ = ProgramState::GoToInitialPoint;
    first_iteration = false;

    clock_start_ = chrono::steady_clock::now();
    printf("Prepositioning to waypoint 0...\n");
    }

    // Phase 1: move smoothly to waypoint 0
    if (prog_state_ == ProgramState::GoToInitialPoint) {
        if (goToInitialPoint(turtle)) {
            if (waypoints_.size() < 2) {
                printf("Trajectory complete (only 1 waypoint).\n");
                return true;
            }

            // Phase 2: run the user trajectory from waypoint0 -> waypoint1 -> ...
            waypoint_index_ = 1;
            prev_waypoint_ = waypoints_[0];
            curr_waypoint_ = waypoints_[1];
            clock_start_ = chrono::steady_clock::now();
            prog_state_ = ProgramState::Running;

            printf("Reached waypoint 0. Starting waypoint trajectory...\n");
        }
        return false;
    }


    if (waypoint_index_ >= static_cast<int>(waypoints_.size())) {
        printf("Trajectory complete.\n");
        return true;
    }

    clock_now_ = chrono::steady_clock::now();
    
    if (processWaypoint(turtle)) {
        waypoint_index_++;
        if (waypoint_index_ < static_cast<int>(waypoints_.size())) {
            prev_waypoint_ = curr_waypoint_;
            curr_waypoint_ = waypoints_[waypoint_index_];
            clock_start_ = chrono::steady_clock::now();
            printf("Next Waypoint %d: (%f, %f)\n",
                   waypoint_index_, curr_waypoint_.point.x, curr_waypoint_.point.y);
        }
    }

    return false;
}

void TrajectoriesParser::generateTempTraj(turtle &turtle) {
    int trajectory = turtle.turtle_gui.drag_traj; 
    int RUN = turtle.turtle_gui.start_flag; 
    
    if (!turtle.turtle_gui.start_flag) {
        first_iteration = true;
        prog_state_ = ProgramState::FirstIteration;
        waypoint_index_ = 0;
        waypoints_.clear();
        turtle.turtle_control.if_control = 0;  
        return;
    }

    waypointTrajectory(turtle);
}

} // namespace control
} // namespace turtle_namespace

