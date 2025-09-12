// trajectories_parser.cpp — Waypoints-only version

#include "controller/trajectories_parser.h"
#include <chrono>
#include <cmath>
#include <iostream>

using namespace std;

namespace turtle_namespace {
namespace control {

void TrajectoriesParser::init() {}

// -------------------------------------------------------
// Motor commanding helpers (kept)
// -------------------------------------------------------
void TrajectoriesParser::setCoupledPosition(turtle &turtle)
{
    float axis_0 = theta_ - gamma_;
    float axis_1 = theta_ + gamma_;
    turtle.turtle_control.left_adduction.set_input_position_radian.input_position = axis_0;
    turtle.turtle_control.left_sweeping.set_input_position_radian.input_position  = axis_1;
}

void TrajectoriesParser::cartesianMotorCommand(turtle &turtle, float target_x, float target_y)
{
    // Uses inverse_kinematics: physicalToAbstract(X,Y, theta_, gamma_, clamp=true)
    physicalToAbstract(target_x, target_y, theta_, gamma_, true);
    setCoupledPosition(turtle);
}

// -------------------------------------------------------
// Waypoint execution
// -------------------------------------------------------
bool TrajectoriesParser::waypointTrajectory(turtle &turtle)
{
    if (first_iteration) {
        generateWaypoints(turtle);
        clock_start_     = chrono::steady_clock::now();
        waypoint_index_  = 0;
        waypoint_state_  = 0;
        state_flag_      = 0;
        traj_complete_   = false;

        // Start from current toe toward first user waypoint
        prev_waypoint_ = Waypoint(turtle.turtle_chassis.Leg_lf.toe_position, 0.0f, 0.0f);
        if (!waypoints_.empty()) {
            curr_waypoint_ = waypoints_[waypoint_index_];
        }

        printf("Waypoint Trajectory Initialized\n");
        printf("Previous Waypoint : (%f, %f)\n",
               prev_waypoint_.point.x, prev_waypoint_.point.y);
        if (!waypoints_.empty()) {
            printf("Current Waypoint 0: (%f, %f)\n",
                   curr_waypoint_.point.x, curr_waypoint_.point.y);
        }
    }

    if (traj_complete_) {
        return true;
    }

    // Step through the current segment; advance on completion
    if (processWaypoint(turtle)) {
        waypoint_index_++;
        state_flag_ = waypoint_index_;  // optional: expose which waypoint we're heading to
        turtle.turtle_chassis.Leg_lf.state_flag = state_flag_;

        if (waypoint_index_ < waypoints_.size()) {
            prev_waypoint_ = curr_waypoint_;
            curr_waypoint_ = waypoints_[waypoint_index_];
            printf("Current Waypoint %d: (%f, %f)\n",
                   waypoint_index_, curr_waypoint_.point.x, curr_waypoint_.point.y);
        } else {
            printf("Waypoint Trajectory Complete\n");
            traj_complete_ = true;
            return true;
        }
    }
    return false;
}

bool TrajectoriesParser::processWaypoint(turtle &turtle)
{
    // time since this waypoint segment started
    t_ = chrono::duration<float>(clock_now_ - clock_start_).count();

    switch (waypoint_state_) {
        case 0: {
            // Clamp requested point into reachable workspace (function from IK module)
            clamp_XY(curr_waypoint_.point);
            clock_start_     = chrono::steady_clock::now();
            waypoint_state_  = 1;
            return false;
        }
        case 1: {
            // Linear trajectory from prev -> curr at curr_waypoint_.vel
            if (linearTraj(t_, curr_waypoint_.vel,
                           prev_waypoint_.point, curr_waypoint_.point,
                           target_x, target_y))
            {
                // Snap exactly to target once complete
                target_x = curr_waypoint_.point.x;
                target_y = curr_waypoint_.point.y;
                cartesianMotorCommand(turtle, target_x, target_y);

                waypoint_state_ = 2;                  // optional dwell
                clock_start_     = chrono::steady_clock::now();
                return false;
            }

            // Keep commanding along the line
            cartesianMotorCommand(turtle, target_x, target_y);
            return false;
        }
        case 2: {
            // Dwell at waypoint (delay may be zero)
            if (t_ > curr_waypoint_.delay) {
                waypoint_state_ = 0;
                return true;                          // segment complete
            }
            return false;
        }
    }
    return false;
}

// -------------------------------------------------------
// Waypoint ingestion (NO MODES)
// Fill internal vector from turtle.traj_data
// -------------------------------------------------------
void TrajectoriesParser::generateWaypoints(turtle &turtle)
{
    waypoints_.clear();

    for (int i = 0; i < turtle.traj_data.num_waypoints; ++i) {
        float x = turtle.traj_data.waypoints_x[i];
        float y = turtle.traj_data.waypoints_y[i];
        float v = turtle.traj_data.waypoints_v[i];
        waypoints_.push_back(Waypoint(x, y, v, 0.0f)); // zero delay between points
    }

    printf("Waypoints:\n");
    for (int i = 0; i < static_cast<int>(waypoints_.size()); ++i) {
        printf("  %d: (%f, %f), vel: %f, delay: %f\n",
               i, waypoints_[i].point.x, waypoints_[i].point.y,
               waypoints_[i].vel, waypoints_[i].delay);
    }
}

// -------------------------------------------------------
// Main tick: when start_flag is true, run waypoints
// -------------------------------------------------------
void TrajectoriesParser::generateTempTraj(turtle &turtle)
{
    const int RUN = turtle.turtle_gui.start_flag;

    // Bookkeeping (harmless)
    prev_toe_pos = curr_toe_pos;
    curr_toe_pos = turtle.turtle_chassis.Leg_lf.toe_position;
    Move_Dist    = distance(prev_toe_pos, curr_toe_pos);

    if (!RUN) {
        // Reset minimal state used by the waypoint runner
        first_iteration  = true;
        traj_complete_   = false;
        waypoint_state_  = 0;
        waypoint_index_  = 0;
        E_STOP           = false;
        return;
    } else if (E_STOP) {
        printf("============ E-STOPPED ============\n");
        return;
    }

    // Tick wall clock for the current segment
    clock_now_ = chrono::steady_clock::now();

    if (first_iteration) {
        printf("General Trajectory (waypoints-only)\n");
        printf("Starting Toe Position: (%f, %f)\n",
               turtle.turtle_chassis.Leg_lf.toe_position.x,
               turtle.turtle_chassis.Leg_lf.toe_position.y);
    }

    waypointTrajectory(turtle);

    // Legacy bookkeeping (safe to keep; unused elsewhere)
    turtle.traj_data.current_t += 0.01000f;

    if (RUN && first_iteration) {
        first_iteration = false;
    }
}

// -------------------------------------------------------
// Optional: prints (renamed fields you mentioned)
// -------------------------------------------------------
void TrajectoriesParser::printTrajData(turtle &turtle)
{
    printf("lateral_angle_range: %f\n", turtle.traj_data.lateral_angle_range);
    printf("drag_speed:          %f\n", turtle.traj_data.drag_speed);
    printf("wiggle_time:         %f\n", turtle.traj_data.wiggle_time);
    printf("servo_speed:         %f\n", turtle.traj_data.servo_speed);
    printf("extraction_angle:    %f\n", turtle.traj_data.extraction_angle);
    printf("wiggle_frequency:    %f\n", turtle.traj_data.wiggle_frequency);
    printf("insertion_depth:     %f\n", turtle.traj_data.insertion_depth);
    printf("wiggle_amptitude:    %f\n", turtle.traj_data.wiggle_amptitude);
    printf("num_waypoints:       %d\n", turtle.traj_data.num_waypoints);
}

} // namespace control
} // namespace turtle_namespace
