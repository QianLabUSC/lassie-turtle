// src/controller/trajectories_parser.cpp

#include "controller/trajectories_parser.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace turtle_namespace {
namespace control {

namespace {
// simple time-based linear step: move from A -> B at constant speed "vel" (m/s)
static bool linearStep(float t_rel, float vel,
                       const XY_pair& A, const XY_pair& B,
                       float& X, float& Y)
{
    const float dx = B.x - A.x;
    const float dy = B.y - A.y;
    const float dist = std::sqrt(dx*dx + dy*dy);

    if (dist < 1e-6f || vel <= 0.0f) {  // nothing to do
        X = B.x; Y = B.y;
        return true;
    }

    float s = (t_rel * vel) / dist;             // normalized progress
    // s = std::clamp(s, 0.0f, 1.0f);

    X = A.x + s * dx;
    Y = A.y + s * dy;
    return (s >= 1.0f);
}

} // anon ns

void TrajectoriesParser::init() {}

void TrajectoriesParser::setCoupledPosition(turtle &turtle)
{
    // radians -> turns 
    const float axis_0_turns = (theta_ - gamma_) / (2.0f * M_PI);
    const float axis_1_turns = (theta_ + gamma_) / (2.0f * M_PI);

    turtle.turtle_control.left_adduction.set_input_position_radian.input_position = axis_0_turns;
    turtle.turtle_control.left_sweeping.set_input_position_radian.input_position  = axis_1_turns;
}

void TrajectoriesParser::cartesianMotorCommand(turtle &turtle, float target_x, float target_y)
{
    physicalToAbstract(target_x, target_y, theta_, gamma_, true);
    setCoupledPosition(turtle); 
}

bool TrajectoriesParser::waypointTrajectory(turtle &turtle)
{
    if (first_iteration) {
        generateWaypoints(turtle);

        // seed prev/curr waypoints
        waypoint_index_  = 0;
        waypoint_state_  = 0;
        traj_complete_   = false;

        if (waypoints_.empty()) {
            traj_complete_ = true;
            return true;
        }

        prev_waypoint_ = waypoints_[0];
        if (waypoints_.size() > 1) {
            curr_waypoint_ = waypoints_[1];
            waypoint_index_ = 1;
        } else {
            curr_waypoint_ = waypoints_[0];
            waypoint_index_ = 0;
        }

        clock_start_ = std::chrono::steady_clock::now();
        std::printf("Waypoint Trajectory Initialized with %zu points\n", waypoints_.size());
    }

    if (traj_complete_) return true;

    if (processWaypoint(turtle)) {
        // Advance to next segment
        if (++waypoint_index_ < static_cast<int>(waypoints_.size())) {
            prev_waypoint_ = curr_waypoint_;
            curr_waypoint_ = waypoints_[waypoint_index_];
            clock_start_   = std::chrono::steady_clock::now();
        } else {
            std::printf("Waypoint Trajectory Complete\n");
            traj_complete_ = true;
            return true;
        }
    }
    return false;
}

bool TrajectoriesParser::processWaypoint(turtle &turtle)
{
    clock_now_ = std::chrono::steady_clock::now();
    t_ = std::chrono::duration<float>(clock_now_ - clock_start_).count();

    switch (waypoint_state_) {
        case 0: {
            // start this segment
            waypoint_state_ = 1;
            return false;
        }
        case 1: {
            if (linearStep(t_, curr_waypoint_.vel, prev_waypoint_.point, curr_waypoint_.point, target_x, target_y)) {
                // snap & optional dwell
                target_x = curr_waypoint_.point.x;
                target_y = curr_waypoint_.point.y;
                cartesianMotorCommand(turtle, target_x, target_y);
                waypoint_state_ = 2;
                clock_start_ = std::chrono::steady_clock::now();
                return false;
            }
            // continue along the line
            cartesianMotorCommand(turtle, target_x, target_y);
            return false;
        }
        case 2: {
            // dwell (if you ever add nonzero delay)
            const float t_dwell = std::chrono::duration<float>(clock_now_ - clock_start_).count();
            if (t_dwell > curr_waypoint_.delay) {
                waypoint_state_ = 0;
                return true; // segment complete
            }
            return false;
        }
    }
    return false;
}

void TrajectoriesParser::generateWaypoints(turtle &turtle)
{
    waypoints_.clear();
    const auto& td = turtle.traj_data;

    for (int i = 0; i < td.num_waypoints; ++i) {
        const float x = td.waypoints_x[i];
        const float y = td.waypoints_y[i];
        const float v = td.waypoints_v[i];
        waypoints_.push_back(Waypoint(x, y, v, 0.0f));
    }

    std::printf("Waypoints loaded: %zu\n", waypoints_.size());
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        std::printf("  %zu: (%.3f, %.3f), v=%.3f\n",
            i, waypoints_[i].point.x, waypoints_[i].point.y, waypoints_[i].vel);
    }
}

void TrajectoriesParser::generateTempTraj(turtle &turtle)
{
    const bool RUN = (turtle.turtle_gui.start_flag != 0);

    if (!RUN) {
        first_iteration = true;
        traj_complete_  = false;
        waypoint_state_ = 0;
        waypoint_index_ = 0;
        E_STOP          = false;
        return;
    }

    if (first_iteration) {
        std::printf("Trajectory: waypoints-only (no clamp / no IK)\n");
    }

    waypointTrajectory(turtle);

    if (RUN && first_iteration) {
        first_iteration = false;
    }
}

void TrajectoriesParser::printTrajData(turtle &turtle)
{
    const auto& td = turtle.traj_data;
    std::printf("sweeping_range:        %f\n", td.sweeping_range);
    std::printf("insertion_depth:       %f\n", td.insertion_depth);
    std::printf("penetration_velocity:  %f\n", td.penetration_velocity);
    std::printf("sweeping_velocity:     %f\n", td.sweeping_velocity);
    std::printf("extraction_velocity:   %f\n", td.extraction_velocity);
    std::printf("swing_velocity:        %f\n", td.swing_velocity);
}

} // namespace control
} // namespace turtle_namespace



