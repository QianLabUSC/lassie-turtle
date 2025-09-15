#include "controller/trajectories_parser.h"
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
    for (int i = 0; i < turtle.traj_data.num_waypoints; i++) {
        float x = turtle.traj_data.waypoints_x[i];
        float y = turtle.traj_data.waypoints_y[i];
        float v = turtle.traj_data.waypoints_v[i];
        waypoints_.push_back(Waypoint(x, y, v, 0.0f));
    }

    printf("Waypoints loaded:\n");
    for (int i = 0; i < waypoints_.size(); i++) {
        printf("Waypoint %d: (%f, %f), vel: %f\n",
               i, waypoints_[i].point.x, waypoints_[i].point.y, waypoints_[i].vel);
    }
}

bool TrajectoriesParser::processWaypoint(turtle &turtle) {
    t_ = chrono::duration<float>(clock_now_ - clock_start_).count();

    if (linearTraj(t_, curr_waypoint_.vel,
                   prev_waypoint_.point, curr_waypoint_.point,
                   target_x, target_y)) {
        return true; // reached waypoint
    }

    cartesianMotorCommand(turtle, target_x, target_y);
    return false;
}

bool TrajectoriesParser::waypointTrajectory(turtle &turtle) {
    if (first_iteration) {
        generateWaypoints(turtle);
        waypoint_index_ = 0;
        prev_waypoint_ = Waypoint(turtle.turtle_chassis.Leg_lf.toe_position, 0.0f, 0.0f);
        curr_waypoint_ = waypoints_[waypoint_index_];
        first_iteration = false;
        printf("Starting waypoint trajectory...\n");
    }

    if (waypoint_index_ >= waypoints_.size()) {
        printf("Trajectory complete.\n");
        return true;
    }

    if (processWaypoint(turtle)) {
        waypoint_index_++;
        if (waypoint_index_ < waypoints_.size()) {
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
    if (!turtle.turtle_gui.start_flag) {
        first_iteration = true;
        waypoint_index_ = 0;
        return;
    }

    clock_now_ = chrono::steady_clock::now();
    waypointTrajectory(turtle);
}

} // namespace control
} // namespace turtle_namespace
