#ifndef TRAJECTORIES_PARSER_
#define TRAJECTORIES_PARSER_

#include <iostream>
#include <vector>
#include <chrono>
#include "proxy/control_data.h"
#include "controller/inverse_kinematics.h"

namespace turtle_namespace {
namespace control {

class TrajectoriesParser
{
public:
    static TrajectoriesParser &getTrajParser()
    {
        static TrajectoriesParser singleton;
        return singleton;
    }

    void generateTempTraj(turtle &);
    void cartesianMotorCommand(turtle &, float target_x, float target_y);

    bool waypointTrajectory(turtle &);
    void generateWaypoints(turtle &);
    bool processWaypoint(turtle &);
    bool trajComplete() const { return traj_complete_; }

private:
    // state for linear trajectory timing
    std::chrono::steady_clock::time_point clock_start_;
    std::chrono::steady_clock::time_point clock_now_;
    float t_ = 0.0f;

    // current abstract parameters
    float theta_ = 0.0f;
    float gamma_ = 0.0f;

    // waypoint management
    std::vector<Waypoint> waypoints_;
    Waypoint curr_waypoint_;
    Waypoint prev_waypoint_;
    int waypoint_index_ = 0;
    bool first_iteration = true;
    bool traj_complete_ = false;

    // cartesian interpolation targets
    float target_x = 0.0f;
    float target_y = 0.0f;

    int last_trajectory_version_ = 0;

};

} // namespace control
} // namespace turtle_namespace

#endif

