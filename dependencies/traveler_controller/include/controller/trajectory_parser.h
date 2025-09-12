#ifndef TRAJECTRIES_PARSER_
#define TRAJECTRIES_PARSER_

#include <vector>
#include <chrono>
#include <cstdint>
#include <iostream>

#include "proxy/control_data.h"
#include "controller/inverse_kinematics.h"

namespace turtle_namespace {
namespace control {

/**
 * Waypoints-only TrajectoriesParser
 * - Runs linear segments between user-provided (x,y,vel) waypoints
 * - Starts from current toe pose -> first waypoint
 * - No modes, no legacy phases
 */
class TrajectoriesParser {
public:
    static TrajectoriesParser& getTrajParser() {
        static TrajectoriesParser singleton;
        return singleton;
    }

    void init();
    void generateTempTraj(turtle&);

    /**
     * @brief Uses internal theta_ and gamma_ to set motor positions.
     *        (theta_, gamma_ are updated by cartesianMotorCommand via physicalToAbstract)
     */
    void setCoupledPosition(turtle& turtle_);

    /**
     * @brief Commands the motors to send the toe to a given (target_x, target_y)
     *        in cartesian space (meters). Handles IK + motor coupling.
     */
    void cartesianMotorCommand(turtle&, float target_x, float target_y);

    /**
     * @brief Takes the toe through the internal waypoint list.
     * @returns True when the full list is complete, false while in progress.
     */
    bool waypointTrajectory(turtle&);

    /**
     * @brief Fills the internal waypoint vector from turtle.traj_data.{waypoints_x,y,v}
     */
    void generateWaypoints(turtle&);

    /**
     * @brief Executes one segment (prev -> curr waypoint). Handles linear interpolation and dwell.
     * @returns True when this segment completes (arrived + dwell elapsed), else false.
     */
    bool processWaypoint(turtle&);

    /**
     * @brief Debug print of key trajectory GUI fields & count of waypoints.
     */
    void printTrajData(turtle&);

private:
    // --- IK / motor command scratch ---
    float target_x = 0.0f;
    float target_y = 0.0f;
    float theta_   = M_PI;      // abstract leg angle (updated by physicalToAbstract)
    float gamma_   = 0.0f;      // motor separation angle (updated by physicalToAbstract)

    // --- general state ---
    bool     first_iteration = false;
    bool     E_STOP          = false;
    uint8_t  state_flag_     = 0;
    float    t_              = 0.0f; // seconds since start of current segment

    // clocks
    std::chrono::steady_clock::time_point clock_start_;
    std::chrono::steady_clock::time_point clock_now_;

    // toe position bookkeeping (optional, used for simple obstruction heuristics if desired)
    XY_pair curr_toe_pos{};
    XY_pair prev_toe_pos{};
    float   Move_Dist = 0.0f;

    // --- waypoint state ---
    std::vector<Waypoint> waypoints_;
    Waypoint curr_waypoint_{};
    Waypoint prev_waypoint_{};
    int      waypoint_index_ = 0;   // which waypoint we’re headed to
    int      waypoint_state_ = 0;   // 0:setup, 1:travel, 2:dwell
    bool     traj_complete_  = false;

    // disallow external construction
    TrajectoriesParser() = default;
    TrajectoriesParser(const TrajectoriesParser&) = delete;
    TrajectoriesParser& operator=(const TrajectoriesParser&) = delete;
};

} // namespace control
} // namespace turtle_namespace

#endif // TRAJECTRIES_PARSER_
