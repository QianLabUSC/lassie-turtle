
#ifndef TRAJECTRIES_PARSER_
#define TRAJECTRIES_PARSER_

#include <vector>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <cmath>

#include "proxy/control_data.h"
#include "controller/inverse_kinematics.h"

namespace turtle_namespace {
namespace control {

/**
 * Waypoints-only TrajectoriesParser
 * - Runs linear segments between user-provided (x,y,vel) waypoints
 * - Starts at the first waypoint and marches through the list
 */
class TrajectoriesParser {
public:
  // Make default ctor public so existing main.cpp can instantiate it.
  TrajectoriesParser() = default;

  // (optional singleton, still available if you ever want it)
  static TrajectoriesParser& getTrajParser() {
    static TrajectoriesParser singleton;
    return singleton;
  }

  void init();
  void generateTempTraj(turtle&);

  // Uses internal theta_ and gamma_ to set motor positions.
  void setCoupledPosition(turtle& turtle_);

  // Command motors to cartesian target (meters)
  void cartesianMotorCommand(turtle&, float target_x, float target_y);

  // Run through internal waypoint list; true when finished
  bool waypointTrajectory(turtle&);

  // Fill internal waypoint vector from turtle.traj_data.{waypoints_x,y,v}
  void generateWaypoints(turtle&);

  // Execute one segment (prev -> curr). True when segment completes.
  bool processWaypoint(turtle&);

  // Debug print
  void printTrajData(turtle&);

private:
  // --- IK / motor command scratch ---
  float target_x = 0.0f;
  float target_y = 0.0f;
  float theta_   = 0.0f;  // updated by physicalToAbstract
  float gamma_   = 0.0f;  // updated by physicalToAbstract

  // --- general state ---
  bool     first_iteration = true;
  bool     E_STOP          = false;
  uint8_t  state_flag_     = 0;
  float    t_              = 0.0f; // seconds since start of current segment

  // clocks
  std::chrono::steady_clock::time_point clock_start_;
  std::chrono::steady_clock::time_point clock_now_;

  // --- waypoint state ---
  std::vector<Waypoint> waypoints_;
  Waypoint curr_waypoint_{};
  Waypoint prev_waypoint_{};
  int      waypoint_index_ = 0;   // which waypoint we’re headed to
  int      waypoint_state_ = 0;   // 0:setup, 1:travel, 2:dwell
  bool     traj_complete_  = false;

  // non-copyable
  TrajectoriesParser(const TrajectoriesParser&) = delete;
  TrajectoriesParser& operator=(const TrajectoriesParser&) = delete;
};

} // namespace control
} // namespace turtle_namespace

#endif // TRAJECTRIES_PARSER_




