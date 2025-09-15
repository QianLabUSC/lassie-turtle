/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-27 16:00:54 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2022-02-02 19:08:47
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "proxy/lowerproxy.h"
#include "proxy/upperproxy.h"
#include "controller/trajectories_parser.h"
#include "proxy/control_data.h"
#include "can_driver/can_driver.hpp"
#include "can_driver/can_suber.hpp"
#include "traveler_msgs/msg/set_input_position.hpp"
#include <chrono>

using namespace std;

using turtle_namespace::control::lowerproxy;
using turtle_namespace::control::upperproxy;
using turtle_namespace::control::TrajectoriesParser;

// Remove unused variables to eliminate warnings
// static float init_cnt = 0;
// static int init_done = 0;
// static float timer[10] = { 0 };

int i = 0;
int TIME_STEP = 5;
float dT = TIME_STEP / 1000.0f;

turtle turtle_; 

#endif

