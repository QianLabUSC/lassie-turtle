/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-27 16:00:54 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2022-02-02 19:08:47
 */


#ifndef MAIN_H_
#define MAIN_H_

#include "controller/controller_monitor.h"
#include "controller/pid_speed_controller.h"
#include "proxy/lowerproxy.h"
#include "proxy/upperproxy.h"
#include "proxy/control_data.h"
#include "can_driver/can_driver.hpp"
#include "can_driver/can_suber.hpp"
#include "traveler_msgs/msg/set_input_position.hpp"
#include <chrono>
//#include <Eigen/Dense>
//#include <qpOASES.hpp>
//#include <glog/logging.h>
// #include "wbInterface.h"
// #include "optimaize.h"
// #include "locomotion_header.h"
// #include "gait_math.h"
// #include "adrc.h"
// #include "convexMPC_interface.h"
// #include "common_types.h"
// #include "SolverMPC.h"
// #include "cppTypes.h"


// using namespace Eigen;
using namespace std;
// using namespace qpOASES;

using turtle_namespace::control::lowerproxy;
using turtle_namespace::control::upperproxy;
// using turtle_namespace::control::visualizor;
using turtle_namespace::control::ControllerMonitor;

static float init_cnt = 0;
static int init_done = 0;
int i = 0;
static float timer[10] = { 0 };
int TIME_STEP = 5;
float dT = TIME_STEP / 1000.;
turtle  turtle_; 

#endif
