#include "controller/inverse_kinematics.h"

#define _USE_MATH_DEFINES
#include <fstream>
#include<iostream>
#include <cmath>
using namespace std;

#define DEBUG

/**
 * ! Leg Workspace:
 * Gamma is the rotation angle of odrive motor and must be within [-0.79 0.79]
 * Theta is the big servo angle and must be within [-1.05, +1.05] radians
 * Beta is the small servo angle and must be within [-1.05, +1.05] radians
 */

/**
 * @brief Finds the motor and server control command for a given toe position
 *
 * @param X : Toe X position
 * @param Y : Toe Y position
 *
 * @return : Returns <theta, gamma , beta>
 */







// Fixed insertion depth verson3 analytic solution with matlab homogeneous matrix verified

// void fixed_insertion_depth_gait_lower_point_version_3_analytic_solution(turtle& turtle_, float t){
    
    
//     //Fixed insertion depth parameters and gamma solver paramters
    
//     double l1=0.145; // flipper length-new shorter flipper
//     double turtle_height=0.079; //height from flipper to fround
//     double lower_point=0.053;
    
//      //get data this is slow version 
//      float horizontal_angle = turtle_.traj_data.lateral_angle_range*180/M_PI;
//     Rectangle_Params rectangle_params;
//     rectangle_params.period_down = turtle_.traj_data.lateral_angle_range * l1 * 2/turtle_.traj_data.drag_speed;
//     rectangle_params.period_up = 0.8; //customize back phase time
//     rectangle_params.period_left = turtle_.traj_data.servo_speed;
//     rectangle_params.period_right = turtle_.traj_data.servo_speed;
//     rectangle_params.vertical_range = turtle_.traj_data.insertion_depth; // depend on insertion depth
//     rectangle_params.horizontal_range = turtle_.traj_data.lateral_angle_range*180/M_PI;
//     rectangle_params.period_waiting_time = 0;
//     float t_mod = fmod(t, (rectangle_params.period_down + rectangle_params.period_up + rectangle_params.period_left + rectangle_params.period_right + rectangle_params.period_waiting_time));
//  turtle_.turtle_chassis.step_count=(t-t_mod)/ (rectangle_params.period_down + rectangle_params.period_up + rectangle_params.period_left + rectangle_params.period_right + rectangle_params.period_waiting_time);
    
//     double desierd_insertion_depth=turtle_.traj_data.insertion_depth;



    
//     float corres_t = 0;
//     float left_hori_servo = 0;// original 100 mannually tuned to 94
//     float right_hori_servo = 0;
//     float extraction_angle = turtle_.traj_data.extraction_angle;




//     // desired insertion depth input

 
//     if (desierd_insertion_depth>0.07)
//    { desierd_insertion_depth=0.07;
//    }
    

//      cout << "desired insertion depth(m)"<<  desierd_insertion_depth<< endl;// m
//     // fixed insertion depth initial calculation
//     double initial_insertion_depth_rad=asin((desierd_insertion_depth+turtle_height)/sqrt((l1*cos(horizontal_angle*M_PI/180))*(l1*cos(horizontal_angle*M_PI/180))+lower_point*lower_point))-atan(lower_point/(l1*cos(horizontal_angle*M_PI/180)));
//     double initial_insertion_depth_deg=initial_insertion_depth_rad*180/M_PI;
//     cout << "TMOD" << t_mod << endl;
//    // cout<<"Initial angle"<<initial_insertion_depth_deg<<endl;
//     // insertion_depth=insertion_depth+10
//     double gamma1 = 0;
//     double theta1 = 0;
//     double gamma2 = 0;
//     double theta2 = 0;
//     //FIRST phase gamma=0 theta=-45
//     if (t_mod <= rectangle_params.period_up)
//     {
//         corres_t = t_mod / rectangle_params.period_up;
//         gamma1 = left_hori_servo + extraction_angle;
//         theta1 = - horizontal_angle + 2*horizontal_angle*corres_t;
//         gamma2 = right_hori_servo - extraction_angle;
//         theta2 = horizontal_angle - 2*horizontal_angle*corres_t;
//         turtle_.turtle_chassis.gait_state = 1;
//     }
//     //Phase2  theta=-45 gamma go to desired value
//     else if (t_mod <= rectangle_params.period_up + rectangle_params.period_right+3.0)
//     {
//         corres_t = (t_mod - rectangle_params.period_up) / rectangle_params.period_right;
//         theta1 = horizontal_angle;
//         gamma1 = left_hori_servo + extraction_angle - 
//                     (initial_insertion_depth_rad*180/M_PI + extraction_angle) * corres_t;
//         theta2 = -horizontal_angle;
//         gamma2 = right_hori_servo - extraction_angle + 
//                     (initial_insertion_depth_rad*180/M_PI + extraction_angle) * corres_t;
//         turtle_.turtle_chassis.gait_state = 2;
//          std::cout << "Initial Adduction" << gamma1 << std::endl;
//     }

//     //
//     else if (t_mod <= rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down)
//     {
//         corres_t = (t_mod - rectangle_params.period_up - rectangle_params.period_right) / rectangle_params.period_down;
        
//         theta1 = horizontal_angle - 2*horizontal_angle*corres_t;
//         gamma1 = left_hori_servo -(asin((desierd_insertion_depth+turtle_height)/sqrt((l1*cos(-theta1*M_PI/180))*(l1*cos(-theta1*M_PI/180))+lower_point*lower_point))-atan(lower_point/(l1*cos(-theta1*M_PI/180))))*180/M_PI;
//         theta2 = -horizontal_angle + 2*horizontal_angle*corres_t;
//         gamma2 = right_hori_servo + (asin((desierd_insertion_depth+turtle_height)/sqrt((l1*cos(-theta1*M_PI/180))*(l1*cos(-theta1*M_PI/180))+lower_point*lower_point))-atan(lower_point/(l1*cos(-theta1*M_PI/180))))*180/M_PI;
//         turtle_.turtle_chassis.gait_state = 3;
//          std::cout << "Phase3 Adduction" << gamma1 << std::endl;
//     }

//     else{
//         corres_t = (t_mod - rectangle_params.period_up - rectangle_params.period_right - rectangle_params.period_down) / rectangle_params.period_left;
//         theta1 = -horizontal_angle;
//         //cout << "LEFT corres_t" << corres_t << endl;
//         gamma1 = left_hori_servo -  (initial_insertion_depth_rad*180/M_PI) +(initial_insertion_depth_rad*180/M_PI + extraction_angle)* corres_t;
//         theta2 = horizontal_angle;
//         gamma2 = right_hori_servo + (initial_insertion_depth_rad*180/M_PI) - (initial_insertion_depth_rad*180/M_PI + extraction_angle) * corres_t;
//         turtle_.turtle_chassis.gait_state = 4;
//        //  cout << "extract"  << endl;
//     }
//     // Stage 5: Hold position for 2 seconds after stage 4 rotation
//     // else if (t_mod <= rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down + rectangle_params.period_left + 2.0)
//     // {
//     //     // During stage 5, maintain the positions achieved at the end of stage 4
//     //     gamma1 = left_hori_servo - (initial_insertion_depth_rad * 180 / M_PI) + (initial_insertion_depth_rad * 180 / M_PI + extraction_angle);
//     //     gamma2 = right_hori_servo + (initial_insertion_depth_rad * 180 / M_PI) - (initial_insertion_depth_rad * 180 / M_PI + extraction_angle);

//     //     // Set gait state to indicate stage 5
//     //     turtle_.turtle_chassis.gait_state = 5;
//     // }
//     //gamma theta in deg

//   //adduction angle: left gamma: gamma2,right gamma:gamma2
    

//       // sweeping angle:    right theta: theta2 left theta   :theta1
//     turtle_.turtle_control.left_adduction.set_input_position_degree.input_position = gamma1;
//     turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position = theta1;
//     turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = gamma2;
//     turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = theta2;
//     turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = -gamma1/360;
//     turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = -theta1/360;
//     turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = -gamma2/360;
//     turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = -theta2/360;
//     // used for test
// }

void fixed_insertion_depth_gait_lower_point_version_3_analytic_solution(turtle& turtle_, float t){

    //Fixed insertion depth parameters and gamma solver parameters
    double l1 = 0.120; // flipper length-new shorter flipper
    double turtle_height = 0.0645; //height from flipper to ground
    double lower_point = 0.05;

    //get data this is slow version 
    float horizontal_angle = turtle_.traj_data.lateral_angle_range * 180 / M_PI;
    Rectangle_Params rectangle_params;
    rectangle_params.period_down = turtle_.traj_data.lateral_angle_range * l1 * 2 / turtle_.traj_data.drag_speed;
    rectangle_params.period_up = 0.8; //customize back phase time
    rectangle_params.period_left = turtle_.traj_data.servo_speed;
    rectangle_params.period_right = turtle_.traj_data.servo_speed;
    rectangle_params.vertical_range = turtle_.traj_data.insertion_depth; // depend on insertion depth
    rectangle_params.horizontal_range = turtle_.traj_data.lateral_angle_range * 180 / M_PI;
    rectangle_params.period_waiting_time = 0;

    float hold_time_1 = 0; // 新阶段1的保持时间
    float hold_time_2 = 0; // 新阶段2的保持时间
    float hold_time_3 = 0; // 新阶段3的保持时间
    float end_delay = 0;   // 延迟时间在结束阶段

    float total_period = rectangle_params.period_down + rectangle_params.period_up + rectangle_params.period_left + rectangle_params.period_right + hold_time_1 + hold_time_2 + hold_time_3 + end_delay + rectangle_params.period_waiting_time;

    float t_mod = fmod(t, total_period);
    turtle_.turtle_chassis.step_count = (t - t_mod) / total_period;

    double desierd_insertion_depth = turtle_.traj_data.insertion_depth;

    if (desierd_insertion_depth > 0.07) {
        desierd_insertion_depth = 0.07;
    }

    cout << "desired insertion depth(m)" << desierd_insertion_depth << endl; // m

    // fixed insertion depth initial calculation
    double initial_insertion_depth_rad = asin((desierd_insertion_depth + turtle_height) / sqrt((l1 * cos(horizontal_angle * M_PI / 180)) * (l1 * cos(horizontal_angle * M_PI / 180)) + lower_point * lower_point)) - atan(lower_point / (l1 * cos(horizontal_angle * M_PI / 180)));
    double initial_insertion_depth_deg = initial_insertion_depth_rad * 180 / M_PI;
    cout << "TMOD" << t_mod << endl;

    // 声明变量
    float corres_t = 0;
    float left_hori_servo = 0; // 原始值 100 手动调整为 94
    float right_hori_servo = 0;
    float extraction_angle = turtle_.traj_data.extraction_angle;

    double gamma1 = 0;
    double theta1 = 0;
    double gamma2 = 0;
    double theta2 = 0;

    // FIRST phase gamma=0 theta=-45
    if (t_mod <= rectangle_params.period_up) {
        corres_t = t_mod / rectangle_params.period_up;
        gamma1 = left_hori_servo + extraction_angle;
        theta1 = - horizontal_angle + 2 * horizontal_angle * corres_t;
        gamma2 = right_hori_servo - extraction_angle;
        theta2 = horizontal_angle - 2 * horizontal_angle * corres_t;
        turtle_.turtle_chassis.gait_state = 1;
    }
    // 新阶段 3：保持位置 3 秒
    else if (t_mod <= rectangle_params.period_up + hold_time_3) {
        gamma1 = left_hori_servo + extraction_angle;
        theta1 = horizontal_angle;
        gamma2 = right_hori_servo - extraction_angle;
        theta2 = -horizontal_angle;
        turtle_.turtle_chassis.gait_state = 7; // 新的 gait_state
    }
    // Phase 2 theta=-45 gamma go to desired value
    else if (t_mod <= rectangle_params.period_up + hold_time_3 + rectangle_params.period_right) {
        corres_t = (t_mod - rectangle_params.period_up - hold_time_3) / rectangle_params.period_right;
        theta1 = horizontal_angle;
        gamma1 = left_hori_servo + extraction_angle - (initial_insertion_depth_rad * 180 / M_PI + extraction_angle) * corres_t;
        theta2 = -horizontal_angle;
        gamma2 = right_hori_servo - extraction_angle + (initial_insertion_depth_rad * 180 / M_PI + extraction_angle) * corres_t;
        turtle_.turtle_chassis.gait_state = 2;
        std::cout << "Initial Adduction" << gamma1 << std::endl;
    }
    // 新阶段 1：保持位置 3 秒
    else if (t_mod <= rectangle_params.period_up + hold_time_3 + rectangle_params.period_right + hold_time_1) {
        gamma1 = left_hori_servo + extraction_angle - (initial_insertion_depth_rad * 180 / M_PI + extraction_angle);
        theta1 = horizontal_angle;
        gamma2 = right_hori_servo - extraction_angle + (initial_insertion_depth_rad * 180 / M_PI + extraction_angle);
        theta2 = -horizontal_angle;
        turtle_.turtle_chassis.gait_state = 5; // 新的 gait_state
    }
    // Phase 3
    else if (t_mod <= rectangle_params.period_up + hold_time_3 + rectangle_params.period_right + hold_time_1 + rectangle_params.period_down) {
        corres_t = (t_mod - rectangle_params.period_up - hold_time_3 - rectangle_params.period_right - hold_time_1) / rectangle_params.period_down;
        theta1 = horizontal_angle - 2 * horizontal_angle * corres_t;
        gamma1 = left_hori_servo - (asin((desierd_insertion_depth + turtle_height) / sqrt((l1 * cos(-theta1 * M_PI / 180)) * (l1 * cos(-theta1 * M_PI / 180)) + lower_point * lower_point)) - atan(lower_point / (l1 * cos(-theta1 * M_PI / 180)))) * 180 / M_PI;
        theta2 = -horizontal_angle + 2 * horizontal_angle * corres_t;
        gamma2 = right_hori_servo + (asin((desierd_insertion_depth + turtle_height) / sqrt((l1 * cos(-theta1 * M_PI / 180)) * (l1 * cos(-theta1 * M_PI / 180)) + lower_point * lower_point)) - atan(lower_point / (l1 * cos(-theta1 * M_PI / 180)))) * 180 / M_PI;
        turtle_.turtle_chassis.gait_state = 3;
        std::cout << "Phase3 Adduction" << gamma1 << std::endl;
    }
    // 新阶段 2：保持位置 3 秒
    else if (t_mod <= rectangle_params.period_up + hold_time_3 + rectangle_params.period_right + hold_time_1 + rectangle_params.period_down + hold_time_2) {
        theta1 = horizontal_angle - 2 * horizontal_angle;
        gamma1 = left_hori_servo - (asin((desierd_insertion_depth + turtle_height) / sqrt((l1 * cos(-theta1 * M_PI / 180)) * (l1 * cos(-theta1 * M_PI / 180)) + lower_point * lower_point)) - atan(lower_point / (l1 * cos(-theta1 * M_PI / 180)))) * 180 / M_PI;
        theta2 = -horizontal_angle + 2 * horizontal_angle;
        gamma2 = right_hori_servo + (asin((desierd_insertion_depth + turtle_height) / sqrt((l1 * cos(-theta1 * M_PI / 180)) * (l1 * cos(-theta1 * M_PI / 180)) + lower_point * lower_point)) - atan(lower_point / (l1 * cos(-theta1 * M_PI / 180)))) * 180 / M_PI;
        turtle_.turtle_chassis.gait_state = 6; // 新的 gait_state
    }
    // Phase 4
    else if (t_mod <= rectangle_params.period_up + hold_time_3 + rectangle_params.period_right + hold_time_1 + rectangle_params.period_down + hold_time_2 + rectangle_params.period_left) {
        corres_t = (t_mod - rectangle_params.period_up - hold_time_3 - rectangle_params.period_right - hold_time_1 - rectangle_params.period_down - hold_time_2) / rectangle_params.period_left;
        theta1 = -horizontal_angle;
        gamma1 = left_hori_servo - (initial_insertion_depth_rad * 180 / M_PI) + (initial_insertion_depth_rad * 180 / M_PI + extraction_angle) * corres_t;
        theta2 = horizontal_angle;
        gamma2 = right_hori_servo + (initial_insertion_depth_rad * 180 / M_PI) - (initial_insertion_depth_rad * 180 / M_PI + extraction_angle) * corres_t;
        turtle_.turtle_chassis.gait_state = 4;
    }
    // End delay phase
    else {
        gamma1 = left_hori_servo + extraction_angle;
        theta1 = -horizontal_angle;
        gamma2 = right_hori_servo - extraction_angle;
        theta2 = horizontal_angle;
        turtle_.turtle_chassis.gait_state = 8; // 新的 gait_state
    }
   
    // 设置伺服位置
    turtle_.turtle_control.left_adduction.set_input_position_degree.input_position = gamma1;
    turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position = theta1;
    turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = gamma2;
    turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = theta2;
    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = -gamma1 / 360;
    turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = -theta1 / 360;
    turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = -gamma2 / 360;
    turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = -theta2 / 360;
}


/**
 * @brief bouding gaits
 * @param time
 * @param bouding gaits
 * @return x y : the coordinates of the toe trajectories
 */

void boundingGAIT(turtle& turtle_, float t)
{
    fixed_insertion_depth_gait_lower_point_version_3_analytic_solution(turtle_,t);
}

