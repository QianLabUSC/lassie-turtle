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
void PhysicalToAbstract(float X1, float Y1, float X2, float Y2, float &theta1, float &gamma1, float &beta1, float &theta2, float &gamma2, float &beta2)
{
    float L = 0.14;

    theta1 = atan2(X1, L); // now -> rad,

    gamma1 = atan2(Y1, L); // now -> rad,
    beta2 = 90;            // now -> rad,
    beta1 = 180 - beta2;   // now -> rad,

    theta2 = atan2(X2, L); // now -> rad,
    gamma2 = atan2(Y2, L); // now -> rad,
    
    checkleft(gamma1, beta1, theta1);
    checkright(gamma2, beta2, theta2);
    

}
/**
 * @brief check whether gamma1, beta1 are out of range and transfer them to angle
 *
 *
 * @return : Returns <gamma1, beta1>
 */
void checkleft(float &gamma1, float &beta1, float &theta1)
{
    // float L = 0.14;
    beta1 = beta1;
    /*140 is hotizontal, 10 is vertical*/

    // if(gamma1 >= 10) {
    //     gamma1 = 140;
    // }
    // else {
    //     float gamma1_degree = 180 * gamma1 / M_PI;
    //     gamma1 = 140 + gamma1_degree * 140/90;
    // }
    // if(gamma1 > 140) gamma1 = 140;
    // if(gamma1 < 10) gamma1 = 10;

    // if(gamma1 >= 0) {
    //     gamma1 = 180;
    // }

    // float gamma1_degree = 180 * gamma1 / M_PI;
    theta1 = theta1 * M_PI/180;
    // gamma1 = 105 + gamma1_degree * (180 - 10) / 120;
    // gamma1 = 140;
    // cout << "gamma_degree_left" << gamma1 << endl;
    if (gamma1 > 105)
        gamma1 = 105;
    if (gamma1 < 10)
        gamma1 = 10;

    // beta1 = 180;
    float shift = 0.6 - 0.08;
    theta1 = theta1 + shift;
    if (theta1 < shift - 0.7)
        theta1 = shift - 0.7;
    if (theta1 > shift + 0.9)
        theta1 = shift + 0.9;
}

/**
 * @brief check whether gamma2, beta2 are out of range and transfer them to angle
 *
 *
 * @return : Returns < gamma2 , beta2>
 */
void checkright(float &gamma2, float &beta2, float &theta2)
{
    // float L = 0.14;
    beta2 = beta2;
    /*50 is hotizontal, 180 is vertical*/
    // if(gamma2 >= 0) {
    //     gamma2 = 50;
    // }
    // else {
    //     float gamma2_degree = 180 * gamma2 / M_PI;
    //     gamma2 = 50 - gamma2_degree * 130 / 90;
    // }
    // if(gamma2 > 180) gamma2 = 180;
    // if(gamma2 < 50) gamma2 = 50;

    // if(gamma2 >= 0) {
    //     gamma2 = 10;
    // }
    // float gamma2_degree = 180 * gamma2 / M_PI;

    // gamma2 = 12 - gamma2_degree * (180 - 10) / 120;
    theta2 = theta2 * M_PI / 180;
    if (gamma2 > 180)
        gamma2 = 180;
    if (gamma2 < 10)
        gamma2 = 10;

    // beta2 = 0;
    float shift = 0.6 + 0.08;
    theta2 = theta2 + shift;
    if (theta2 < shift - 0.9)
        theta2 = shift - 0.9;
    if (theta2 > shift + 0.7)
        theta2 = shift + 0.7;
}

/**
 * @brief Finds the physical (X, Y) position of the toe for a given L and Theta
 *
 * @param L : Leg length
 * @param Theta : Abstract leg angle
 * @return : Pair (X, Y) representing the position of the toe in relation to
 *              the Origin (the hip joint)
 */
void AbstractToPhysical(float L, float Theta, float gamma, float &x, float &y)
{
    x = L * sinf(Theta);
    y = L * sinf(gamma);
}

/**
 * @brief oval trajectories
 * @param time
 * @param OvalParams
 * @return x y : the coordinates of the toe trajectories
 */







// Fixed insertion depth verson3 analytic solution with matlab homogeneous matrix verified

void fixed_insertion_depth_gait_lower_point_version_3_analytic_solution(turtle& turtle_, float t)git {
     //get data
     float horizontal_angle = turtle_.traj_data.lateral_angle_range*180/M_PI;
    Rectangle_Params rectangle_params;
    rectangle_params.period_down = turtle_.traj_data.lateral_angle_range * 0.14 * 2/turtle_.traj_data.drag_speed;
    rectangle_params.period_up = 0.35; //customize back phase time
    rectangle_params.period_left = turtle_.traj_data.servo_speed;
    rectangle_params.period_right = turtle_.traj_data.servo_speed * 8;
    rectangle_params.vertical_range = turtle_.traj_data.insertion_depth; // depend on insertion depth
    rectangle_params.horizontal_range = turtle_.traj_data.lateral_angle_range*180/M_PI;
    rectangle_params.period_waiting_time = 0;
    float t_mod = fmod(t, (rectangle_params.period_down + rectangle_params.period_up + rectangle_params.period_left + rectangle_params.period_right + rectangle_params.period_waiting_time));
 turtle_.turtle_chassis.step_count=(t-t_mod)/ (rectangle_params.period_down + rectangle_params.period_up + rectangle_params.period_left + rectangle_params.period_right + rectangle_params.period_waiting_time);
    
    double desierd_insertion_depth=turtle_.traj_data.insertion_depth;

//adaptation insertion depth
    // if  (turtle_.turtle_chassis.step_count>3)
    // {
    //     desierd_insertion_depth=0.05;


    // }


    
    float corres_t = 0;
    float left_hori_servo = 0;// original 100 mannually tuned to 94
    float right_hori_servo = 0;
    float extraction_angle = turtle_.traj_data.extraction_angle;


    //Fixed insertion depth parameters and gamma solver paramters
    
    double l1=0.145-0.025; // flipper length-new shorter flipper
    double turtle_height=0.055; //height from flipper to fround
    double lower_point=0.05;

    // desired insertion depth input

 
    if (desierd_insertion_depth>0.07)
   { desierd_insertion_depth=0.07;
   }
    

     cout << "desired insertion depth(m)"<<  desierd_insertion_depth<< endl;// m
    // fixed insertion depth initial calculation
    double initial_insertion_depth_rad=asin((desierd_insertion_depth+turtle_height)/sqrt((l1*cos(horizontal_angle*M_PI/180))*(l1*cos(horizontal_angle*M_PI/180))+lower_point*lower_point))-atan(lower_point/(l1*cos(horizontal_angle*M_PI/180)));
    double initial_insertion_depth_deg=initial_insertion_depth_rad*180/M_PI;
    cout << "TMOD" << t_mod << endl;
   // cout<<"Initial angle"<<initial_insertion_depth_deg<<endl;
    // insertion_depth=insertion_depth+10
    double gamma1 = 0;
    double theta1 = 0;
    double gamma2 = 0;
    double theta2 = 0;
    //FIRST phase gamma=0 theta=-45
    if (t_mod <= rectangle_params.period_up)
    {
        corres_t = t_mod / rectangle_params.period_up;
       // cout << "UP corres_t" << corres_t << endl;
        gamma1 = left_hori_servo + extraction_angle;
        // Y1 = - r_v + r_v * corres_t;
        theta1 = - horizontal_angle + 2*horizontal_angle*corres_t;
        // -r_v + (r_v + upper_height) * corres_t;
        gamma2 = right_hori_servo - extraction_angle;
        // Y2 = - r_v + r_v * corres_t;
        // Y2 = -r_v + (r_v + upper_height) * corres_t;
        theta2 = horizontal_angle - 2*horizontal_angle*corres_t;
        // set turtle gait state flag:
        turtle_.turtle_chassis.gait_state = 1;
    }
    //Phase2  theta=-45 gamma go to desired value
    else if (t_mod <= rectangle_params.period_up + rectangle_params.period_right)
    {
        corres_t = (t_mod - rectangle_params.period_up) / rectangle_params.period_right;
        //cout << "RIGHT corres_t" << corres_t << endl;
        // X1 = r_h - r_h * 2 * corres_t;
        theta1 = horizontal_angle;
        // Y1 = 0;
        // Y1 = upper_height;
        gamma1 = left_hori_servo + extraction_angle - 
                    (initial_insertion_depth_rad*180/M_PI + extraction_angle) * corres_t;
        // X2 = -r_h + r_h * 2 * corres_t;
        theta2 = -horizontal_angle;
        
        // Y2 = 0;
        // Y2 = upper_height;
        gamma2 = right_hori_servo - extraction_angle + 
                    (initial_insertion_depth_rad*180/M_PI + extraction_angle) * corres_t;
        turtle_.turtle_chassis.gait_state = 2;
    }

    //
    else if (t_mod <= rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down)
    {
        corres_t = (t_mod - rectangle_params.period_up - rectangle_params.period_right) / rectangle_params.period_down;
      //  cout << "DOWN corres_t" << corres_t << endl;
        
        theta1 = horizontal_angle - 2*horizontal_angle*corres_t;
        gamma1 = left_hori_servo -(asin((desierd_insertion_depth+turtle_height)/sqrt((l1*cos(-theta1*M_PI/180))*(l1*cos(-theta1*M_PI/180)+lower_point*lower_point))-atan(lower_point/(l1*cos(-theta1*M_PI/180)))))*180/M_PI;
      
        //cout << "solved_gamma" << gammasolver(l1,lower_point,desierd_insertion_depth,turtle_height,-theta1*M_PI/180,initial_gamma, tol, maxIteration )*180/M_PI << endl;
        //cout<<"theta1"<<theta1<<endl;
        //cout<<"gamma_analytic_solution"<<(asin((desierd_insertion_depth+turtle_height)/sqrt((l1*cos(-theta1*M_PI/180))*(l1*cos(-theta1*M_PI/180)+lower_point*lower_point))-atan(lower_point/(l1*cos(-theta1*M_PI/180)))))*180/M_PI<<endl;
        theta2 = -horizontal_angle + 2*horizontal_angle*corres_t;
        gamma2 = right_hori_servo + (asin((desierd_insertion_depth+turtle_height)/sqrt((l1*cos(-theta1*M_PI/180))*(l1*cos(-theta1*M_PI/180)+lower_point*lower_point))-atan(lower_point/(l1*cos(-theta1*M_PI/180)))))*180/M_PI;
        turtle_.turtle_chassis.gait_state = 3;
         // cout << "swipe"  << endl;


       // csv output
    //    std::ofstream outputFile("output.csv",std::ios::app);
    //    outputFile<< (asin((desierd_insertion_depth+turtle_height)/sqrt((l1*cos(-theta1*M_PI/180))*(l1*cos(-theta1*M_PI/180)+lower_point*lower_point))-atan(lower_point/(l1*cos(-theta1*M_PI/180)))))*180/M_PI<<"\n";
    //    outputFile.close();
    //    std::cout<<"CSV done";
    }
    else{
        corres_t = (t_mod - rectangle_params.period_up - rectangle_params.period_right - rectangle_params.period_down) / rectangle_params.period_left;
        theta1 = -horizontal_angle;
        //cout << "LEFT corres_t" << corres_t << endl;
        gamma1 = left_hori_servo -  (initial_insertion_depth_rad*180/M_PI) +(initial_insertion_depth_rad*180/M_PI + extraction_angle)* corres_t;
        theta2 = horizontal_angle;
        gamma2 = right_hori_servo + (initial_insertion_depth_rad*180/M_PI) - (initial_insertion_depth_rad*180/M_PI + extraction_angle) * corres_t;
        turtle_.turtle_chassis.gait_state = 4;
       //  cout << "extract"  << endl;
    }
    
    //gamma theta in deg

  //adduction angle: left gamma: gamma2,right gamma:gamma2
    

      // sweeping angle:    right theta: theta2 left theta   :theta1
    turtle_.turtle_control.left_adduction.set_input_position_degree.input_position = gamma1;
    turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position = theta1;
    turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = gamma2;
    turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = theta2;
    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = -gamma1/360;
    turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = -theta1/360;
    turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = -gamma2/360;
    turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = -theta2/360;
    // used for test
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

