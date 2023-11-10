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
    float L = 0.14;

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
    float L = 0.14;

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

void oval_generator(float t, float &X1, float &Y1, float &X2, float &Y2)
{
    OvalParams oval_params = {0.3f, .3f, 0.17f, 0.17f};
    float t_mod = fmod(t, (oval_params.period_down + oval_params.period_up));
    float corres_angle = 0;
    if (t_mod < oval_params.period_down)
    {
        corres_angle = M_PI + M_PI * t_mod / oval_params.period_down;
    }
    else
    {
        // corres_angle = M_PI;
        corres_angle = M_PI * ((t_mod - oval_params.period_down) / oval_params.period_up);
    }

    if (corres_angle == M_PI / 2)
    {
        X1 = 0;
        Y1 = 0;
        X2 = 0;
        Y2 = 0;
    }
    if (corres_angle == 3 * M_PI / 2)
    {
        X1 = 0;
        Y1 = -oval_params.vertical_range;
        X2 = 0;
        Y2 = -oval_params.vertical_range;
    }
    // else if(corres_angle == 3*M_PI/2){
    //     X = 0;
    //     Y = - oval_params.vertical_range;
    // }
    // else if(corres_angle > M_PI/2 && corres_angle < 3*M_PI/2){
    //     float r_h = oval_params.horizontal_range;
    //     float r_v = oval_params.vertical_range;
    //     X = - r_h * r_v / sqrt(r_v*r_v + r_h*r_h*tanf(corres_angle)*tanf(corres_angle));
    //     Y = - r_h * r_v * tanf(corres_angle)/sqrt(r_v*r_v + r_h*r_h*tanf(corres_angle)*tanf(corres_angle));
    // }
    // else{
    //     float r_h = oval_params.horizontal_range;
    //     float r_v = oval_params.vertical_range;
    //     X = r_h * r_v / sqrt(r_v*r_v + r_h*r_h*tanf(corres_angle)*tanf(corres_angle));
    //     Y = r_h * r_v * tanf(corres_angle)/sqrt(r_v*r_v + r_h*r_h*tanf(corres_angle)*tanf(corres_angle));
    // }
    else if (corres_angle > 0 && corres_angle < M_PI / 2)
    {
        float r_h = oval_params.horizontal_range;
        float r_v = oval_params.vertical_range;
        X1 = r_h * r_v / sqrt(r_v * r_v + r_h * r_h * tanf(corres_angle) * tanf(corres_angle));
        Y1 = 0;
        X2 = -r_h * r_v / sqrt(r_v * r_v + r_h * r_h * tanf(corres_angle) * tanf(corres_angle));
        Y2 = 0;
    }

    else if (corres_angle > M_PI / 2 && corres_angle < M_PI)
    {
        float r_h = oval_params.horizontal_range;
        float r_v = oval_params.vertical_range;
        X1 = -r_h * r_v / sqrt(r_v * r_v + r_h * r_h * tanf(corres_angle) * tanf(corres_angle));
        Y1 = 0;
        X2 = r_h * r_v / sqrt(r_v * r_v + r_h * r_h * tanf(corres_angle) * tanf(corres_angle));
        Y2 = 0;
    }

    else if (corres_angle > M_PI && corres_angle < 3 * M_PI / 2)
    {
        float r_h = oval_params.horizontal_range;
        float r_v = oval_params.vertical_range;
        X1 = -r_h * r_v / sqrt(r_v * r_v + r_h * r_h * tanf(corres_angle) * tanf(corres_angle));
        Y1 = -r_h * r_v * tanf(corres_angle) / sqrt(r_v * r_v + r_h * r_h * tanf(corres_angle) * tanf(corres_angle));
        ;
        X2 = r_h * r_v / sqrt(r_v * r_v + r_h * r_h * tanf(corres_angle) * tanf(corres_angle));
        Y2 = -r_h * r_v * tanf(corres_angle) / sqrt(r_v * r_v + r_h * r_h * tanf(corres_angle) * tanf(corres_angle));
        ;
    }

    else if (corres_angle > 3 * M_PI / 2 && corres_angle < 2 * M_PI)
    {
        float r_h = oval_params.horizontal_range;
        float r_v = oval_params.vertical_range;
        X1 = r_h * r_v / sqrt(r_v * r_v + r_h * r_h * tanf(corres_angle) * tanf(corres_angle));
        Y1 = r_h * r_v * tanf(corres_angle) / sqrt(r_v * r_v + r_h * r_h * tanf(corres_angle) * tanf(corres_angle));
        ;
        X2 = -r_h * r_v / sqrt(r_v * r_v + r_h * r_h * tanf(corres_angle) * tanf(corres_angle));
        Y2 = r_h * r_v * tanf(corres_angle) / sqrt(r_v * r_v + r_h * r_h * tanf(corres_angle) * tanf(corres_angle));
        ;
    }
    X1 = -X1;
    X2 = -X2;
}

void rectangle_generator(turtle turtle_, float t, float &X1, float &Y1, float &X2, float &Y2)
{
    // float temp = 2.5f;
    // float idle = 8.0f;
    // float servo_time = 0.25f;
    // float period = 1.25;

    // please change this value to control the turtle
    // turtle_.traj_data.drag_speed;
    // turtle_.traj_data.extraction_height;
    // turtle_.traj_data.insertion_angle;
    // turtle_.traj_data.lateral_angle_range;
    // turtle_.traj_data.servo_time;
    // turtle_.traj_data.wiggle_amptitude;
    // turtle_.traj_data.wiggle_frequency;
    // turtle_.traj_data.wiggle_time;

    // please use these two variables to start the robot and stop the robot
    // turtle_.turtle_gui.drag_traj; //5
    // turtle_.turtle_gui.start_flag; //1 or 0
    float horizontal_range = 0.14 * tanf(turtle_.traj_data.lateral_angle_range);
    float drag = horizontal_range/turtle_.traj_data.drag_speed;

    float servo_time = turtle_.traj_data.servo_time;
    float pad_back_time = drag;

    float upper_height = 0.05;
    float insertion_depth = 0.14*sinf(turtle_.traj_data.insertion_angle);
    
    float waiting_time = turtle_.traj_data.wiggle_time;
    float wiggle_length = turtle_.traj_data.wiggle_amptitude;
    float wiggle_frequency = turtle_.traj_data.wiggle_frequency;

    // cout << "horizontal_range" << horizontal_range << "drag" <<drag<<"servo_time"<<servo_time<<"insertion_depth"<<insertion_depth<<"waiting_time"<<waiting_time<<"wiggle_length"<<wiggle_length<<"wiggle_frequency"<<wiggle_frequency<<endl;


    Rectangle_Params rectangle_params = {drag, pad_back_time, servo_time, servo_time, insertion_depth, horizontal_range, waiting_time, wiggle_length, wiggle_frequency};
    float t_mod = fmod(t, (rectangle_params.period_down + rectangle_params.period_up + rectangle_params.period_left + rectangle_params.period_right + rectangle_params.period_waiting_time));
    // t_mod = rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right + 0.5 * rectangle_params.period_down;
    // t_mod = rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down;
    // t_mod = rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right;
    // t_mod = rectangle_params.period_left + rectangle_params.period_up;
    float corres_t = 0;
    float r_h = rectangle_params.horizontal_range;
    float r_v = rectangle_params.vertical_range;
    if (t_mod <= rectangle_params.period_left)
    {
        corres_t = t_mod / rectangle_params.period_left;

        X1 = r_h;
        // Y1 = - r_v + r_v * corres_t;
        Y1 = -r_v + (r_v + upper_height) * corres_t;
        X2 = -r_h;
        // Y2 = - r_v + r_v * corres_t;
        Y2 = -r_v + (r_v + upper_height) * corres_t;
    }
    else if (t_mod <= rectangle_params.period_left + rectangle_params.period_up)
    {
        corres_t = (t_mod - rectangle_params.period_left) / rectangle_params.period_up;

        X1 = r_h - r_h * 2 * corres_t;
        // Y1 = 0;
        Y1 = upper_height;
        X2 = -r_h + r_h * 2 * corres_t;
        // Y2 = 0;
        Y2 = upper_height;
    }
    else if (t_mod <= rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right)
    {
        corres_t = (t_mod - rectangle_params.period_left - rectangle_params.period_up) / rectangle_params.period_right;
        X1 = -rectangle_params.horizontal_range;
        // Y1 = - rectangle_params.vertical_range * corres_t;
        Y1 = upper_height - (r_v + upper_height) * corres_t;
        X2 = rectangle_params.horizontal_range;
        // Y2 = - rectangle_params.vertical_range * corres_t;
        Y2 = upper_height - (r_v + upper_height) * corres_t;
    }

    // ###no wiggle
    // else              
    // {

    //     if (t_mod > rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down)
    //     {
    //         X1 = r_h;
    //         Y1 = -r_v;
    //         X2 = -r_h;
    //         Y2 = -r_v;
    //     }
    //     else
    //     {
    //         corres_t = (t_mod - rectangle_params.period_left - rectangle_params.period_up - rectangle_params.period_right) / rectangle_params.period_down;

    //         X1 = -r_h + r_h * 2 * corres_t;
    //         Y1 = -r_v;
    //         X2 = r_h - r_h * 2 * corres_t;
    //         Y2 = -r_v;
    //     }
    //     // corres_t = (t_mod - rectangle_params.period_left - rectangle_params.period_up - rectangle_params.period_right) / rectangle_params.period_down;
    //     // float r_h = rectangle_params.horizontal_range;
    //     // float r_v = rectangle_params.vertical_range;
    //     // X1 = r_h - r_h * 2 * corres_t;
    //     // Y1 = - r_v;
    //     // X2 = - r_h + r_h * 2 * corres_t;
    //     // Y2 = - r_v;
    // }

    // wiggle
    else if (t_mod <= rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down)
    {   corres_t = (t_mod - rectangle_params.period_left - rectangle_params.period_up - rectangle_params.period_right) / rectangle_params.period_down;
        X1 = -r_h + r_h * 2 * corres_t;
        Y1 = -r_v;
        X2 = r_h - r_h * 2 * corres_t;
        Y2 = -r_v;

    }
    else
    {
        corres_t = fmod(t_mod - rectangle_params.period_left - rectangle_params.period_up - rectangle_params.period_right - rectangle_params.period_down, rectangle_params.period_waiting_time / rectangle_params.wiggle_frequency);   //  flow of half period of wiggle
        if (corres_t <= rectangle_params.period_waiting_time / (2 * rectangle_params.wiggle_frequency))
        {
            X1 = r_h - wiggle_length * corres_t;
            Y1 = -r_v;
            X2 = -r_h + wiggle_length * corres_t;
            Y2 = -r_v;
        }
        else
        {
            X1 = r_h - wiggle_length + wiggle_length * corres_t;
            Y1 = -r_v;
            X2 = -r_h + wiggle_length - wiggle_length * corres_t;
            Y2 = -r_v;
        }

    }
}

void servo_angle_gait(turtle& turtle_, float t, float& theta2, float& gamma1,float& theta1, float& gamma2){
    float horizontal_angle = turtle_.traj_data.lateral_angle_range*180/M_PI;
    float drag = turtle_.traj_data.lateral_angle_range * 0.14 * 2/turtle_.traj_data.drag_speed;

    float servo_time = turtle_.traj_data.servo_time;
    // float pad_back_time = drag;
   float pad_back_time = 0.35;  //speed up 
    float insertion_angle = turtle_.traj_data.insertion_angle*180/M_PI;
    cout << "angle outp put"<< insertion_angle << " horizontal " << horizontal_angle << endl;
    float waiting_time = 0;
    float wiggle_length = turtle_.traj_data.wiggle_amptitude;
    float wiggle_frequency = turtle_.traj_data.wiggle_frequency;
    Rectangle_Params rectangle_params = {drag, pad_back_time, servo_time, servo_time, insertion_angle, horizontal_angle, waiting_time, wiggle_length, wiggle_frequency};
    
    float t_mod = fmod(t, (rectangle_params.period_down + rectangle_params.period_up + rectangle_params.period_left + rectangle_params.period_right + rectangle_params.period_waiting_time));
    // t_mod = rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right + 0.5 * rectangle_params.period_down;
    // t_mod = rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down;
    // t_mod = rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right;
    // t_mod = rectangle_params.period_left + rectangle_params.period_up;
    float corres_t = 0;
    float left_hori_servo = 100;
    float right_hori_servo = 23;
  
    cout << "TMOD" << t_mod << endl;
    // insertion_angle=insertion_angle+10
    if (t_mod <= rectangle_params.period_up)
    {
        corres_t = t_mod / rectangle_params.period_up;
        cout << "UP corres_t" << corres_t << endl;
        gamma1 = left_hori_servo;
        // Y1 = - r_v + r_v * corres_t;
        theta1 = - horizontal_angle + 2*horizontal_angle*corres_t;
        // -r_v + (r_v + upper_height) * corres_t;
        gamma2 = right_hori_servo;
        // Y2 = - r_v + r_v * corres_t;
        // Y2 = -r_v + (r_v + upper_height) * corres_t;
        theta2 = horizontal_angle - 2*horizontal_angle*corres_t;
    }
    else if (t_mod <= rectangle_params.period_up + rectangle_params.period_right)
    {
        corres_t = (t_mod - rectangle_params.period_up) / rectangle_params.period_right;
        cout << "RIGHT corres_t" << corres_t << endl;
        // X1 = r_h - r_h * 2 * corres_t;
        theta1 = horizontal_angle;
        // Y1 = 0;
        // Y1 = upper_height;
        gamma1 = left_hori_servo - insertion_angle * corres_t;
        // X2 = -r_h + r_h * 2 * corres_t;
        theta2 = -horizontal_angle;
        
        // Y2 = 0;
        // Y2 = upper_height;
        gamma2 = right_hori_servo + insertion_angle * corres_t;
    }
    else if (t_mod <= rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down)
    {
        corres_t = (t_mod - rectangle_params.period_up - rectangle_params.period_right) / rectangle_params.period_down;
        // X1 = -rectangle_params.horizontal_range;
        cout << "DOWN corres_t" << corres_t << endl;
        theta1 = horizontal_angle - 2*horizontal_angle*corres_t;
        // Y1 = - rectangle_params.vertical_range * corres_t;
        // Y1 = upper_height - (r_v + upper_height) * corres_t;
        gamma1 = left_hori_servo - insertion_angle;
        // X2 = rectangle_params.horizontal_range;
        theta2 = -horizontal_angle + 2*horizontal_angle*corres_t;
        // Y2 = - rectangle_params.vertical_range * corres_t;
        // Y2 = upper_height - (r_v + upper_height) * corres_t;
        gamma2 = right_hori_servo + insertion_angle;
    }
    else{
        corres_t = (t_mod - rectangle_params.period_up - rectangle_params.period_right - rectangle_params.period_down) / rectangle_params.period_left;
        theta1 = -horizontal_angle;
        cout << "LEFT corres_t" << corres_t << endl;
        gamma1 = left_hori_servo - insertion_angle  + insertion_angle * corres_t;
        theta2 = horizontal_angle;
        gamma2 = right_hori_servo + insertion_angle - insertion_angle * corres_t;
    }
    // ###no wiggle
    // else              
    // {

    //     if (t_mod > rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down)
    //     {
    //         X1 = r_h;
    //         Y1 = -r_v;
    //         X2 = -r_h;
    //         Y2 = -r_v;
    //     }
    //     else
    //     {
    //         corres_t = (t_mod - rectangle_params.period_left - rectangle_params.period_up - rectangle_params.period_right) / rectangle_params.period_down;

    //         X1 = -r_h + r_h * 2 * corres_t;
    //         Y1 = -r_v;
    //         X2 = r_h - r_h * 2 * corres_t;
    //         Y2 = -r_v;
    //     }
    //     // corres_t = (t_mod - rectangle_params.period_left - rectangle_params.period_up - rectangle_params.period_right) / rectangle_params.period_down;
    //     // float r_h = rectangle_params.horizontal_range;
    //     // float r_v = rectangle_params.vertical_range;
    //     // X1 = r_h - r_h * 2 * corres_t;
    //     // Y1 = - r_v;
    //     // X2 = - r_h + r_h * 2 * corres_t;
    //     // Y2 = - r_v;
    // }






    // wiggle
    // else if (t_mod <= rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down)
    // {   corres_t = (t_mod - rectangle_params.period_left - rectangle_params.period_up - rectangle_params.period_right) / rectangle_params.period_down;
    //     X1 = -r_h + r_h * 2 * corres_t;
    //     Y1 = -r_v;
    //     X2 = r_h - r_h * 2 * corres_t;
    //     Y2 = -r_v;

    // }
    // else
    // {
    //     corres_t = fmod(t_mod - rectangle_params.period_left - rectangle_params.period_up - rectangle_params.period_right - rectangle_params.period_down, rectangle_params.period_waiting_time / rectangle_params.wiggle_frequency);   //  flow of half period of wiggle
    //     if (corres_t <= rectangle_params.period_waiting_time / (2 * rectangle_params.wiggle_frequency))
    //     {
    //         X1 = r_h - wiggle_length * corres_t;
    //         Y1 = -r_v;
    //         X2 = -r_h + wiggle_length * corres_t;
    //         Y2 = -r_v;
    //     }
    //     else
    //     {
    //         X1 = r_h - wiggle_length + wiggle_length * corres_t;
    //         Y1 = -r_v;
    //         X2 = -r_h + wiggle_length - wiggle_length * corres_t;
    //         Y2 = -r_v;
    //     }

    // }

}



void fixed_insertion_depth_gait(turtle& turtle_, float t, float& theta2, float& gamma1,float& theta1, float& gamma2){
    float horizontal_angle = turtle_.traj_data.lateral_angle_range*180/M_PI;
    float drag = turtle_.traj_data.lateral_angle_range * 0.14 * 2/turtle_.traj_data.drag_speed;

    float servo_time = turtle_.traj_data.servo_time;
    // float pad_back_time = drag;
   float pad_back_time = 0.35;  //speed up 
    float insertion_angle = turtle_.traj_data.insertion_angle*180/M_PI;
    cout << "angle outp put"<< insertion_angle << " horizontal " << horizontal_angle << endl;
    float waiting_time = 0;
    float wiggle_length = turtle_.traj_data.wiggle_amptitude;
    float wiggle_frequency = turtle_.traj_data.wiggle_frequency;
    Rectangle_Params rectangle_params = {drag, pad_back_time, servo_time, servo_time, insertion_angle, horizontal_angle, waiting_time, wiggle_length, wiggle_frequency};
    
    float t_mod = fmod(t, (rectangle_params.period_down + rectangle_params.period_up + rectangle_params.period_left + rectangle_params.period_right + rectangle_params.period_waiting_time));
    // t_mod = rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right + 0.5 * rectangle_params.period_down;
    // t_mod = rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down;
    // t_mod = rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right;
    // t_mod = rectangle_params.period_left + rectangle_params.period_up;
    float corres_t = 0;
    float left_hori_servo = 110;
    float right_hori_servo = 17;
      float initial_insertion_angle_rad=asin((0.06+0.015)/(0.14*cos(45*M_PI/180.0)));
    float initial_insertion_angle_deg=initial_insertion_angle_rad*180.0/M_PI;
    cout << "TMOD" << t_mod << endl;
    // insertion_angle=insertion_angle+10
    if (t_mod <= rectangle_params.period_up)
    {
        corres_t = t_mod / rectangle_params.period_up;
        cout << "UP corres_t" << corres_t << endl;
        gamma1 = left_hori_servo;
        // Y1 = - r_v + r_v * corres_t;
        theta1 = - horizontal_angle + 2*horizontal_angle*corres_t;
        // -r_v + (r_v + upper_height) * corres_t;
        gamma2 = right_hori_servo;
        // Y2 = - r_v + r_v * corres_t;
        // Y2 = -r_v + (r_v + upper_height) * corres_t;
        theta2 = horizontal_angle - 2*horizontal_angle*corres_t;
    }
    else if (t_mod <= rectangle_params.period_up + rectangle_params.period_right)
    {
        corres_t = (t_mod - rectangle_params.period_up) / rectangle_params.period_right;
        cout << "RIGHT corres_t" << corres_t << endl;
        // X1 = r_h - r_h * 2 * corres_t;
        theta1 = horizontal_angle;
        // Y1 = 0;
        // Y1 = upper_height;
        gamma1 = left_hori_servo - initial_insertion_angle_deg * corres_t;
        // X2 = -r_h + r_h * 2 * corres_t;
        theta2 = -horizontal_angle;
        
        // Y2 = 0;
        // Y2 = upper_height;
        gamma2 = right_hori_servo + initial_insertion_angle_deg* corres_t;
    }
    else if (t_mod <= rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down)
    {
        corres_t = (t_mod - rectangle_params.period_up - rectangle_params.period_right) / rectangle_params.period_down;
        // X1 = -rectangle_params.horizontal_range;
        cout << "DOWN corres_t" << corres_t << endl;
        theta1 = horizontal_angle - 2*horizontal_angle*corres_t;
        // Y1 = - rectangle_params.vertical_range * corres_t;
        // Y1 = upper_height - (r_v + upper_height) * corres_t;
        gamma1 = left_hori_servo - asin((0.06+0.015)/(0.14*cos(-1*theta1*M_PI/180.0)))*180/M_PI;
        // X2 = rectangle_params.horizontal_range;
        theta2 = -horizontal_angle + 2*horizontal_angle*corres_t;
        // Y2 = - rectangle_params.vertical_range * corres_t;
        // Y2 = upper_height - (r_v + upper_height) * corres_t;
        gamma2 = right_hori_servo + asin((0.06+0.015)/(0.14*cos(-1*theta1*M_PI/180.0)))*180/M_PI;
    }
    else{
        corres_t = (t_mod - rectangle_params.period_up - rectangle_params.period_right - rectangle_params.period_down) / rectangle_params.period_left;
        theta1 = -horizontal_angle;
        cout << "LEFT corres_t" << corres_t << endl;
        gamma1 = left_hori_servo - initial_insertion_angle_deg +initial_insertion_angle_deg* corres_t;
        theta2 = horizontal_angle;
        gamma2 = right_hori_servo + initial_insertion_angle_deg - initial_insertion_angle_deg * corres_t;
    }
    

}



void fixed_insertion_depth_gait_lower_point(turtle& turtle_, float t, float& theta2, float& gamma1,float& theta1, float& gamma2){
    float horizontal_angle = turtle_.traj_data.lateral_angle_range*180/M_PI;
    float drag = turtle_.traj_data.lateral_angle_range * 0.14 * 2/turtle_.traj_data.drag_speed;

    float servo_time = turtle_.traj_data.servo_time;
    // float pad_back_time = drag;
   float pad_back_time = 0.65;  //speed up 0.35
    float insertion_angle = turtle_.traj_data.insertion_angle*180/M_PI;
    cout << "angle outp put"<< insertion_angle << " horizontal " << horizontal_angle << endl;
    float waiting_time = 0;
    float wiggle_length = turtle_.traj_data.wiggle_amptitude;
    float wiggle_frequency = turtle_.traj_data.wiggle_frequency;
    Rectangle_Params rectangle_params = {drag, pad_back_time, servo_time, servo_time, insertion_angle, horizontal_angle, waiting_time, wiggle_length, wiggle_frequency};
    
    float t_mod = fmod(t, (rectangle_params.period_down + rectangle_params.period_up + rectangle_params.period_left + rectangle_params.period_right + rectangle_params.period_waiting_time));
    // t_mod = rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right + 0.5 * rectangle_params.period_down;
    // t_mod = rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down;
    // t_mod = rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right;
    // t_mod = rectangle_params.period_left + rectangle_params.period_up;
    float corres_t = 0;
    float left_hori_servo = 94;// original 100 mannually tuned to 97
    float right_hori_servo = 23;
    //Fixed insertion depth parameters
    float turtle_height=0.06;
    float lower_point=0.05;
    float desierd_insertion_depth=0.075;
    // fixed insertion depth calculation
    float initial_insertion_angle_rad=asin((turtle_height+desierd_insertion_depth)/(0.125*cos(45*M_PI/180.0)+lower_point));
    float initial_insertion_angle_deg=initial_insertion_angle_rad*180.0/M_PI;
    cout << "TMOD" << t_mod << endl;
    // insertion_angle=insertion_angle+10
    if (t_mod <= rectangle_params.period_up)
    {
        corres_t = t_mod / rectangle_params.period_up;
        cout << "UP corres_t" << corres_t << endl;
        gamma1 = left_hori_servo;
        // Y1 = - r_v + r_v * corres_t;
        theta1 = - horizontal_angle + 2*horizontal_angle*corres_t;
        // -r_v + (r_v + upper_height) * corres_t;
        gamma2 = right_hori_servo;
        // Y2 = - r_v + r_v * corres_t;
        // Y2 = -r_v + (r_v + upper_height) * corres_t;
        theta2 = horizontal_angle - 2*horizontal_angle*corres_t;
    }
    else if (t_mod <= rectangle_params.period_up + rectangle_params.period_right)
    {
        corres_t = (t_mod - rectangle_params.period_up) / rectangle_params.period_right;
        cout << "RIGHT corres_t" << corres_t << endl;
        // X1 = r_h - r_h * 2 * corres_t;
        theta1 = horizontal_angle;
        // Y1 = 0;
        // Y1 = upper_height;
        gamma1 = left_hori_servo - initial_insertion_angle_deg * corres_t;
        // X2 = -r_h + r_h * 2 * corres_t;
        theta2 = -horizontal_angle;
        
        // Y2 = 0;
        // Y2 = upper_height;
        gamma2 = right_hori_servo + initial_insertion_angle_deg* corres_t;
    }
    else if (t_mod <= rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down)
    {
        corres_t = (t_mod - rectangle_params.period_up - rectangle_params.period_right) / rectangle_params.period_down;
        // X1 = -rectangle_params.horizontal_range;
        cout << "DOWN corres_t" << corres_t << endl;
        theta1 = horizontal_angle - 2*horizontal_angle*corres_t;
        // Y1 = - rectangle_params.vertical_range * corres_t;
        // Y1 = upper_height - (r_v + upper_height) * corres_t;
        gamma1 = left_hori_servo - asin((turtle_height+desierd_insertion_depth)/(0.125*cos(-1*(theta1*M_PI/180.0))+lower_point))*180/M_PI;
        // X2 = rectangle_params.horizontal_range;
        theta2 = -horizontal_angle + 2*horizontal_angle*corres_t;
        // Y2 = - rectangle_params.vertical_range * corres_t;
        // Y2 = upper_height - (r_v + upper_height) * corres_t;
        gamma2 = right_hori_servo + asin((turtle_height+desierd_insertion_depth)/(0.125*cos(-1*(theta1*M_PI/180.0))+lower_point))*180/M_PI;
        cout << "swipe"  << endl;
    }
    else{
        corres_t = (t_mod - rectangle_params.period_up - rectangle_params.period_right - rectangle_params.period_down) / rectangle_params.period_left;
        theta1 = -horizontal_angle;
        cout << "LEFT corres_t" << corres_t << endl;
        gamma1 = left_hori_servo - initial_insertion_angle_deg +initial_insertion_angle_deg* corres_t;
        theta2 = horizontal_angle;
        gamma2 = right_hori_servo + initial_insertion_angle_deg - initial_insertion_angle_deg * corres_t;
         cout << "extract"  << endl;
    }
    

}




float gammasolver(float flipper_l1,float flipper_l2,float insertiondepth_h, float turtleheight_a,float swiping_theta0,float adduction_gamma0, float tol, int maxIter ){
float theta_solved=swiping_theta0;
float gamma_solved=adduction_gamma0;
int iter=0;

while (iter<maxIter){
float f_function=sin(gamma_solved)*flipper_l1*cos(theta_solved)+cos(gamma_solved)*flipper_l2-insertiondepth_h-turtleheight_a;
float f_dot_gamma=cos(gamma_solved)*flipper_l1*cos(theta_solved)-sin(gamma_solved)*flipper_l2;

float gamma_solved_new=gamma_solved-f_function/f_dot_gamma;
if (fabs(gamma_solved_new-gamma_solved)<tol){
break;
}
gamma_solved=gamma_solved_new;
iter++;

}


return gamma_solved;
}


void fixed_insertion_depth_gait_lower_point_version_2_approximate(turtle& turtle_, float t, float& theta2, float& gamma1,float& theta1, float& gamma2){
  float horizontal_angle = turtle_.traj_data.lateral_angle_range*180/M_PI;
    float drag = turtle_.traj_data.lateral_angle_range * 0.14 * 2/turtle_.traj_data.drag_speed;

    float servo_time = turtle_.traj_data.servo_time;
    // float pad_back_time = drag;
   float pad_back_time = 0.65;  //speed up 0.35
    float insertion_angle = turtle_.traj_data.insertion_angle*180/M_PI;
    cout << "angle outp put"<< insertion_angle << " horizontal " << horizontal_angle << endl;
    float waiting_time = 0;
    float wiggle_length = turtle_.traj_data.wiggle_amptitude;
    float wiggle_frequency = turtle_.traj_data.wiggle_frequency;
    Rectangle_Params rectangle_params = {drag, pad_back_time, servo_time, servo_time, insertion_angle, horizontal_angle, waiting_time, wiggle_length, wiggle_frequency};
    
    float t_mod = fmod(t, (rectangle_params.period_down + rectangle_params.period_up + rectangle_params.period_left + rectangle_params.period_right + rectangle_params.period_waiting_time));
    // t_mod = rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right + 0.5 * rectangle_params.period_down;
    // t_mod = rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down;
    // t_mod = rectangle_params.period_left + rectangle_params.period_up + rectangle_params.period_right;
    // t_mod = rectangle_params.period_left + rectangle_params.period_up;
    float corres_t = 0;
    float left_hori_servo = 94;// original 100 mannually tuned to 94
    float right_hori_servo = 23;


    //Fixed insertion depth parameters and gamma solver paramters
    
    float l1=0.14;
    float initial_gamma=0.6;
    float turtle_height=0.06;
    float lower_point=0.05;

    // desired insertion depth input
    float desierd_insertion_depth=0.05;


    float tol=1e-5;
    float initial_theta=-45*M_PI/180;
    int maxIteration=500;

    // fixed insertion depth calculation
    float initial_insertion_angle_rad=gammasolver(l1,lower_point,desierd_insertion_depth,turtle_height,initial_theta,initial_gamma, tol, maxIteration );
    float initial_insertion_angle_deg=initial_insertion_angle_rad*180.0/M_PI;
    cout << "TMOD" << t_mod << endl;
    // insertion_angle=insertion_angle+10
    if (t_mod <= rectangle_params.period_up)
    {
        corres_t = t_mod / rectangle_params.period_up;
        cout << "UP corres_t" << corres_t << endl;
        gamma1 = left_hori_servo;
        // Y1 = - r_v + r_v * corres_t;
        theta1 = - horizontal_angle + 2*horizontal_angle*corres_t;
        // -r_v + (r_v + upper_height) * corres_t;
        gamma2 = right_hori_servo;
        // Y2 = - r_v + r_v * corres_t;
        // Y2 = -r_v + (r_v + upper_height) * corres_t;
        theta2 = horizontal_angle - 2*horizontal_angle*corres_t;
    }
    else if (t_mod <= rectangle_params.period_up + rectangle_params.period_right)
    {
        corres_t = (t_mod - rectangle_params.period_up) / rectangle_params.period_right;
        cout << "RIGHT corres_t" << corres_t << endl;
        // X1 = r_h - r_h * 2 * corres_t;
        theta1 = horizontal_angle;
        // Y1 = 0;
        // Y1 = upper_height;
        gamma1 = left_hori_servo - initial_insertion_angle_deg * corres_t;
        // X2 = -r_h + r_h * 2 * corres_t;
        theta2 = -horizontal_angle;
        
        // Y2 = 0;
        // Y2 = upper_height;
        gamma2 = right_hori_servo + initial_insertion_angle_deg* corres_t;
    }
    else if (t_mod <= rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down)
    {
        corres_t = (t_mod - rectangle_params.period_up - rectangle_params.period_right) / rectangle_params.period_down;
        // X1 = -rectangle_params.horizontal_range;
        cout << "DOWN corres_t" << corres_t << endl;
        theta1 = horizontal_angle - 2*horizontal_angle*corres_t;
        // Y1 = - rectangle_params.vertical_range * corres_t;
        // Y1 = upper_height - (r_v + upper_height) * corres_t;
        gamma1 = left_hori_servo - gammasolver(l1,lower_point,desierd_insertion_depth,turtle_height,-theta1*M_PI/180,initial_gamma, tol, maxIteration )*180/M_PI;
        cout << "solved_gamma" << gammasolver(l1,lower_point,desierd_insertion_depth,turtle_height,-theta1*M_PI/180,initial_gamma, tol, maxIteration )*180/M_PI << endl;
        // X2 = rectangle_params.horizontal_range;
        theta2 = -horizontal_angle + 2*horizontal_angle*corres_t;
        // Y2 = - rectangle_params.vertical_range * corres_t;
        // Y2 = upper_height - (r_v + upper_height) * corres_t;
        gamma2 = right_hori_servo + gammasolver(l1,lower_point,desierd_insertion_depth,turtle_height,-theta1*M_PI/180,initial_gamma, tol, maxIteration )*180/M_PI;
        cout << "swipe"  << endl;
    }
    else{
        corres_t = (t_mod - rectangle_params.period_up - rectangle_params.period_right - rectangle_params.period_down) / rectangle_params.period_left;
        theta1 = -horizontal_angle;
        cout << "LEFT corres_t" << corres_t << endl;
        gamma1 = left_hori_servo - initial_insertion_angle_deg +initial_insertion_angle_deg* corres_t;
        theta2 = horizontal_angle;
        gamma2 = right_hori_servo + initial_insertion_angle_deg - initial_insertion_angle_deg * corres_t;
         cout << "extract"  << endl;
    }
    
    
}






// Fixed insertion depth verson3 analytic solution with matlab homogeneous matrix verified

void fixed_insertion_depth_gait_lower_point_version_3_analytic_solution(turtle& turtle_, float t, float& theta2, float& gamma1,float& theta1, float& gamma2){
     //get data
    float horizontal_angle = turtle_.traj_data.lateral_angle_range*180/M_PI;
    float drag = turtle_.traj_data.lateral_angle_range * 0.14 * 2/turtle_.traj_data.drag_speed;
    float servo_time = turtle_.traj_data.servo_time;
    // float pad_back_time = drag;
   float pad_back_time = 0.65;  //speed up 0.35
    float insertion_angle = turtle_.traj_data.insertion_angle*180/M_PI;
   // cout << "angle outp put"<< insertion_angle << " horizontal " << horizontal_angle << endl;
    float waiting_time = 0;
    float wiggle_length = turtle_.traj_data.wiggle_amptitude;
    float wiggle_frequency = turtle_.traj_data.wiggle_frequency;
    Rectangle_Params rectangle_params = {drag, pad_back_time, servo_time, servo_time, insertion_angle, horizontal_angle, waiting_time, wiggle_length, wiggle_frequency};
    
    float t_mod = fmod(t, (rectangle_params.period_down + rectangle_params.period_up + rectangle_params.period_left + rectangle_params.period_right + rectangle_params.period_waiting_time));

    float corres_t = 0;
    float left_hori_servo = 94;// original 100 mannually tuned to 94
    float right_hori_servo = 23;


    //Fixed insertion depth parameters and gamma solver paramters
    
    double l1=0.14; // flipper length
    double turtle_height=0.06; //height from flipper to fround
    double lower_point=0.05;

    // desired insertion depth input
    double desierd_insertion_depth=0.05;

    // fixed insertion depth initial calculation
    double initial_insertion_angle_rad=asin((desierd_insertion_depth+turtle_height)/sqrt((l1*cos(-45*M_PI/180))*(l1*cos(-45*M_PI/180))+lower_point*lower_point))-atan(lower_point/(l1*cos(-45*M_PI/180)));
    double initial_insertion_angle_deg=initial_insertion_angle_rad*180/M_PI;
    cout << "TMOD" << t_mod << endl;
    cout<<"Initial angle"<<initial_insertion_angle_deg<<endl;
    // insertion_angle=insertion_angle+10

    //FIRST phase gamma=0 theta=-45
    if (t_mod <= rectangle_params.period_up)
    {
        corres_t = t_mod / rectangle_params.period_up;
       // cout << "UP corres_t" << corres_t << endl;
        gamma1 = left_hori_servo;
        // Y1 = - r_v + r_v * corres_t;
        theta1 = - horizontal_angle + 2*horizontal_angle*corres_t;
        // -r_v + (r_v + upper_height) * corres_t;
        gamma2 = right_hori_servo;
        // Y2 = - r_v + r_v * corres_t;
        // Y2 = -r_v + (r_v + upper_height) * corres_t;
        theta2 = horizontal_angle - 2*horizontal_angle*corres_t;
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
        gamma1 = left_hori_servo - initial_insertion_angle_rad*180/M_PI * corres_t;
        // X2 = -r_h + r_h * 2 * corres_t;
        theta2 = -horizontal_angle;
        
        // Y2 = 0;
        // Y2 = upper_height;
        gamma2 = right_hori_servo + initial_insertion_angle_rad*180/M_PI* corres_t;
    }

    //
    else if (t_mod <= rectangle_params.period_up + rectangle_params.period_right + rectangle_params.period_down)
    {
        corres_t = (t_mod - rectangle_params.period_up - rectangle_params.period_right) / rectangle_params.period_down;
      //  cout << "DOWN corres_t" << corres_t << endl;
        
        theta1 = horizontal_angle - 2*horizontal_angle*corres_t;
        gamma1 = left_hori_servo -(asin((desierd_insertion_depth+turtle_height)/sqrt((l1*cos(-theta1*M_PI/180))*(l1*cos(-theta1*M_PI/180)+lower_point*lower_point))-atan(lower_point/(l1*cos(-theta1*M_PI/180)))))*180/M_PI;
      
        //cout << "solved_gamma" << gammasolver(l1,lower_point,desierd_insertion_depth,turtle_height,-theta1*M_PI/180,initial_gamma, tol, maxIteration )*180/M_PI << endl;
     cout<<"theta1"<<theta1<<endl;
       cout<<"gamma_analytic_solution"<<(asin((desierd_insertion_depth+turtle_height)/sqrt((l1*cos(-theta1*M_PI/180))*(l1*cos(-theta1*M_PI/180)+lower_point*lower_point))-atan(lower_point/(l1*cos(-theta1*M_PI/180)))))*180/M_PI<<endl;
        theta2 = -horizontal_angle + 2*horizontal_angle*corres_t;
        gamma2 = right_hori_servo + (asin((desierd_insertion_depth+turtle_height)/sqrt((l1*cos(-theta1*M_PI/180))*(l1*cos(-theta1*M_PI/180)+lower_point*lower_point))-atan(lower_point/(l1*cos(-theta1*M_PI/180)))))*180/M_PI;
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
        gamma1 = left_hori_servo -  (initial_insertion_angle_rad*180/M_PI) +(initial_insertion_angle_rad*180/M_PI)* corres_t;
        theta2 = horizontal_angle;
        gamma2 = right_hori_servo + (initial_insertion_angle_rad*180/M_PI) - (initial_insertion_angle_rad*180/M_PI) * corres_t;
       //  cout << "extract"  << endl;
    }
    
}














/**
 * @brief bouding gaits
 * @param time
 * @param bouding gaits
 * @return x y : the coordinates of the toe trajectories
 */

void boundingGAIT(turtle& turtle_, float t, float &theta1, float &gamma1,
                  float &theta2, float &gamma2)
{
    // float x1, y1, x2, y2;
    // rectangle_generator(turtle_, t, x1, y1, x2, y2);

    // PhysicalToAbstract(x1, y1, x2, y2, theta2, gamma1, beta1, theta1, gamma2, beta2);

    // for big turtle, angle movement
    // servo_angle_gait(turtle_,t, theta2, gamma1,theta1, gamma2);
    // fixed_insertion_depth_gait(turtle_,t, theta2, gamma1,theta1, gamma2);
    // fixed_insertion_depth_gait_lower_point(turtle_,t, theta2, gamma1,theta1, gamma2);
    //fixed_insertion_depth_gait_lower_point_version_2_approximate(turtle_,t, theta2, gamma1,theta1, gamma2);
    fixed_insertion_depth_gait_lower_point_version_3_analytic_solution(turtle_,t, theta2, gamma1,theta1, gamma2);
    turtle_.turtle_control.left_adduction.set_input_position_degree.input_position = gamma1;
    turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position = theta1;
    turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = gamma2;
    turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = theta2;
    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = gamma1/360;
    turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = theta1/360;
    turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = gamma2/360;
    turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = theta2/360;
    // used for test
    // float left_add = (gamma1*M_PI)/180;
    // float left_swe = (theta1*M_PI)/180;
    // float right_add = (gamma2*M_PI)/180;
    // float right_swe = (theta2*M_PI)/180;
    // cout << "la"<< left_add << "ls" <<left_swe << endl;
    // cout << "ra"<< right_add << "rs" <<right_swe << endl;
    // cout << " theta1: " << theta1 << " gamma1: " << gamma1 << endl;
    // cout << " theta2: " << theta2 << " gamma2: " << gamma2 << endl;
    // //checkleft(gamma1, beta1, theta1);
    // //checkright(gamma2, beta2, theta2);
    // cout << endl;
    // cout << " theta1: " << theta1 << " gamma1: " << gamma1 << endl;
    // cout << " theta2: " << theta2 << " gamma2: " << gamma2 << endl;
    // cout << endl;
}

/**
 * @brief rhex walking
 * @param time
 * @param bouding gaits
 * @return x y : the coordinates of the toe trajectories
 */

void rhex_walking(float t, float &theta1, float &theta2, float curr_theta1, float curr_theta2)
{
    float speed = 36.4;
    theta1 = curr_theta1 - speed * 0.01;
    theta2 = 0.2 - curr_theta1 + speed * 0.01;
    // theta1 = 0;
    // theta2 = 0.2;
}