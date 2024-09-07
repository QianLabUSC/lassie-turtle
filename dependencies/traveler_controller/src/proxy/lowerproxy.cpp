/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-21 21:58:00 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2022-02-02 18:31:48
 */

#include "proxy/lowerproxy.h"
#include "controller/inverse_kinematics.h"
#include <fstream>
#include <iostream>
#include <iomanip>

/**
 * lowerproxy - class to publish control command to webots turtle_namespace or real 
 * agile taur.
 */
const double PI = 3.141592653589793238463;
const double TWO_PI = PI*2;




namespace turtle_namespace{
namespace control{


lowerproxy::lowerproxy(std::string name) : Node(name){
    std::cout<<"Start to create the ros node subscriber and publisher"
                <<std::endl;
   
    controller_state_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/robot_state", 10);
    

    _count = 0;

    RCLCPP_INFO(this->get_logger(), "Publisher created!!");                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
}

float lowerproxy::fmodf_mpi_pi(float f)
{
  if (f>0)
    return (fmodf(f+PI, TWO_PI) - PI);
  else
    return (fmodf(f-PI, TWO_PI) + PI);
}


/// @brief estimate terrain in this function
void lowerproxy::terrain_estimation(){

}



/// @brief calculate the motor command using different controller:
///        default: inverse kinematic controller
/// @param turtle
void lowerproxy::calculate_position(turtle &turtle_ )
    {
        /**
         * The motor positions must be converted from Radians to Turns
         * 
         * The controller interprets angular position in radians, but the ODrive
         * uses turns as its angular unit.
        */
        //float axis0_pos = (-1.0f * turtle_.turtle_control.Leg_lf.axis0.motor_control_position + M0_OFFSET + (M_PI / 2)) / (2 * M_PI);
        //float axis1_pos = (traveler_.traveler_control.Leg_lf.axis1.motor_control_position - (M_PI / 2) + M1_OFFSET) / (2 * M_PI);
        
 
       // clamp both motor angles to within 1 turn;
        

        // auto message_channel_0_axis0 = traveler_msgs::msg::SetInputPosition();
        // message_channel_0.can_channel = 0;
        // message_channel_0.axis = 0;
        // message_channel_0.input_position = axis0_pos;
        // //message_channel_0.input_position = sign ;
        // message_channel_0.vel_ff = 0;
        // message_channel_0.torque_ff = 0;

        // auto message_channel_1 = traveler_msgs::msg::SetInputPosition();
        // message_channel_1.can_channel = 1;
        // message_channel_1.axis = 0;
        // message_channel_1.input_position = axis1_pos;
        // message_channel_1.vel_ff = 0;
        // message_channel_1.torque_ff = 0;
        // // instead of publiishing to ros topics, publish to local class
        // turtle_.turtle_control.Leg_lf.axis0.set_input_position = message_channel_0;
        // turtle_.turtle_control.Leg_lf.axis1.set_input_position = message_channel_1;
        // Position_publisher_channel_0->publish(message_channel_0);
        // Position_publisher_channel_1->publish(message_channel_1);

        
        
        if(turtle_.turtle_gui.start_flag==1){
                t2 = std::chrono::high_resolution_clock::now();
                std::chrono::high_resolution_clock::time_point current_time_pre;
                std::chrono::duration<double> deltaTime_pre;

                std::chrono::high_resolution_clock::time_point current_time;
                std::chrono::duration<double> deltaTime;

                std::chrono::high_resolution_clock::time_point current_time1;
                std::chrono::duration<double> deltaTime1;

                std::chrono::high_resolution_clock::time_point TurtleBackCurrentTime;
                std::chrono::duration<double> TurtleBackRunningTime;
                double TurtleBackRunningTimeCount;

                std::chrono::high_resolution_clock::time_point TurtlePenetrationCurrentTime;
                std::chrono::duration<double> TurtlePenetrationRunningTime;
                double TurtlePenetrationRunningTimeCount;   

                std::chrono::high_resolution_clock::time_point TurtleSweepingCurrentTime;
                std::chrono::duration<double> TurtleSweepingRunningTime;
                double TurtleSweepingRunningTimeCount;   

                std::chrono::high_resolution_clock::time_point TurtleExtractionCurrentTime;
                std::chrono::duration<double> TurtleExtractionRunningTime;
                double TurtleExtractionRunningTimeCount;      

                double pre_t;
                double curr_initial_phase_time;
                double running_t;
                double corres_t;

                switch (currentState) {
                    case ProgramState::FirstIteration:                       
                        turtle_.turtle_chassis.gait_state = -1;
                        // Additional code for first iteration
                        std::cout << "First Interation: " << std::endl;
                        starting_time = std::chrono::high_resolution_clock::now();
                        currentState = ProgramState::SetToControlAndCalibrate;
                        turtle_.turtle_control.if_control = false;
                        break;

                    case ProgramState::SetToControlAndCalibrate:
                        current_time_pre = std::chrono::high_resolution_clock::now();
                        deltaTime_pre = current_time_pre - starting_time;
                        pre_t = deltaTime_pre.count();
                        if(turtle_.turtle_chassis.if_idle_count <= 0 && pre_t > set_close_control_time){
                            currentState = ProgramState::GoToInitialPoint;
                        }
                        std::cout << "waiting Interation: " << std::endl;
                        saved_left_adduction=turtle_.turtle_chassis.left_adduction.pos_estimate;
                        saved_left_sweeping= turtle_.turtle_chassis.left_sweeping.pos_estimate;
                        saved_right_adduction= turtle_.turtle_chassis.right_adduction.pos_estimate;
                        saved_right_sweeping= turtle_.turtle_chassis.right_sweeping.pos_estimate; 
                        std::cout << "saved_left_sweeping" << saved_left_sweeping << std::endl;
                        std::cout << "saved_left_adduction" << saved_left_adduction << std::endl;
                        std::cout << "saved_right_sweeping" << saved_right_sweeping << std::endl;
                        std::cout << "saved_right_adduction" << saved_right_adduction << std::endl;
                        turtle_.turtle_control.if_control = false;
                        break;
                        

                    case ProgramState::GoToInitialPoint:
                        current_time = std::chrono::high_resolution_clock::now();
                        deltaTime = current_time - starting_time;
                        curr_initial_phase_time = deltaTime.count() - set_close_control_time;
                        turtle_.turtle_chassis.gait_state = 0;
                        turtle_.turtle_control.if_control = true;
                        if (curr_initial_phase_time < initial_phase_time) {
                            goback2desiredangle(turtle_, 
                                -turtle_.traj_data.extraction_angle, 
                                turtle_.traj_data.lateral_angle_range, 
                                turtle_.traj_data.extraction_angle, 
                                -turtle_.traj_data.lateral_angle_range,
                                saved_left_adduction, saved_left_sweeping,
                                saved_right_adduction, saved_right_sweeping,
                                curr_initial_phase_time, initial_phase_time);
                        } else {
                            currentState = ProgramState::Running;
                        }
                        // Additional code for first iteration
                        std::cout << "go to initial point Interation: " << curr_initial_phase_time << std::endl;
                        break;

                    case ProgramState::Running:

                        current_time1 = std::chrono::high_resolution_clock::now();
                        deltaTime1 = current_time1 - starting_time;
                        curr_initial_phase_time = deltaTime1.count();
                        turtle_.turtle_control.if_control = true;
                        running_t = curr_initial_phase_time- initial_phase_time - set_close_control_time;
                        double gamma1 = 0;
                        double theta1 = 0;
                        double gamma2 = 0;
                        double theta2 = 0;
                        float left_hori_servo = 0; // 原始值 100 手动调整为 94
                        float right_hori_servo = 0;
                      

                        Rectangle_Params rectangle_params;
                        
                        switch(TurtleCurrentPhase){
                        
                            case Turtlephase::GetTurtleGUICommand:
                                //Fixed insertion depth parameters and gamma solver parameters


                                //get data this is slow version 
                                // float horizontal_angle = turtle_.traj_data.lateral_angle_range * 180 / M_PI;
                             
                                rectangle_params.l1 = 0.120; // flipper length-new shorter flipper
                                rectangle_params.turtle_height = 0.0645; //height from flipper to ground
                                rectangle_params.lower_point = 0.05;
                                rectangle_params.period_down = turtle_.traj_data.lateral_angle_range * rectangle_params.l1 * 2 / turtle_.traj_data.drag_speed;
                                rectangle_params.period_up = 0.8; //customize back phase time
                                rectangle_params.period_left = turtle_.traj_data.servo_speed;
                                rectangle_params.period_right = turtle_.traj_data.servo_speed;
                                rectangle_params.vertical_range = turtle_.traj_data.insertion_depth; // depend on insertion depth
                                rectangle_params.horizontal_range = turtle_.traj_data.lateral_angle_range * 180 / M_PI;
                                rectangle_params.period_waiting_time = 0;
                                rectangle_params.extraction_angle = turtle_.traj_data.extraction_angle;
                                if (rectangle_params.vertical_range > 0.07) {
                                    rectangle_params.vertical_range = 0.07;
                                }

                                std::cout << "desired insertion depth(m)" << rectangle_params.vertical_range << std::endl; // m

                                
                                // fixed insertion depth initial calculation
                                rectangle_params.initial_insertion_depth_rad = asin((rectangle_params.vertical_range + rectangle_params.turtle_height) / sqrt((rectangle_params.l1 * cos(rectangle_params.horizontal_range * M_PI / 180)) * (rectangle_params.l1 * cos(rectangle_params.horizontal_range * M_PI / 180)) + rectangle_params.lower_point * rectangle_params.lower_point)) - atan(rectangle_params.lower_point / (rectangle_params.l1 * cos(rectangle_params.horizontal_range * M_PI / 180)));
                                rectangle_params.initial_insertion_depth_deg = rectangle_params.initial_insertion_depth_rad * 180 / M_PI;
                                // cout << "TMOD" << t_mod << endl;

                                TurtleRunningstarting_time= std::chrono::high_resolution_clock::now();
                                TurtleCurrentPhase=Turtlephase::TurleBack;
                                break;
                            case Turtlephase::TurleBack:
                                TurtleBackCurrentTime = std::chrono::high_resolution_clock::now();
                                TurtleBackRunningTime =TurtleBackCurrentTime-TurtleRunningstarting_time;
                                TurtleBackRunningTimeCount=TurtleBackRunningTime.count();

                                if(TurtleBackRunningTimeCount<=rectangle_params.period_up){
                                    corres_t = TurtleBackRunningTimeCount / rectangle_params.period_up;
                                    gamma1 = left_hori_servo + rectangle_params.extraction_angle;
                                    theta1 = - rectangle_params.horizontal_range + 2 * rectangle_params.horizontal_range * corres_t;
                                    gamma2 = right_hori_servo - rectangle_params.extraction_angle;
                                    theta2 = rectangle_params.horizontal_range - 2 * rectangle_params.horizontal_range * corres_t;
                                    turtle_.turtle_chassis.gait_state = 1;
                                    std::cout << "BackPhase" << std::endl;

                                    // 设置伺服位置
                                    turtle_.turtle_control.left_adduction.set_input_position_degree.input_position = gamma1;
                                    turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position = theta1;
                                    turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = gamma2;
                                    turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = theta2;
                                    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = -gamma1 / 360;
                                    turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = -theta1 / 360;
                                    turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = -gamma2 / 360;
                                    turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = -theta2 / 360;
                                }else{
                                    TurtleCurrentPhase = Turtlephase::TurtlePentration;
                                }
                                break;



                            case Turtlephase::TurtlePentration:
                                TurtlePenetrationCurrentTime = std::chrono::high_resolution_clock::now();
                                TurtlePenetrationRunningTime =TurtlePenetrationCurrentTime-TurtleRunningstarting_time;
                                TurtlePenetrationRunningTimeCount=TurtlePenetrationRunningTime.count()-rectangle_params.period_up;

                                if(TurtlePenetrationRunningTimeCount<=rectangle_params.period_right){
                                    corres_t = TurtlePenetrationRunningTimeCount / rectangle_params.period_right;
                                    theta1 = rectangle_params.horizontal_range;
                                    gamma1 = left_hori_servo + rectangle_params.extraction_angle - (rectangle_params.initial_insertion_depth_rad * 180 / M_PI + rectangle_params.extraction_angle) * corres_t;
                                    theta2 = -rectangle_params.horizontal_range;
                                    gamma2 = right_hori_servo - rectangle_params.extraction_angle + (rectangle_params.initial_insertion_depth_rad * 180 / M_PI + rectangle_params.extraction_angle) * corres_t;
                                    turtle_.turtle_chassis.gait_state = 2;
                                    std::cout << "PenetrationPhase" << std::endl;

                                    // 设置伺服位置
                                    turtle_.turtle_control.left_adduction.set_input_position_degree.input_position = gamma1;
                                    turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position = theta1;
                                    turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = gamma2;
                                    turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = theta2;
                                    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = -gamma1 / 360;
                                    turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = -theta1 / 360;
                                    turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = -gamma2 / 360;
                                    turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = -theta2 / 360;
                                }else{
                                    TurtleCurrentPhase = Turtlephase::TurtleSweeping;
                                }
                                break;

                            case Turtlephase::TurtleSweeping:

                                TurtleSweepingCurrentTime = std::chrono::high_resolution_clock::now();
                                TurtleSweepingRunningTime =TurtleSweepingCurrentTime-TurtleRunningstarting_time;
                                TurtleSweepingRunningTimeCount=TurtleSweepingRunningTime.count()-rectangle_params.period_up-rectangle_params.period_right;

                                if(TurtleSweepingRunningTimeCount<=rectangle_params.period_down){
                                    corres_t = TurtleSweepingRunningTimeCount / rectangle_params.period_down;

                                    theta1 = rectangle_params.horizontal_range - 2 * rectangle_params.horizontal_range * corres_t;
                                    gamma1 = left_hori_servo - (asin((rectangle_params.vertical_range + rectangle_params.turtle_height) / sqrt((rectangle_params.l1 * cos(-theta1 * M_PI / 180)) * (rectangle_params.l1 * cos(-theta1 * M_PI / 180)) + rectangle_params.lower_point * rectangle_params.lower_point)) - atan(rectangle_params.lower_point / (rectangle_params.l1 * cos(-theta1 * M_PI / 180)))) * 180 / M_PI;
                                    theta2 = -rectangle_params.horizontal_range + 2 * rectangle_params.horizontal_range * corres_t;
                                    gamma2 = right_hori_servo + (asin((rectangle_params.vertical_range + rectangle_params.turtle_height) / sqrt((rectangle_params.l1 * cos(-theta1 * M_PI / 180)) * (rectangle_params.l1 * cos(-theta1 * M_PI / 180)) + rectangle_params.lower_point * rectangle_params.lower_point)) - atan(rectangle_params.lower_point / (rectangle_params.l1 * cos(-theta1 * M_PI / 180)))) * 180 / M_PI;
                                    turtle_.turtle_chassis.gait_state = 3;
                                    std::cout << "SweepingPhase" << std::endl;

                                    // 设置伺服位置
                                    turtle_.turtle_control.left_adduction.set_input_position_degree.input_position = gamma1;
                                    turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position = theta1;
                                    turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = gamma2;
                                    turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = theta2;
                                    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = -gamma1 / 360;
                                    turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = -theta1 / 360;
                                    turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = -gamma2 / 360;
                                    turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = -theta2 / 360;
                                }else{
                                    TurtleCurrentPhase = Turtlephase::TurtleExtraction;
                                }
                                break;
                            

                            case Turtlephase::TurtleExtraction:

                                TurtleExtractionCurrentTime = std::chrono::high_resolution_clock::now();
                                TurtleExtractionRunningTime =TurtleExtractionCurrentTime-TurtleRunningstarting_time;
                                TurtleExtractionRunningTimeCount=TurtleExtractionRunningTime.count()-rectangle_params.period_up-rectangle_params.period_right-rectangle_params.period_down;

                                if(TurtleExtractionRunningTimeCount<=rectangle_params.period_left){
                                    corres_t = TurtleExtractionRunningTimeCount / rectangle_params.period_left;

                                    theta1 = -rectangle_params.horizontal_range;
                                    gamma1 = left_hori_servo - (rectangle_params.initial_insertion_depth_rad * 180 / M_PI) + (rectangle_params.initial_insertion_depth_rad * 180 / M_PI + rectangle_params.extraction_angle) * corres_t;
                                    theta2 = rectangle_params.horizontal_range;
                                    gamma2 = right_hori_servo + (rectangle_params.initial_insertion_depth_rad * 180 / M_PI) - (rectangle_params.initial_insertion_depth_rad * 180 / M_PI + rectangle_params.extraction_angle) * corres_t;
                                    turtle_.turtle_chassis.gait_state = 4;
                                    std::cout << "ExtractionPhase" << std::endl;

                                    // 设置伺服位置
                                    turtle_.turtle_control.left_adduction.set_input_position_degree.input_position = gamma1;
                                    turtle_.turtle_control.left_sweeping.set_input_position_degree.input_position = theta1;
                                    turtle_.turtle_control.right_adduction.set_input_position_degree.input_position = gamma2;
                                    turtle_.turtle_control.right_sweeping.set_input_position_degree.input_position = theta2;
                                    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position = -gamma1 / 360;
                                    turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position = -theta1 / 360;
                                    turtle_.turtle_control.right_adduction.set_input_position_radian.input_position = -gamma2 / 360;
                                    turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position = -theta2 / 360;
                                }else{
                                    TurtleCurrentPhase = Turtlephase::GetTurtleGUICommand;
                                }
                                break;
        
    

                            
                        }

                        // boundingGAIT(turtle_, running_t);           
                        // std::cout << "running Interation: " << running_t << std::endl;
                        break;
                }

       
            }
            else{
                turtle_.turtle_chassis.gait_state = 5;
                currentState = ProgramState::FirstIteration;
                TurtleCurrentPhase = Turtlephase::GetTurtleGUICommand;
                turtle_.turtle_control.if_control = false;
                // probably not used 
                auto current_back_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> deltaTime = current_back_time - t2;
                double back_curr_time = deltaTime.count();
                turtle_.turtle_chassis.step_count=0;
                // goback2desiredangle(turtle_,0,0,0,0,
                //                     saved_left_adduction, saved_left_sweeping,
                //                     saved_right_adduction, saved_right_sweeping,
                //                         back_curr_time,initial_phase_time);
                
               
            }
        
      
    }

void lowerproxy::goback2desiredangle(turtle& turtle_, float left_adduction, 
                                    float left_sweeping, float right_adduction,
                                    float right_sweeping,  float start_left_adduction, 
                                    float start_left_sweeping, float start_right_adduction,  
                                    float start_right_sweeping,  
                                    float t_decrease_time,float total_time)
{
    // right and left sweeping angle is no longer for use, the desired angle is determined by GUI theta range
    left_adduction = left_adduction/360;
    left_sweeping = left_sweeping/TWO_PI;
    right_adduction = right_adduction/360;
    right_sweeping = right_sweeping/TWO_PI;
    total_time = total_time/2;
    if( t_decrease_time>total_time)
   {
     t_decrease_time=total_time;
      turtle_.turtle_control.left_adduction.set_input_position_radian.input_position= start_left_adduction+ (left_adduction-start_left_adduction);
      turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position  =start_left_sweeping+ (left_sweeping-start_left_sweeping);
      turtle_.turtle_control.right_adduction.set_input_position_radian.input_position=start_right_adduction + (right_adduction-start_right_adduction);
      turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position= start_right_sweeping+(right_sweeping-start_right_sweeping);
  
   }
   else
   {

    //left_adduciton should be in turns(unit)
    turtle_.turtle_control.left_adduction.set_input_position_radian.input_position= start_left_adduction+ (left_adduction-start_left_adduction)*(t_decrease_time/total_time);
    turtle_.turtle_control.left_sweeping.set_input_position_radian.input_position  =start_left_sweeping+ (left_sweeping-start_left_sweeping)*(t_decrease_time/total_time);
    turtle_.turtle_control.right_adduction.set_input_position_radian.input_position=start_right_adduction + (right_adduction-start_right_adduction)*(t_decrease_time/total_time);
    turtle_.turtle_control.right_sweeping.set_input_position_radian.input_position= start_right_sweeping+(right_sweeping-start_right_sweeping)*(t_decrease_time/total_time);
   }
}

void lowerproxy::Estop(){

}

void lowerproxy::UpdateJoystickStatus(turtle& turtle_){

    // instead of reading message from ros2, directly call the function 
    // in odrivepro drive to get the message
    // use the intermediate structure instead of the raw turtle_ to avoid messy
    turtle_inter_ = turtle_;

    // get raw encoder estimate from ODrive in unit of turns
    // convert to radians
    // auto odrive0 = turtle_inter_.turtle_chassis.left_adduction;
    // auto odrive1 = turtle_inter_.turtle_chassis.left_sweeping;
    // auto odrive2 = turtle_inter_.turtle_chassis.left_adduction;
    // auto odrive3 = turtle_inter_.turtle_chassis.left_sweeping;
    

    // clamp the position estimate from [0, 2pi]
    // pos_estimate_rad = fmodf_0_2pi(pos_estimate_rad);

    // need to modify here to get the status feedback
    // traveler_leg_.traveler_chassis.Leg_lf.axis0.effort = odrive0.iq_measured * TORQUE_CONST;
    // traveler_leg_.traveler_chassis.Leg_lf.axis0.position = 
    //     -1.0f * (odrive0.pos_estimate) * 2 * M_PI + M0_OFFSET + (M_PI/2);

    // traveler_leg_.traveler_chassis.Leg_lf.axis1.effort = odrive1.iq_measured * TORQUE_CONST;
    // traveler_leg_.traveler_chassis.Leg_lf.axis1.position =
    //     (odrive1.pos_estimate) * 2 * M_PI + (M_PI/2) - M1_OFFSET;


    // to be implemented
    terrain_estimation();


    // to publish information back to gui
    auto robot_state = std_msgs::msg::Float64MultiArray();
    robot_state.data.push_back(turtle_inter_.turtle_chassis.gait_state); // state flag
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_adduction.pos_estimate); //left adduction motor position status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_sweeping.pos_estimate); //left sweeping motor position status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_adduction.pos_estimate); //right adduction motor position status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_sweeping.pos_estimate); //right sweeping motor position status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_adduction.iq_setpoint); //left adduction motor torque status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.left_sweeping.iq_setpoint); //left sweeping motor torque status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_adduction.iq_setpoint); //right sweeping motor torque status
    robot_state.data.push_back(turtle_inter_.turtle_chassis.right_sweeping.iq_setpoint); //left adduction motor torque status
    robot_state.data.push_back(turtle_inter_.turtle_control.left_adduction.set_input_position_radian.input_position);
    robot_state.data.push_back(turtle_inter_.turtle_control.left_sweeping.set_input_position_radian.input_position);
    robot_state.data.push_back(turtle_inter_.turtle_control.right_adduction.set_input_position_radian.input_position);
    robot_state.data.push_back(turtle_inter_.turtle_control.right_sweeping.set_input_position_radian.input_position);
    
    // add more information that related to robot state here
    /*
    something important
    */
    
    controller_state_publisher->publish(robot_state);



    
    
}
    
} //namespace control
} //namespace turtle_namespace



