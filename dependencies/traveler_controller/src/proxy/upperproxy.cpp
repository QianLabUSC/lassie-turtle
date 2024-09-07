/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-21 21:58:00 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2021-11-28 14:38:09
 */

#include "proxy/upperproxy.h"


/**
 * upperproxy - class to collect robot's information and trajectories from path
 * planning and decision making part. 
 * agile taur.
 */

namespace turtle_namespace{
namespace control{

upperproxy::upperproxy(std::string name) : Node(name){
    std::cout<<"Traveler Upper Proxy established"
                <<std::endl;
    GUI_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/drag_times", 10);
    GUI_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>
        ("/Gui_information", 10, std::bind(&upperproxy::handle_gui, this, _1));
}

void upperproxy::handle_gui
    (const std_msgs::msg::Float64MultiArray::SharedPtr msg){
        // int len = msg->data.size();
        


            
            
            turtle_inter_.turtle_gui.start_flag = msg->data[0]; 
            turtle_inter_.turtle_gui.drag_traj = msg->data[1];
            turtle_inter_.traj_data.lateral_angle_range = msg->data[7];
            turtle_inter_.traj_data.insertion_depth = msg->data[6];
            turtle_inter_.traj_data.penetration_speed = msg->data[2];
            turtle_inter_.traj_data.drag_speed = msg->data[3];
            turtle_inter_.traj_data.servo_speed = msg->data[4];
            turtle_inter_.traj_data.back_speed = msg->data[5];
            std::cout << "trajectroy " <<turtle_inter_.turtle_gui.drag_traj<< std::endl;

            // turtle_inter_.traj_data.extraction_angle = msg->data[6];
            // turtle_inter_.traj_data.wiggle_frequency = msg->data[7];
            
            // turtle_inter_.traj_data.wiggle_amptitude = msg->data[9];
        
        
        
    }

void upperproxy::UpdateGuiCommand(turtle& turtle_){
    turtle_.turtle_gui = turtle_inter_.turtle_gui;
    turtle_.traj_data = turtle_inter_.traj_data;
}
void upperproxy::PublishStatusFeedback(turtle& turtle_){
    if(turtle_.turtle_gui.status_update_flag == true){
        auto message = std_msgs::msg::Float64MultiArray();
        // std::cout <<  message.data[message.data.size() - 1] << std::endl;
        GUI_publisher->publish(message);
        turtle_.turtle_gui.status_update_flag = false;
    }
    
}

} //namespace control
} //namespace turtle_namespace