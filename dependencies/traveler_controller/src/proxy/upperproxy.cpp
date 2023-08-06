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

namespace traveler_namespace{
namespace control{

upperproxy::upperproxy(std::string name) : Node(name){
    std::cout<<"Traveler Upper Proxy established"
                <<std::endl;
    // GUI_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
    //     ("/drag_times", 10);
    GUI_subscriber = this->create_subscription<traveler_msgs::msg::TravelerConfig>
        ("/traveler/config", 10, std::bind(&upperproxy::handle_gui, this, _1));
    start_subscriber = this->create_subscription<traveler_msgs::msg::TravelerMode>
        ("/traveler/start_flag", 10, std::bind(&upperproxy::handle_start, this, _1));
}

void upperproxy::handle_start
    (const traveler_msgs::msg::TravelerMode::SharedPtr msg){
        traveler_leg.traveler_gui.start_flag = msg->start_flag;
        traveler_leg.traveler_gui.drag_traj = msg->traveler_mode;
    }

void upperproxy::handle_gui
    (const traveler_msgs::msg::TravelerConfig::SharedPtr msg){
        // int len = msg->data.size();
        // traveler_leg.traveler_gui.start_flag = msg->start_flag;
        // traveler_leg.traveler_gui.drag_traj = msg->drag_traj;
        traveler_leg.traj_data.extrude_speed = msg->extrude_speed / 100.0f;
        traveler_leg.traj_data.extrude_angle = (msg->extrude_angle / -180 * M_PI) + M_PI;
        traveler_leg.traj_data.extrude_depth = msg->extrude_depth / 100.0f;
        traveler_leg.traj_data.shear_penetration_depth = msg->shear_penetration_depth / 100.0f;
        traveler_leg.traj_data.shear_penetration_speed = msg->shear_penetration_speed / 100.0f;
        traveler_leg.traj_data.shear_penetration_delay = msg->shear_penetration_delay;
        traveler_leg.traj_data.shear_length = msg->shear_length/ 100.0f;
        traveler_leg.traj_data.shear_speed = msg->shear_speed/ 100.0f;
        traveler_leg.traj_data.shear_delay = msg->shear_delay;
        traveler_leg.traj_data.shear_return_speed = msg->shear_return_speed / 100.0f;
        traveler_leg.traj_data.workspace_angular_speed = msg->workspace_angular_speed / 100.0f;
        traveler_leg.traj_data.workspace_moving_angle = msg->workspace_moving_angle / 180 * M_PI;
        traveler_leg.traj_data.workspace_time_delay = msg->workspace_time_delay;
        traveler_leg.traj_data.static_length = msg->static_length / 100.0f;
        traveler_leg.traj_data.static_angle = (msg->static_angle / -180 * M_PI) + M_PI;
        traveler_leg.traj_data.search_start = msg->search_start / 100.0f;
        traveler_leg.traj_data.search_end = msg->search_end / 100.0f;
        traveler_leg.traj_data.ground_height = msg->ground_height / 100.0f;
        traveler_leg.traj_data.back_speed = msg->back_speed / 100.0f;
    }

void upperproxy::UpdateGuiCommand(Traveler& traveler_){
    traveler_.traveler_gui = traveler_leg.traveler_gui;
    traveler_.traj_data = traveler_leg.traj_data;
}
// void upperproxy::PublishStatusFeedback(Traveler& traveler_){
//     if(traveler_.traveler_gui.status_update_flag == true){
//         auto message = std_msgs::msg::Float64MultiArray();
//         // std::cout <<  message.data[message.data.size() - 1] << std::endl;
//         GUI_publisher->publish(message);
//         traveler_.traveler_gui.status_update_flag = false;
//     }
    
// }

} //namespace control
} //namespace traveler_namespace