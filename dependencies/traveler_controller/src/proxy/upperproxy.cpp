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
 */

namespace turtle_namespace{
namespace control{

upperproxy::upperproxy() : Node("upperproxy") {
    std::cout<<"Traveler Upper Proxy established" << std::endl;
    GUI_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/drag_times", 10);
    GUI_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>
        ("/Gui_information", 10, std::bind(&upperproxy::handle_gui, this, _1));

    manual_waypoints();
}

upperproxy::upperproxy(std::string name) : Node(name) {
    std::cout<<"Traveler Upper Proxy established" << std::endl;
    GUI_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>
        ("/drag_times", 10);
    GUI_subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>
        ("/Gui_information", 10, std::bind(&upperproxy::handle_gui, this, _1));

    manual_waypoints();
}

void upperproxy::manual_waypoints(){
    auto& td = turtle_inter_.traj_data;
    td.num_waypoints = 0;
    td.waypoints_x.clear();
    td.waypoints_y.clear();
    td.waypoints_v.clear();

    auto push_xy = [&](float x, float y, float v) {
        td.waypoints_x.push_back(x);
        td.waypoints_y.push_back(y);
        td.waypoints_v.push_back(v);
        td.num_waypoints++;
        printf("Waypoint %d (manual): (%.3f, %.3f), v=%.3f\n",
               td.num_waypoints, x, y, v);
    };
    
    push_xy(0.10f, 0.12f, 0.03f);
    push_xy(0.12f, 0.08f, 0.03f);
    push_xy(0.09f, 0.15f, 0.03f);
    push_xy(0.11f, 0.11f, 0.03f);
}

void upperproxy::handle_gui(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    (void)msg; 
    manual_waypoints();
}

void upperproxy::handle_trajectory_points(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    (void)msg;
    // Implementation for handling trajectory points from GUI
}

void upperproxy::UpdateGuiCommand(turtle& turtle_) {
    turtle_.traj_data = turtle_inter_.traj_data;
}

void upperproxy::PublishStatusFeedback(turtle& turtle_) {
    if(turtle_.turtle_gui.status_update_flag == true) {
        auto message = std_msgs::msg::Float64MultiArray();
        GUI_publisher->publish(message);
        turtle_.turtle_gui.status_update_flag = false;
    }
}

void upperproxy::GenerateTrajectoryFromWaypoints() {
    // Implementation for generating trajectory from waypoints
    // This can be expanded based on specific requirements
}

} //namespace control
} //namespace turtle_namespace

