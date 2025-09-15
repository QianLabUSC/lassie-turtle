/*
 * @Author: Ryoma Liu -- ROBOLAND
 * @Date: 2021-11-27 16:20:05
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2022-02-02 19:12:01
 */

#include "main.h"
#include "rclcpp/rclcpp.hpp"
#include "controller/trajectories_parser.h"

auto& traj = turtle_namespace::control::TrajectoriesParser::getTrajParser();

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // initial ros

    std::shared_ptr<upperproxy> Upper_proxy_ = std::make_shared<upperproxy>();
    std::shared_ptr<lowerproxy> Lower_proxy_ = std::make_shared<lowerproxy>();
    std::shared_ptr<can_driver> Can_driver_  = std::make_shared<can_driver>();

    rclcpp::Rate loop_rate(1000);

    // -----------------------------
    // BENCH TEST: bypass GUI
    // -----------------------------
    turtle_.turtle_gui.start_flag = 1;                // force run immediately
    turtle_.turtle_control.if_control = true;         // allow commands through

    turtle_.traj_data.num_waypoints = 4;
    turtle_.traj_data.waypoints_x = {0.10f, 0.12f, 0.09f, 0.11f};
    turtle_.traj_data.waypoints_y = {0.12f, 0.08f, 0.15f, 0.11f};
    turtle_.traj_data.waypoints_v = {0.03f, 0.03f, 0.03f, 0.03f};

    while (rclcpp::ok())
    {
        rclcpp::spin_some(Upper_proxy_);
        rclcpp::spin_some(Lower_proxy_);

        Can_driver_->get_motor_status(turtle_);
        Lower_proxy_->UpdateJoystickStatus(turtle_);
        Upper_proxy_->UpdateGuiCommand(turtle_);

        // trajectory stepper
        traj.generateTempTraj(turtle_);

        Can_driver_->change_odrive_state(turtle_);
        Lower_proxy_->calculate_position(turtle_);
        Can_driver_->setControl(turtle_);

        loop_rate.sleep();
    }

    return 0;
}



