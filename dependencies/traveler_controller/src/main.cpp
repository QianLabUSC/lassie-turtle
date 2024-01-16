/*
 * @Author: Ryoma Liu -- ROBOLAND
 * @Date: 2021-11-27 16:20:05
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2022-02-02 19:12:01
 */

#include "main.h"
#include "rclcpp/rclcpp.hpp"


/**
 * main - entrance of turtle robot Scontroller.
 * @param argc
 * @param argv
 * @return 0
 */

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv); // initial ros
	// receive information from high level sensor
	//rclcpp::executors::MultiThreadedExecutor exec;
	std::shared_ptr<upperproxy> Upper_proxy_ = std::make_shared<upperproxy>();

	// detect motor status, and publish motion command
	std::shared_ptr<lowerproxy> Lower_proxy_ = std::make_shared<lowerproxy>();
	std::shared_ptr<can_driver> Can_driver_  = std::make_shared<can_driver>();
    //std::shared_ptr<CanSuber> Can_suber_ = std::make_shared<CanSuber>();
   
   // rclcpp::spin(Can_suber_);

    //rclcpp::shutdown();
    rclcpp::Rate loop_rate(1000); 
    
	while (rclcpp::ok())
	{
		rclcpp::spin_some(Upper_proxy_);
		rclcpp::spin_some(Lower_proxy_);
		Can_driver_->get_motor_status(turtle_);
		Lower_proxy_->UpdateJoystickStatus(turtle_);
		Upper_proxy_->UpdateGuiCommand(turtle_); 
		
        Can_driver_->change_odrive_state(turtle_);
		Lower_proxy_->calculate_position(turtle_);  
		
		Can_driver_->setControl(turtle_);
		
		loop_rate.sleep();
		
	}

	 return 0;
}
