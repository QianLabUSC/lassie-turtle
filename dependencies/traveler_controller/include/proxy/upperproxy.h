/*
 * @Author: Ryoma Liu -- ROBOLAND
 * @Date: 2021-11-27 16:00:26
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2021-11-27 21:23:15
 */

#ifndef UPPER_PROXY_H_
#define UPPER_PROXY_H_
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "proxy/control_data.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "traveler_msgs/msg/traveler_config.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "traveler_msgs/msg/traveler_mode.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
namespace traveler_namespace
{
	namespace control
	{

		class upperproxy : public rclcpp::Node
		{
		public:
			upperproxy(std::string name = "upper_proxy");
			void UpdateGuiCommand(Traveler &);
			void PublishStatusFeedback(Traveler &);

		private:
			Traveler traveler_leg;
			rclcpp::TimerBase::SharedPtr _timer;
			// rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
			// 	GUI_publisher;
			void handle_gui(const traveler_msgs::msg::TravelerConfig::SharedPtr msg);
			void handle_start(const traveler_msgs::msg::TravelerMode::SharedPtr msg);
			rclcpp::Subscription<traveler_msgs::msg::TravelerConfig>::SharedPtr
				GUI_subscriber;
			rclcpp::Subscription<traveler_msgs::msg::TravelerMode>::SharedPtr
				start_subscriber;
			
		};

	} // namespace control
} // namespace traveler_namespace

#endif