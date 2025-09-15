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
#include "std_msgs/msg/float64_multi_array.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
namespace turtle_namespace
{
	namespace control
	{

		// Structure to hold a trajectory waypoint from the GUI
		struct TrajectoryWaypoint {
			double time;     // Time to reach this point
			double gamma;    // Gamma angle
			double theta;    // Theta angle
		};

		// Add to the upperproxy class
		class upperproxy : public rclcpp::Node {
		public:
			upperproxy();
			upperproxy(std::string name);
			void handle_gui(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
			void handle_trajectory_points(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
			void UpdateGuiCommand(turtle &turtle_);
			void PublishStatusFeedback(turtle &turtle_);
			
			// New method to handle trajectory points
			void GenerateTrajectoryFromWaypoints();
			
		private:
			rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr GUI_publisher;
			rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr GUI_subscriber;
			rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr trajectory_subscriber;
			
			turtle turtle_inter_;
			std::vector<TrajectoryWaypoint> waypoints;
		};

	} // namespace control
} // namespace turtle_namespace

#endif

