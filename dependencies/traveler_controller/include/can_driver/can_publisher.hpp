#include <linux/can/raw.h>
#include <linux/can.h>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "socketcan_interface.hpp"
#include "odrive_can.hpp"

#include "traveler_msgs/msg/odrive_status.hpp"
#include "traveler_msgs/msg/set_input_position.hpp"

class CanPublisher : public rclcpp::Node
{
public:
    CanPublisher(/* args */);
    ~CanPublisher();

private:
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    SocketcanInterface socket_channel0_axis0_read_;
    SocketcanInterface socket_channel0_axis1_read_;
    SocketcanInterface socket_channel1_axis0_read_;
    SocketcanInterface socket_channel1_axis1_read_;
    SocketcanInterface socket_channel0_get_iq_;
    SocketcanInterface socket_channel1_get_iq_;
    SocketcanInterface socket_get_encoder_estimates_0;
    SocketcanInterface socket_get_encoder_estimates_1;
    //SocketcanInterface socket_topic_set_position_0;
    //SocketcanInterface socket_topic_set_position_1;
    traveler_msgs::msg::OdriveStatus odrive_status_msg_0;
    traveler_msgs::msg::OdriveStatus odrive_status_msg_1;
    

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<traveler_msgs::msg::OdriveStatus>::SharedPtr publisher_0;
    rclcpp::Publisher<traveler_msgs::msg::OdriveStatus>::SharedPtr publisher_1;
    //void setPositionCallback(traveler_msgs::msg::SetInputPosition::SharedPtr msg);
    //rclcpp::Subscription<traveler_msgs::msg::SetInputPosition>::SharedPtr Subscription_channel_0;
    //rclcpp::Subscription<traveler_msgs::msg::SetInputPosition>::SharedPtr Subscription_channel_1;
    rclcpp::Clock ros_clock_;
    int count;
    std::chrono::system_clock::time_point now;
    //void timerCallback();
    void updateChannel2StatusCallback();
    void updateChannel1StatusCallback();
    
};