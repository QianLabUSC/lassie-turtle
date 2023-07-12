#include <linux/can/raw.h>
#include <linux/can.h>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "socketcan_interface.hpp"
#include "odrive_can.hpp"

#include "odrive_pro_srvs_msgs/msg/odrive_status.hpp"

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
    odrive_pro_srvs_msgs::msg::OdriveStatus odrive_status_msg;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<odrive_pro_srvs_msgs::msg::OdriveStatus>::SharedPtr publisher_;
    rclcpp::Clock ros_clock_;

    void timerCallback();
    void updateStatusCallback();
};