#include <linux/can/raw.h>
#include <linux/can.h>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "socketcan_interface.hpp"
#include "odrive_can.hpp"

#include "odrive_pro_srvs_msgs/msg/odrive_status.hpp"
#include "odrive_pro_srvs_msgs/msg/set_input_position.hpp"

class CanSuber : public rclcpp::Node
{
public:
    
    CanSuber(/* args */);
    ~CanSuber();

private:
    
    SocketcanInterface socket_topic_set_position_0;
    SocketcanInterface socket_topic_set_position_1;
    
    void setPositionCallback(odrive_pro_srvs_msgs::msg::SetInputPosition::SharedPtr msg);
    rclcpp::Subscription<odrive_pro_srvs_msgs::msg::SetInputPosition>::SharedPtr Subscription_channel_0;
    rclcpp::Subscription<odrive_pro_srvs_msgs::msg::SetInputPosition>::SharedPtr Subscription_channel_1;  
};
