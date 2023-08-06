#include "can_driver/can_suber.hpp"
CanSuber::CanSuber() : Node("can_suber"),
                               socket_topic_set_position_0(odrive_can::Msg::MSG_SET_INPUT_POS | odrive_can::AXIS::AXIS_0_ID,0, 0x7FF, 500),
                               socket_topic_set_position_1(odrive_can::Msg::MSG_SET_INPUT_POS | odrive_can::AXIS::AXIS_0_ID,1, 0x7FF, 500)
                        
{   
    
    Subscription_channel_0 = this->create_subscription<traveler_msgs::msg::SetInputPosition>("/odrivepos0", 10, std::bind(&CanSuber::setPositionCallback, this, std::placeholders::_1));
    Subscription_channel_1 = this->create_subscription<traveler_msgs::msg::SetInputPosition>("/odrivepos1", 10, std::bind(&CanSuber::setPositionCallback, this, std::placeholders::_1));
    // RCLCPP_INFO(this->get_logger(), "Do Encoder Response Received11");
    
}

CanSuber::~CanSuber()
{
}

void CanSuber::setPositionCallback(const traveler_msgs::msg::SetInputPosition::SharedPtr msg)
{   
    // RCLCPP_INFO(this->get_logger(), "Do Encoder Response Received");
    // std::cout<<"hahahaha"<<std::endl;
    can_frame send_frame;
    send_frame.can_dlc = 8;
    std::memcpy(&send_frame.data[0], &msg->input_position, sizeof(msg->input_position));
    std::memcpy(&send_frame.data[4], &msg->vel_ff, sizeof(msg->vel_ff));
    std::memcpy(&send_frame.data[6], &msg->torque_ff, sizeof(msg->torque_ff));

    send_frame.can_id = odrive_can::Msg::MSG_SET_INPUT_POS | odrive_can::AXIS::AXIS_0_ID;
    ;
    if (msg->can_channel == 0)
    {
        socket_topic_set_position_0.writeFrame(send_frame);
    }
    else
    {
        socket_topic_set_position_1.writeFrame(send_frame);
        return;
    }
    // socket_generic_write_.writeFrame(send_frame);
    
}
