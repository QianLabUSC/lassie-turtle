#include "odrivepro_can_includes/can_publisher.hpp"

CanPublisher::CanPublisher() : Node("can_publisher"),
                               socket_channel0_axis0_read_(odrive_can::Msg::MSG_ODRIVE_HEARTBEAT | odrive_can::AXIS::AXIS_0_ID,0, 0x7FF, 500),
                               socket_channel0_axis1_read_(odrive_can::Msg::MSG_ODRIVE_HEARTBEAT | odrive_can::AXIS::AXIS_1_ID,0, 0x7FF, 110000),
                               socket_channel1_axis0_read_(odrive_can::Msg::MSG_ODRIVE_HEARTBEAT | odrive_can::AXIS::AXIS_0_ID,1, 0x7FF, 500),
                               socket_channel1_axis1_read_(odrive_can::Msg::MSG_ODRIVE_HEARTBEAT | odrive_can::AXIS::AXIS_1_ID,1, 0x7FF, 110000),
                               socket_channel0_get_iq_(odrive_can::Msg::MSG_GET_IQ | odrive_can::AXIS::AXIS_0_ID,0, 0x7FF, 500),
                               socket_channel1_get_iq_(odrive_can::Msg::MSG_GET_IQ | odrive_can::AXIS::AXIS_0_ID,1, 0x7FF, 500),
                               socket_get_encoder_estimates_0(odrive_can::Msg::MSG_GET_ENCODER_ESTIMATES | odrive_can::AXIS::AXIS_0_ID,0, 0x7FF, 500),
                               socket_get_encoder_estimates_1(odrive_can::Msg::MSG_GET_ENCODER_ESTIMATES | odrive_can::AXIS::AXIS_0_ID,1, 0x7FF, 500)
                               //socket_topic_set_position_0(odrive_can::Msg::MSG_SET_INPUT_POS | odrive_can::AXIS::AXIS_0_ID,0, 0x7FF, 500),
                               //socket_topic_set_position_1(odrive_can::Msg::MSG_SET_INPUT_POS | odrive_can::AXIS::AXIS_0_ID,1, 0x7FF, 500)
                               // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
{   
    count = 0;
    now = std::chrono::system_clock::now();
    publisher_0= this->create_publisher<odrive_pro_srvs_msgs::msg::OdriveStatus>("/odrive/odrive_status", 100);
    publisher_1= this->create_publisher<odrive_pro_srvs_msgs::msg::OdriveStatus>("/odrive/odrive_status", 100);

    //Subscription_channel_0 = this->create_subscription<odrive_pro_srvs_msgs::msg::SetInputPosition>("/odrivepos0", 10, std::bind(&CanPublisher::setPositionCallback, this, std::placeholders::_1));
    //Subscription_channel_1 = this->create_subscription<odrive_pro_srvs_msgs::msg::SetInputPosition>("/odrivepos1", 10, std::bind(&CanPublisher::setPositionCallback, this, std::placeholders::_1));
    //RCLCPP_INFO(this->get_logger(), "Do Encoder Response Received11");
    // odrive_status_msg_1.axis_error = 0;
    // odrive_status_msg_0.axis_error = 0;
    // odrive_status_msg_1.axis_state = 0;
    // odrive_status_msg_0.axis_state = 0;
    //timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CanPublisher::timerCallback, this));
    timer_ = this->create_wall_timer(std::chrono::microseconds(1000), std::bind(&CanPublisher::updateChannel1StatusCallback, this));
    timer_ = this->create_wall_timer(std::chrono::microseconds(1000), std::bind(&CanPublisher::updateChannel2StatusCallback, this));
    
}

CanPublisher::~CanPublisher()
{

}

// void CanPublisher::timerCallback()
// {
    
    
//     // odrive_status_msg.axis = 1;
//     // can_frame axis1_frame;
//     // int axis1_read = socket_axis1_read_.readFrame(&axis1_frame);
//     // if (axis1_read < 0)
//     // {
//     //    RCLCPP_INFO(this->get_logger(), "No Axis 1 Heartbeat message");
//     //    odrive_status_msg.is_axis_alive = false;
//     // }
//     // else
//     // {
//     //    odrive_status_msg.is_axis_alive = true;
//     //     RCLCPP_DEBUG(this->get_logger(), "%x %x %x %x %x %x %x %x", axis1_frame.data[0], axis1_frame.data[1], axis1_frame.data[2], axis1_frame.data[3], axis1_frame.data[4], axis1_frame.data[5], axis1_frame.data[6], axis1_frame.data[7]);
//     //     odrive_status_msg.axis_error = odrive_can::can_getSignal<uint32_t>(axis1_frame, 0, 32, true);
//     //     odrive_status_msg.axis_state = odrive_can::can_getSignal<uint32_t>(axis1_frame, 32, 32, true);
//     // }
//     publisher_0->publish(odrive_status_msg_0);
//     publisher_1->publish(odrive_status_msg_1);
// }


void CanPublisher::updateChannel1StatusCallback()
{   
    odrive_status_msg_0.can_channel = 0;
    odrive_status_msg_0.axis = 0;

    

    can_frame recv_frame_0;
    if (socket_get_encoder_estimates_0.readFrame(&recv_frame_0) < 0) 
     {
        // RCLCPP_INFO(this->get_logger(), "No Encoder Response Received");
     }
      else{
        odrive_status_msg_0.pos_estimate = odrive_can::can_getSignal<float>(recv_frame_0, 0, 32, true);
        odrive_status_msg_0.vel_estimate = odrive_can::can_getSignal<float>(recv_frame_0, 32, 32, true);
    }
    
    can_frame iq_recv_frame_0;
    if (socket_channel0_get_iq_.readFrame(&iq_recv_frame_0) < 0)
     {
        //  RCLCPP_INFO(this->get_logger(), "No Current Response Received");
     }
     else{
        odrive_status_msg_0.iq_setpoint = odrive_can::can_getSignal<float>(iq_recv_frame_0, 0, 32, true);
        odrive_status_msg_0.iq_measured = odrive_can::can_getSignal<float>(iq_recv_frame_0, 32, 32, true);
     }
    
    publisher_0->publish(odrive_status_msg_0);
}   

void CanPublisher::updateChannel2StatusCallback(){
    count = count + 1;
    odrive_status_msg_1.axis = 0;
    odrive_status_msg_1.can_channel = 1;

    // the same reason for channel 1

    // can_frame axis0_frame_channel1;
    
    
    can_frame recv_frame_1;
    if (socket_get_encoder_estimates_1.readFrame(&recv_frame_1) < 0) 
     {
        // RCLCPP_INFO(this->get_logger(), "No Encoder Response Received");
     }
     else{
        odrive_status_msg_1.pos_estimate = odrive_can::can_getSignal<float>(recv_frame_1, 0, 32, true);
        odrive_status_msg_1.vel_estimate = odrive_can::can_getSignal<float>(recv_frame_1, 32, 32, true);
    
     }
 
    can_frame iq_recv_frame_1;
    if (socket_channel1_get_iq_.readFrame(&iq_recv_frame_1) < 0)
     {
        //  RCLCPP_INFO(this->get_logger(), "No Current Response Received");
     }
     else{
        odrive_status_msg_1.iq_setpoint = odrive_can::can_getSignal<float>(iq_recv_frame_1, 0, 32, true);
        odrive_status_msg_1.iq_measured = odrive_can::can_getSignal<float>(iq_recv_frame_1, 32, 32, true);
 
     }
    publisher_1->publish(odrive_status_msg_1);
    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> diff = end - now; // duration in seconds as a double
    // float time_cucrr = static_cast<float>(diff.count());
    // std::cout<< "time: " << time_cucrr << "count: " << count << std::endl;
}

// void CanPublisher::setPositionCallback(const odrive_pro_srvs_msgs::msg::SetInputPosition::SharedPtr msg)
// {   
//     RCLCPP_INFO(this->get_logger(), "Do Encoder Response Received");
//     std::cout<<"hahahaha"<<std::endl;
//     can_frame send_frame;
//     send_frame.can_dlc = 8;
//     std::memcpy(&send_frame.data[0], &msg->input_position, sizeof(msg->input_position));
//     std::memcpy(&send_frame.data[4], &msg->vel_ff, sizeof(msg->vel_ff));
//     std::memcpy(&send_frame.data[6], &msg->torque_ff, sizeof(msg->torque_ff));

//     send_frame.can_id = odrive_can::Msg::MSG_SET_INPUT_POS | odrive_can::AXIS::AXIS_0_ID;
//     ;
//     if (msg->can_channel == 0)
//     {
//         socket_topic_set_position_0.writeFrame(send_frame);
//     }
//     else
//     {
//         socket_topic_set_position_1.writeFrame(send_frame);
//         return;
//     }
//     // socket_generic_write_.writeFrame(send_frame);
    
// }