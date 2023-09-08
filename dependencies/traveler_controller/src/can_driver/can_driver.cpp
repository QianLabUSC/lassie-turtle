#include "can_driver/can_driver.hpp"

can_driver::can_driver() : Node("can_driver"),
                               socket_channel0_axis0_read_(odrive_can::Msg::MSG_ODRIVE_HEARTBEAT | odrive_can::AXIS::AXIS_0_ID,0, 0x7FF, 500),
                               socket_channel0_axis1_read_(odrive_can::Msg::MSG_ODRIVE_HEARTBEAT | odrive_can::AXIS::AXIS_1_ID,0, 0x7FF, 110000),
                               socket_channel1_axis0_read_(odrive_can::Msg::MSG_ODRIVE_HEARTBEAT | odrive_can::AXIS::AXIS_0_ID,1, 0x7FF, 500),
                               socket_channel1_axis1_read_(odrive_can::Msg::MSG_ODRIVE_HEARTBEAT | odrive_can::AXIS::AXIS_1_ID,1, 0x7FF, 110000),
                               socket_channel0_get_iq_(odrive_can::Msg::MSG_GET_IQ | odrive_can::AXIS::AXIS_0_ID,0, 0x7FF, 500),
                               socket_channel1_get_iq_(odrive_can::Msg::MSG_GET_IQ | odrive_can::AXIS::AXIS_0_ID,1, 0x7FF, 500),
                               socket_get_encoder_estimates_0(odrive_can::Msg::MSG_GET_ENCODER_ESTIMATES | odrive_can::AXIS::AXIS_0_ID,0, 0x7FF, 500),
                               socket_get_encoder_estimates_1(odrive_can::Msg::MSG_GET_ENCODER_ESTIMATES | odrive_can::AXIS::AXIS_0_ID,1, 0x7FF, 500),
                               socket_topic_set_position_0(odrive_can::Msg::MSG_SET_INPUT_POS | odrive_can::AXIS::AXIS_0_ID,0, 0x7FF, 500),
                               socket_topic_set_position_1(odrive_can::Msg::MSG_SET_INPUT_POS | odrive_can::AXIS::AXIS_0_ID,1, 0x7FF, 500)
{   
    count = 0;
    now = std::chrono::system_clock::now();
    timer_ = this->create_wall_timer(std::chrono::microseconds(1000), std::bind(&can_driver::updateChannel1StatusCallback, this));
    timer_ = this->create_wall_timer(std::chrono::microseconds(1000), std::bind(&can_driver::updateChannel2StatusCallback, this));
    
}

can_driver::~can_driver()
{

}

void can_driver::updateChannel1StatusCallback()
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
    
   //  publisher_0->publish(odrive_status_msg_0);
}   

void can_driver::updateChannel2StatusCallback(){
   //  count = count + 1;
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
   //  publisher_1->publish(odrive_status_msg_1);
   //  auto end = std::chrono::high_resolution_clock::now();
   //  std::chrono::duration<double> diff = end - now; // duration in seconds as a double
   //  float time_cucrr = static_cast<float>(diff.count());
   //  std::cout<< "time: " << time_cucrr << "count: " << count << std::endl;
}

void can_driver::get_motor_status(turtle& turtle_){
      updateChannel1StatusCallback();
      updateChannel2StatusCallback();
      turtle_.turtle_chassis.left_adduction = odrive_status_msg_0;
      turtle_.turtle_chassis.left_sweeping = odrive_status_msg_1;
      turtle_.turtle_chassis.right_adduction = odrive_status_msg_0;
      turtle_.turtle_chassis.right_sweeping = odrive_status_msg_1;
}

void can_driver::setControl(turtle& turtle_){
   setPosition(turtle_.turtle_control.left_adduction.set_input_position);
   setPosition(turtle_.turtle_control.left_sweeping.set_input_position);
   setPosition(turtle_.turtle_control.right_adduction.set_input_position);
   setPosition(turtle_.turtle_control.right_sweeping.set_input_position);
}

void can_driver::setPosition(traveler_msgs::msg::SetInputPosition msg)
{   
    // RCLCPP_INFO(this->get_logger(), "Do Encoder Response Received");
    // std::cout<<"hahahaha"<<std::endl;
    can_frame send_frame;
    send_frame.can_dlc = 8;
    std::memcpy(&send_frame.data[0], &msg.input_position, sizeof(msg.input_position));
    std::memcpy(&send_frame.data[4], &msg.vel_ff, sizeof(msg.vel_ff));
    std::memcpy(&send_frame.data[6], &msg.torque_ff, sizeof(msg.torque_ff));

    send_frame.can_id = odrive_can::Msg::MSG_SET_INPUT_POS | odrive_can::AXIS::AXIS_0_ID;
    if (msg.can_channel == 0)
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