#include <linux/can/raw.h>
#include <linux/can.h>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "socketcan_interface.hpp"
#include "odrive_can.hpp"

#include "traveler_msgs/msg/odrive_status.hpp"

#include "traveler_msgs/srv/odrive_estop.hpp"
#include "traveler_msgs/srv/get_motor_error.hpp"
#include "traveler_msgs/srv/get_encoder_error.hpp"
#include "traveler_msgs/srv/set_axis_node_id.hpp"
#include "traveler_msgs/srv/set_axis_requested_state.hpp"
#include "traveler_msgs/srv/set_axis_startup_config.hpp"
#include "traveler_msgs/srv/get_encoder_estimates.hpp"
#include "traveler_msgs/srv/get_encoder_count.hpp"
#include "traveler_msgs/srv/set_controller_modes.hpp"
#include "traveler_msgs/srv/set_input_pos.hpp"
#include "traveler_msgs/srv/set_input_vel.hpp"
#include "traveler_msgs/srv/set_input_torque.hpp"
#include "traveler_msgs/srv/set_vel_limit.hpp"
#include "traveler_msgs/srv/start_anticogging.hpp"
#include "traveler_msgs/srv/set_traj_vel_limit.hpp"
#include "traveler_msgs/srv/set_traj_accel_limits.hpp"
#include "traveler_msgs/srv/set_traj_inertia.hpp"
#include "traveler_msgs/srv/get_iq.hpp"
#include "traveler_msgs/srv/reset_odrive.hpp"
#include "traveler_msgs/srv/get_vbus_voltage.hpp"
#include "traveler_msgs/srv/clear_errors.hpp"
#include "traveler_msgs/srv/get_temperature.hpp"
#include "traveler_msgs/srv/set_absolute_pos.hpp"
#include "traveler_msgs/srv/set_pos_gain.hpp"
#include "traveler_msgs/srv/set_vel_gains.hpp"



class CanService : public rclcpp::Node
{
public:
    CanService(/* args */);
    ~CanService();

private:
    rclcpp::Service<traveler_msgs::srv::OdriveEstop>::SharedPtr service_odrive_estop_;
    rclcpp::Service<traveler_msgs::srv::GetMotorError>::SharedPtr service_get_motor_error_;
    rclcpp::Service<traveler_msgs::srv::GetEncoderError>::SharedPtr service_get_encoder_error_;
    rclcpp::Service<traveler_msgs::srv::SetAxisNodeId>::SharedPtr service_set_axis_node_id_;
    rclcpp::Service<traveler_msgs::srv::SetAxisRequestedState>::SharedPtr service_set_axis_requested_state_;
    rclcpp::Service<traveler_msgs::srv::SetAxisStartupConfig>::SharedPtr service_set_axis_startup_config_;
    rclcpp::Service<traveler_msgs::srv::GetEncoderEstimates>::SharedPtr service_get_encoder_estimates_;
    rclcpp::Service<traveler_msgs::srv::GetEncoderCount>::SharedPtr service_get_encoder_count_;
    rclcpp::Service<traveler_msgs::srv::SetControllerModes>::SharedPtr service_set_controller_modes_;
    rclcpp::Service<traveler_msgs::srv::SetInputPos>::SharedPtr service_set_input_pos_;
    rclcpp::Service<traveler_msgs::srv::SetInputVel>::SharedPtr service_set_input_vel_;
    rclcpp::Service<traveler_msgs::srv::SetInputTorque>::SharedPtr service_set_input_torque_;
    rclcpp::Service<traveler_msgs::srv::SetVelLimit>::SharedPtr service_set_vel_limit_;
    rclcpp::Service<traveler_msgs::srv::StartAnticogging>::SharedPtr service_start_anticogging_;
    rclcpp::Service<traveler_msgs::srv::SetTrajVelLimit>::SharedPtr service_set_traj_vel_limit_;
    rclcpp::Service<traveler_msgs::srv::SetTrajAccelLimits>::SharedPtr service_set_traj_accel_limits_;
    rclcpp::Service<traveler_msgs::srv::SetTrajInertia>::SharedPtr service_set_traj_inertia_;
    rclcpp::Service<traveler_msgs::srv::GetIq>::SharedPtr service_get_iq_;
    rclcpp::Service<traveler_msgs::srv::GetTemperature>::SharedPtr service_get_temperature_;
    rclcpp::Service<traveler_msgs::srv::ResetOdrive>::SharedPtr service_reset_odrive_;
    rclcpp::Service<traveler_msgs::srv::GetVbusVoltage>::SharedPtr service_get_vbus_voltage_;
    rclcpp::Service<traveler_msgs::srv::ClearErrors>::SharedPtr service_clear_errors_;
    rclcpp::Service<traveler_msgs::srv::SetAbsolutePos>::SharedPtr service_set_absolute_pos_;
    rclcpp::Service<traveler_msgs::srv::SetPosGain>::SharedPtr service_set_pos_gain_;
    rclcpp::Service<traveler_msgs::srv::SetVelGains>::SharedPtr service_set_vel_gains_;



    // SocketcanInterface socket_get_motor_error_ = SocketcanInterface(odrive_can::Msg::MSG_GET_MOTOR_ERROR | odrive_can::AXIS::AXIS_0_ID);
    // SocketcanInterface socket_get_encoder_error_ = SocketcanInterface(odrive_can::Msg::MSG_GET_ENCODER_ERROR | odrive_can::AXIS::AXIS_0_ID);
    // SocketcanInterface socket_get_encoder_estimates_ = SocketcanInterface(odrive_can::Msg::MSG_GET_ENCODER_ESTIMATES | odrive_can::AXIS::AXIS_0_ID);
    // SocketcanInterface socket_get_encoder_count_ = SocketcanInterface(odrive_can::Msg::MSG_GET_ENCODER_COUNT | odrive_can::AXIS::AXIS_0_ID);
    // SocketcanInterface socket_get_iq_ = SocketcanInterface(odrive_can::Msg::MSG_GET_IQ | odrive_can::AXIS::AXIS_0_ID);
    // SocketcanInterface socket_get_temperature_ = SocketcanInterface(odrive_can::Msg::MSG_GET_TEMPERATURE | odrive_can::AXIS::AXIS_0_ID);
    // SocketcanInterface socket_get_vbus_voltage_ = SocketcanInterface(odrive_can::Msg::MSG_GET_VBUS_VOLTAGE | odrive_can::AXIS::AXIS_0_ID);
    // SocketcanInterface socket_generic_write_ = SocketcanInterface(odrive_can::Msg::MSG_CO_NMT_CTRL | odrive_can::AXIS::AXIS_0_ID);

    SocketcanInterface socket_get_motor_error_;
    SocketcanInterface socket_get_encoder_error_;
    SocketcanInterface socket_get_encoder_estimates_;
    SocketcanInterface socket_get_encoder_count_;
    SocketcanInterface socket_get_iq_;
    SocketcanInterface socket_get_temperature_;
    SocketcanInterface socket_get_vbus_voltage_;
    SocketcanInterface socket_set_position_channel0;
    SocketcanInterface socket_set_position_channel1;
    SocketcanInterface socket_generic_write_;
    


    void odrive_estop_callback(const std::shared_ptr<traveler_msgs::srv::OdriveEstop::Request> request, std::shared_ptr<traveler_msgs::srv::OdriveEstop::Response> response);
    void get_motor_error_callback(const std::shared_ptr<traveler_msgs::srv::GetMotorError::Request> request, std::shared_ptr<traveler_msgs::srv::GetMotorError::Response> response);
    void get_encoder_error_callback(const std::shared_ptr<traveler_msgs::srv::GetEncoderError::Request> request, std::shared_ptr<traveler_msgs::srv::GetEncoderError::Response> response);
    void set_axis_node_id_callback(const std::shared_ptr<traveler_msgs::srv::SetAxisNodeId::Request> request, std::shared_ptr<traveler_msgs::srv::SetAxisNodeId::Response> response);
    void set_axis_requested_state_callback(const std::shared_ptr<traveler_msgs::srv::SetAxisRequestedState::Request> request, std::shared_ptr<traveler_msgs::srv::SetAxisRequestedState::Response> response);
    void set_axis_startup_config_callback(const std::shared_ptr<traveler_msgs::srv::SetAxisStartupConfig::Request> request, std::shared_ptr<traveler_msgs::srv::SetAxisStartupConfig::Response> response);
    void get_encoder_estimates_callback(const std::shared_ptr<traveler_msgs::srv::GetEncoderEstimates::Request> request, std::shared_ptr<traveler_msgs::srv::GetEncoderEstimates::Response> response);
    void get_encoder_count_callback(const std::shared_ptr<traveler_msgs::srv::GetEncoderCount::Request> request, std::shared_ptr<traveler_msgs::srv::GetEncoderCount::Response> response);
    void set_controller_modes_callback(const std::shared_ptr<traveler_msgs::srv::SetControllerModes::Request> request, std::shared_ptr<traveler_msgs::srv::SetControllerModes::Response> response);
    void set_input_pos_callback(const std::shared_ptr<traveler_msgs::srv::SetInputPos::Request> request, std::shared_ptr<traveler_msgs::srv::SetInputPos::Response> response);
    void set_input_vel_callback(const std::shared_ptr<traveler_msgs::srv::SetInputVel::Request> request, std::shared_ptr<traveler_msgs::srv::SetInputVel::Response> response);
    void set_input_torque_callback(const std::shared_ptr<traveler_msgs::srv::SetInputTorque::Request> request, std::shared_ptr<traveler_msgs::srv::SetInputTorque::Response> response);
    void set_vel_limit_callback(const std::shared_ptr<traveler_msgs::srv::SetVelLimit::Request> request, std::shared_ptr<traveler_msgs::srv::SetVelLimit::Response> response);
    void start_anticogging_callback(const std::shared_ptr<traveler_msgs::srv::StartAnticogging::Request> request, std::shared_ptr<traveler_msgs::srv::StartAnticogging::Response> response);
    void set_traj_vel_limit_callback(const std::shared_ptr<traveler_msgs::srv::SetTrajVelLimit::Request> request, std::shared_ptr<traveler_msgs::srv::SetTrajVelLimit::Response> response);
    void set_traj_accel_limits_callback(const std::shared_ptr<traveler_msgs::srv::SetTrajAccelLimits::Request> request, std::shared_ptr<traveler_msgs::srv::SetTrajAccelLimits::Response> response);
    void set_traj_inertia_callback(const std::shared_ptr<traveler_msgs::srv::SetTrajInertia::Request> request, std::shared_ptr<traveler_msgs::srv::SetTrajInertia::Response> response);
    void get_iq_callback(const std::shared_ptr<traveler_msgs::srv::GetIq::Request> request, std::shared_ptr<traveler_msgs::srv::GetIq::Response> response);
    void get_temperature_callback(const std::shared_ptr<traveler_msgs::srv::GetTemperature::Request> request, std::shared_ptr<traveler_msgs::srv::GetTemperature::Response> response);
    void reset_odrive_callback(const std::shared_ptr<traveler_msgs::srv::ResetOdrive::Request> request, std::shared_ptr<traveler_msgs::srv::ResetOdrive::Response> response);
    void get_vbus_voltage_callback(const std::shared_ptr<traveler_msgs::srv::GetVbusVoltage::Request> request, std::shared_ptr<traveler_msgs::srv::GetVbusVoltage::Response> response);
    void clear_errors_callback(const std::shared_ptr<traveler_msgs::srv::ClearErrors::Request> request, std::shared_ptr<traveler_msgs::srv::ClearErrors::Response> response);
    void set_absolute_pos_callback(const std::shared_ptr<traveler_msgs::srv::SetAbsolutePos::Request> request, std::shared_ptr<traveler_msgs::srv::SetAbsolutePos::Response> response);
    void set_pos_gain_callback(const std::shared_ptr<traveler_msgs::srv::SetPosGain::Request> request, std::shared_ptr<traveler_msgs::srv::SetPosGain::Response> response);
    void set_vel_gains_callback(const std::shared_ptr<traveler_msgs::srv::SetVelGains::Request> request, std::shared_ptr<traveler_msgs::srv::SetVelGains::Response> response);
};