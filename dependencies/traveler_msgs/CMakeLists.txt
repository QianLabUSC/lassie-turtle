cmake_minimum_required(VERSION 3.8)
project(traveler_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# Add msgs/srvs
set(msg_files
  "msg/OdriveStatus.msg"
  "msg/TravelerConfig.msg"
  "msg/TravelerMode.msg"
  "msg/TravelerStatus.msg"

)
# Add msgs/srvs
set(odrive_msg_files
  
  "odrivemsg/SetInputPosition.msg"
)
set(odrive_srv_files
  "odrivesrv/SetInputPos.srv"
  "odrivesrv/SetInputVel.srv"
  "odrivesrv/SetInputTorque.srv"
  "odrivesrv/ClearErrors.srv"
  "odrivesrv/GetEncoderCount.srv"
  "odrivesrv/GetEncoderError.srv"
  "odrivesrv/GetEncoderEstimates.srv"
  "odrivesrv/GetIq.srv"
  "odrivesrv/GetMotorError.srv"
  "odrivesrv/GetTemperature.srv"
  "odrivesrv/GetVbusVoltage.srv"
  "odrivesrv/OdriveEstop.srv"
  "odrivesrv/ResetOdrive.srv"
  "odrivesrv/SetAbsolutePos.srv"
  "odrivesrv/SetAxisNodeId.srv"
  "odrivesrv/SetAxisRequestedState.srv"
  "odrivesrv/SetAxisStartupConfig.srv"
  "odrivesrv/SetControllerModes.srv"
  "odrivesrv/SetPosGain.srv"
  "odrivesrv/SetTrajAccelLimits.srv"
  "odrivesrv/SetTrajInertia.srv"
  "odrivesrv/SetTrajVelLimit.srv"
  "odrivesrv/SetVelGains.srv"
  "odrivesrv/SetVelLimit.srv"
  "odrivesrv/StartAnticogging.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${odrive_msg_files}
  # ${odrive_srv_files}
)

# Allows the use of smsgs/srvs defined in this package
# we don't need this because we put the srv and msg in a different package
#rosidl_get_typesupport_target(cpp_typesupport_target
#  ${PROJECT_NAME} "rosidl_typesupport_cpp"
#)

# rosidl_get_typesupport_target(${PROJECT_NAME}_interface
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
