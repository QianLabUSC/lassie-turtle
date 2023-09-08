cmake_minimum_required(VERSION 3.5)
project(turtle_high_controller)

set(CMAKE_BUILD_TYPE Debug)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(traveler_msgs REQUIRED)
find_package(traveler_msgs REQUIRED)
#find_package(glog REQUIRED)
#find_package(Eigen3 REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    /usr/include/eigen3
    /usr/local/include/qpOASES
    /usr/local/include/
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)


add_executable( turtle_high_controller
    src/main.cpp

    src/controller/controller_monitor.cpp
    src/controller/pid_speed_controller.cpp
    src/controller/Inverse_kinematics.cpp
    # src/controller/trajectories_parser.cpp

    src/proxy/lowerproxy.cpp
    src/proxy/upperproxy.cpp

    src/can_driver/can_driver.cpp
    src/can_driver/socketcan_interface.cpp

    # src/utils/lowpassfilter.cpp
)
ament_target_dependencies(turtle_high_controller 
  rclcpp
  std_msgs
  control_msgs
  traveler_msgs
)


install(TARGETS
  turtle_high_controller
  # shearing_speed
  DESTINATION lib/${PROJECT_NAME})


## below compile the can node alone and it allows you to subscribe and 
## publish a message
add_executable(odrive_can_interface 
      src/can_driver/odrive_can.cpp
      # src/can_service.cpp
      src/can_driver/can_publisher.cpp
      src/can_driver/socketcan_interface.cpp
      src/can_driver/can_suber.cpp)
ament_target_dependencies(odrive_can_interface
      rclcpp
      traveler_msgs
      )

install(TARGETS odrive_can_interface
        DESTINATION lib/${PROJECT_NAME})

ament_package()
