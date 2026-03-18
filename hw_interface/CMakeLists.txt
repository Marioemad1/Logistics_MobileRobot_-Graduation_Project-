cmake_minimum_required(VERSION 3.8)
project(hw_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_library(LGPIO_LIBRARY lgpio)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
if(NOT LGPIO_LIBRARY)
  message(FATAL_ERROR "lgpio library not found! Install liblgpio-dev")
endif()

add_executable(ultrasonic_sensor src/ultrasonic_sensor.cpp)
ament_target_dependencies(ultrasonic_sensor rclcpp sensor_msgs )

target_link_libraries(ultrasonic_sensor ${LGPIO_LIBRARY})

include_directories(include)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/mpu6050_hardware_interface.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_lifecycle hardware_interface pluginlib )

pluginlib_export_plugin_description_file(hardware_interface mpu6050_hardware_interface.xml)

install(TARGETS 
#don't forget to add the nodes in the targets
ultrasonic_sensor
DESTINATION lib/${PROJECT_NAME}

)

install(
  DIRECTORY include
  DESTINATION include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
