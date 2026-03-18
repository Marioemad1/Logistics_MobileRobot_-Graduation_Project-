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
if(NOT LGPIO_LIBRARY)
  message(FATAL_ERROR "lgpio library not found! Install liblgpio-dev")
endif()

add_executable(ultrasonic_sensor src/ultrasonic_sensor.cpp)
ament_target_dependencies(ultrasonic_sensor rclcpp sensor_msgs )

target_link_libraries(ultrasonic_sensor ${LGPIO_LIBRARY})

install(TARGETS 
#don't forget to add the nodes in the targets
ultrasonic_sensor
DESTINATION lib/${PROJECT_NAME}

)

ament_package()
