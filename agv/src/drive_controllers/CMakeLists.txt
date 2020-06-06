cmake_minimum_required(VERSION 2.8.3)
project(drive_controllers)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(motor_driver src/driver_controllers.cpp)
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
target_link_libraries(motor_driver ${WIRINGPI_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/driver_controllers_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} motor_driver yaml-cpp)
