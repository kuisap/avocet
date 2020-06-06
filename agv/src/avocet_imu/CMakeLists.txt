cmake_minimum_required(VERSION 2.8.3)
project(avocet_imu)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_subdirectory(../third_party/yaml-cpp third_party)

add_library(imu src/imu_capture.cpp)
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
target_link_libraries(imu ${WIRINGPI_LIBRARIES})
target_include_directories(imu PUBLIC include)

add_executable(${PROJECT_NAME}_node src/imu_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  imu
  yaml-cpp
)
