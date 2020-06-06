cmake_minimum_required(VERSION 2.8.3)
project(avocet_agv)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  compressed_image_transport
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(../third_party/yaml-cpp third_party)

add_executable(${PROJECT_NAME}_node src/image_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

add_library(imu src/imu_capture.cpp)
find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
target_link_libraries(imu ${WIRINGPI_LIBRARIES})
target_include_directories(imu PUBLIC include)

add_executable(avocet_imu_node src/imu_node.cpp)
add_dependencies(avocet_imu_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(avocet_imu_node
  ${catkin_LIBRARIES}
  imu
  yaml-cpp
)
