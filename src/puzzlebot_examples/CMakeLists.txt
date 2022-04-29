cmake_minimum_required(VERSION 2.8.3)
project(puzzlebot_examples)

add_compile_options(-std=c++11)
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs geometry_msgs sensor_msgs message_generation tf)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS  std_msgs nav_msgs geometry_msgs roscpp message_runtime)

include_directories(include 
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS})

add_library(PID_controller src/PIDController.cpp)
add_dependencies(PID_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(PID_controller ${catkin_LIBRARIES})

add_executable(square src/square.cpp)
add_dependencies(square ${catkin_EXPORTED_TARGETS})
target_link_libraries(square ${catkin_LIBRARIES} pid_controller)


