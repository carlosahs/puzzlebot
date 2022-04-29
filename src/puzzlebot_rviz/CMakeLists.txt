cmake_minimum_required(VERSION 2.8.3)
project(puzzlebot_rviz)

add_compile_options(-std=c++11)
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs geometry_msgs sensor_msgs message_generation tf)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS  std_msgs nav_msgs geometry_msgs sensor_msgs roscpp message_runtime)

include_directories(include 
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS})

add_library(pid_controller src/PIDController.cpp)
add_dependencies(pid_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(pid_controller ${catkin_LIBRARIES})

add_executable(puzzlebot_sim src/puzzlebot_sim_node.cpp src/PuzzleBotSim.cpp)
add_dependencies(puzzlebot_sim ${catkin_EXPORTED_TARGETS})
target_link_libraries(puzzlebot_sim ${catkin_LIBRARIES} pid_controller)

add_executable(motor src/motor_node.cpp src/Motor.cpp)
add_dependencies(motor ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor ${catkin_LIBRARIES})

add_executable(puzzlebot src/puzzlebot_node.cpp src/PuzzleBot.cpp)
add_dependencies(puzzlebot ${catkin_EXPORTED_TARGETS})
target_link_libraries(puzzlebot ${catkin_LIBRARIES})

add_executable(test_motor_constants src/test_motor_constants.cpp)
add_dependencies(test_motor_constants ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_motor_constants ${catkin_LIBRARIES})