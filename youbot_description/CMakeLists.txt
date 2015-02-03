cmake_minimum_required(VERSION 2.8.3)
project(youbot_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  robot_state_publisher
  urdf
  trajectory_msgs
  brics_actuator
  nav_msgs
  pr2_controller_interface
  control_toolbox
  pr2_controllers_msgs
  realtime_tools
  orocos_kdl
  sensor_msgs
  control_msgs
  actionlib
  youbot_trajectory_action_server
  pr2_gazebo_plugins
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp robot_state_publisher urdf trajectory_msgs brics_actuator nav_msgs pr2_controller_interface control_toolbox pr2_controllers_msgs realtime_tools sensor_msgs control_msgs actionlib youbot_trajectory_action_server pr2_gazebo_plugins
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

#youbot base teleoperation experiment 
add_executable(youbot_keyboard_teleop_experiment examples/keyboard_teleop_experiment.cpp)
target_link_libraries(youbot_keyboard_teleop_experiment
  ${catkin_LIBRARIES}
)


add_executable(arm_joint_position_control_test examples/arm_joint_position_control_test.cpp)
target_link_libraries(arm_joint_position_control_test
  ${catkin_LIBRARIES}
)

add_executable(arm_joint_velocity_control_test examples/arm_joint_velocity_control_test.cpp)
target_link_libraries(arm_joint_velocity_control_test
  ${catkin_LIBRARIES}
)

add_executable(arm_joint_torque_control_test examples/arm_joint_torque_control_test.cpp)
target_link_libraries(arm_joint_torque_control_test
  ${catkin_LIBRARIES}
)

add_executable(static_joint_publisher src/static_joint_publisher.cpp)
target_link_libraries(static_joint_publisher
  ${catkin_LIBRARIES}
)

add_executable(base_command_republisher src/base_command_republisher.cpp)
target_link_libraries(base_command_republisher
  ${catkin_LIBRARIES}
)



#test for a joint position control of the youbot arm
add_library(joint_position_control src/joint_position_control.cpp)
target_link_libraries(joint_position_control
  ${catkin_LIBRARIES}
)

#youbot universal controller: joint space position control, joint velocity control, joint torque control
add_library(youbot_universal_control src/youbot_universal_control.cpp src/joint_state_observer_gazebo.cpp)

target_link_libraries(youbot_universal_control
  youbot_trajectory_action_server
  ${catkin_LIBRARIES}
  )
