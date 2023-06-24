cmake_minimum_required(VERSION 2.8.3)

if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(dynamixel_msgs)

find_package(catkin REQUIRED
  COMPONENTS message_generation std_msgs)

add_message_files(
  FILES
  JointState.msg
  MotorState.msg
  MotorStateList.msg
  MovingStatus.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
