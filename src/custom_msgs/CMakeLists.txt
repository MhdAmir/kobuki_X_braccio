cmake_minimum_required(VERSION 3.5.0)
add_compile_options(-std=c++11)
project(custom_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  message_generation
)
add_message_files(
  FILES
  Intl.msg
  Comm.msg
  Object.msg
  Realsense.msg
  HumanPose.msg
)

# add_service_files(
#   FILES
#     getTarget.srv
#     setInit.srv
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)
catkin_package(CATKIN_DEPENDS 
  CATKIN_DEPENDS
  std_msgs
  sensor_msgs
  message_runtime
)