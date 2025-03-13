cmake_minimum_required(VERSION 3.0.2)
project(kobuki_communication)

find_package(catkin REQUIRED COMPONENTS
  rospy
  custom_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/main.py  # Sesuaikan dengan nama file Python-mu
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)