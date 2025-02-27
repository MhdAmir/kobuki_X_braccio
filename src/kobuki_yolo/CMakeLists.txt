cmake_minimum_required(VERSION 2.8.3)
project(kobuki_yolo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs 
  std_srvs 
  std_msgs 
  roscpp 
  ecl_exceptions 
  ecl_threads 
  ecl_time 
  kobuki_msgs
  cv_bridge          
  image_transport    
  message_filters    
)

find_package(OpenCV 4.8.1 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kobuki
  CATKIN_DEPENDS geometry_msgs std_srvs std_msgs roscpp ecl_exceptions ecl_threads ecl_time kobuki_msgs
                 cv_bridge image_transport message_filters  
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  /opt/intel/openvino/runtime/include
  /opt/intel/openvino/runtime/include/ie
  /opt/intel/openvino/runtime/include/ngraph
)

add_executable(${PROJECT_NAME}
  src/main.cc
  # src/inference.cpp
  # src/object_detection.cpp
  src/subscribe_frame.cpp
  # src/object_detection.cpp
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
  yaml-cpp 
  realsense2
  /opt/intel/openvino/runtime/lib/intel64/libopenvino.so
)