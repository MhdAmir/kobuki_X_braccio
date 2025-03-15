cmake_minimum_required(VERSION 2.8.3)
project(kobuki_intelligent)

find_package(catkin REQUIRED COMPONENTS 
geometry_msgs 
std_srvs 
std_msgs 
roscpp 
ecl_exceptions 
ecl_threads 
ecl_time 
kobuki_msgs 
custom_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES kobuki
   CATKIN_DEPENDS geometry_msgs std_srvs std_msgs roscpp ecl_exceptions ecl_threads ecl_time kobuki_msgs custom_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}
	src/intelligent.cpp
	src/running.cpp
	src/main.cpp
	src/utility.cpp
)
target_link_libraries(kobuki_intelligent ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
