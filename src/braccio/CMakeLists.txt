cmake_minimum_required(VERSION 2.8.3)
project(braccio)

find_package(catkin REQUIRED COMPONENTS 
geometry_msgs 
std_srvs 
std_msgs 
roscpp 
ecl_exceptions 
ecl_threads 
ecl_time 
kobuki_msgs 
brac_msgs
serial
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES kobuki
   CATKIN_DEPENDS geometry_msgs std_srvs std_msgs roscpp ecl_exceptions ecl_threads ecl_time kobuki_msgs brac_msgs
   serial
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}
	src/main.cpp
)
target_link_libraries(braccio ${catkin_LIBRARIES} serial)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
