cmake_minimum_required(VERSION 2.8.3)
project(sonar_youbot)

find_package(catkin REQUIRED COMPONENTS 
youbot_driver
roscpp
rospy
std_msgs
)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
catkin_package()


add_definitions(-DYOUBOT_CONFIGURATIONS_DIR="${youbot_driver_CONFIG_DIR}")


add_executable(sonar_youbot
  src/sonar_youbot.cpp
)

target_link_libraries(sonar_youbot 
	${catkin_LIBRARIES}
)
OPTION(USE_SETCAP "Set permissions to access ethernet interface without sudo" ON)

SET(SUDO_COMMAND gksudo)
IF($ENV{USE_NORMAL_SUDO})
    SET(SUDO_COMMAND sudo)
ENDIF($ENV{USE_NORMAL_SUDO})

IF(USE_SETCAP)
add_custom_command(TARGET sonar_youbot POST_BUILD
    COMMAND ${SUDO_COMMAND} setcap cap_net_raw+ep $<TARGET_FILE:sonar_youbot>
)    
endif(USE_SETCAP)
