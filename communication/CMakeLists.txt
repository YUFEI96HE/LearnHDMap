cmake_minimum_required(VERSION 3.0.2)
project(communication)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# Configure
SET (PROJECT_ROOT "${PROJECT_SOURCE_DIR}")
SET (PROJECT_SOURCE_DIR "${PROJECT_ROOT}/src")
SET (PROJECT_INCLUDE_DIR "${PROJECT_ROOT}/include")
# SET (PROJECT_BINARY_DIR "${PROJECT_ROOT}/bin")
# SET (PROJECT_LIB_DIR "${PROJECT_ROOT}/lib/arm64")

MESSAGE ("PROJECT_ROOT: ${PROJECT_ROOT}")
MESSAGE ("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
MESSAGE ("PROJECT_INCLUDE_DIR: ${PROJECT_INCLUDE_DIR}")
# MESSAGE ("PROJECT_BINARY_DIR: ${PROJECT_BINARY_DIR}")
# MESSAGE ("PROJECT_LIB_DIR: ${PROJECT_LIB_DIR}")

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	nav_msgs
	roscpp
	rospy
	sensor_msgs
	geometry_msgs
	std_msgs
	message_generation
	adam_msgs
	roslib
)

# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

# add_message_files(
# 	DIRECTORY msg
# 	#FILES GaoPeiNan.msg
# )

generate_messages(
	DEPENDENCIES std_msgs
	  			 geometry_msgs
	  			 adam_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES communication
	CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs std_msgs adam_msgs roslib
	DEPENDS 
)


###########
## Build ##
###########

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS}
					${PROJECT_INCLUDE_DIR}/MessageProtocol/
					${PROJECT_INCLUDE_DIR}/TCP/
					${PROJECT_INCLUDE_DIR}/Thread/
					${PROJECT_INCLUDE_DIR}/Utils/
					)

add_executable(${PROJECT_NAME}_node   	${PROJECT_SOURCE_DIR}/TCPClient.cpp
										${PROJECT_SOURCE_DIR}/base.cpp
										${PROJECT_SOURCE_DIR}/Utils.cpp
										${PROJECT_SOURCE_DIR}/HandleAPP.cpp
										${PROJECT_SOURCE_DIR}/HandleCCU.cpp
										${PROJECT_SOURCE_DIR}/HandleADT.cpp
										${PROJECT_SOURCE_DIR}/MessageCenter.cpp
										${PROJECT_ROOT}/communication_node.cpp)									

#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})										
target_link_libraries(${PROJECT_NAME}_node	${catkin_LIBRARIES})
