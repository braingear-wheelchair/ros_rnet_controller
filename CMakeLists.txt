cmake_minimum_required(VERSION 2.8.3)
project(ros_rnet_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
			 geometry_msgs)

find_package(PkgConfig)
pkg_search_module(RNETSERIAL REQUIRED rnetserial)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

catkin_package(
  INCLUDE_DIRS 
	include
	${RNETSERIAL_INCLUDE_DIRS}
  LIBRARIES 
    ${RNETSERIAL_LIBRARIES}
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	geometry_msgs

  DEPENDS
	RNETSERIAL
)


###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} 
					${RNETSERIAL_INCLUDE_DIRS} 
					include)

add_library(${PROJECT_NAME}	
			src/RNetController.cpp
			)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
									  ${RNETSERIAL_LIBRARIES})

add_dependencies(${PROJECT_NAME} 
				 ${catkin_EXPORTED_TARGETS}
)

#################
## Executables ##
#################
add_executable(rnet_controller src/rnet_controller.cpp)
target_link_libraries(rnet_controller ${PROJECT_NAME})
