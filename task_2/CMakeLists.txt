cmake_minimum_required(VERSION 2.8.3)
project(task_2)


add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation 
  gazebo_msgs 
  roscpp 
  rospy  
  std_srvs 
  geometry_msgs 
  sensor_msgs 
  nav_msgs
  std_msgs
  gazebo_ros
)

find_package(catkin REQUIRED)
find_package(roslaunch)

##### INCLUDE #####
include_directories( include ${catkin_INCLUDE_DIRS})


find_package(gazebo REQUIRED)
include_directories( include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
	DEPENDS 
	    roscpp 
	    gazebo_ros
      std_msgs 
      geometry_msgs 
      message_runtime
      dynamic_reconfigure
)


### BUILD ###

add_library(custom_force src/custom_force.cpp)
add_dependencies(custom_force task_2_custom_force_cpp)
target_link_libraries (custom_force ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

### INSTALL ###

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  custom_force
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)