cmake_minimum_required(VERSION 2.8.9)
project(planner)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  
  roscpp
  rospy
  sbpl_ros
  roslib
  dynamic_reconfigure
  actionlib_msgs
  OpenCV
)

generate_dynamic_reconfigure_options(cfg/planner.cfg)
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(SBPL REQUIRED sbpl)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS	
    roscpp
    rospy
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
 
)



add_executable(global_planner src/global_planner.cpp
			      src/sbpl_includes.cpp
            src/ros_class.cpp
            src/environment.cpp)


target_link_libraries(global_planner ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} sbpl)

add_dependencies(global_planner ${PROJECT_NAME}_gencfg)
