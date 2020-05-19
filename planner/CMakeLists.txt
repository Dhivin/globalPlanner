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
 
)



#add_executable(test2 src/planner_new.cpp)
add_executable(global_planner src/global_planner.cpp
			      src/sbpl_includes.cpp
            src/ros_class.cpp
            src/environment.cpp)


#target_link_libraries(test2 ${catkin_LIBRARIES} sbpl)
target_link_libraries(global_planner ${catkin_LIBRARIES} sbpl)

add_dependencies(global_planner ${PROJECT_NAME}_gencfg)
