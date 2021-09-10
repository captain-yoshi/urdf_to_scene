cmake_minimum_required(VERSION 3.1.3)
project(urdf_to_scene)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	eigen_conversions
	geometric_shapes
	geometry_msgs
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_visual_tools
	roscpp
	urdf
	visualization_msgs
)

catkin_package(
	LIBRARIES
		${PROJECT_NAME}
	INCLUDE_DIRS
		include
	CATKIN_DEPENDS
		eigen_conversions
		geometric_shapes
		geometry_msgs
		moveit_core
		moveit_ros_planning
		moveit_ros_planning_interface
		moveit_visual_tools
		roscpp
		urdf
		visualization_msgs
)

set(CMAKE_CXX_STANDARD 14)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/scene_parser.cpp)

add_executable(load_urdf src/load_urdf.cpp)
target_link_libraries(load_urdf ${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS load_urdf ${PROJECT_NAME}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
