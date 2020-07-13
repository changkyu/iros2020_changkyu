cmake_minimum_required(VERSION 2.8.3)
project(test_pkg)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -fPIC")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  common_utilities
  iiwa_msgs
  iiwa_ros
  task_planning
  motion_planning
  roscpp
  moveit_core 
  moveit_ros_planning
  moveit_ros_planning_interface
  # chimp_pose_estimation
  grasping
  std_msgs
  tf
  message_generation
)
find_package (PCL REQUIRED COMPONENTS io common visualization features) 

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   iiwa_msgs
# )
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 # LIBRARIES test_pkg
 CATKIN_DEPENDS common_utilities task_planning roscpp cv_bridge
 # DEPENDS system_lib
)
find_package(OpenCV REQUIRED)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(test_pkg
#   src/${PROJECT_NAME}/test_pkg.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
add_executable(test_node src/TestScript.cpp)

add_executable(demo_node src/JDXDemo.cpp)
#add_executable(perception_test src/perception_test.cpp)

add_executable(fake_joint_publisher src/fake_joint_state_publisher.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(test_pkg_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_dependencies(test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# ## Specify libraries to link a library or executable target against
target_link_libraries(test_node
  ${catkin_LIBRARIES}
)


message( WARNING "Catkin Libraries for TestPkg: " ${catkin_LIBRARIES} )


add_dependencies(demo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} test_pkg_generate_message_cpp)
target_link_libraries(demo_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#add_dependencies(perception_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(perception_test
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#)

target_link_libraries(fake_joint_publisher 
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(fake_joint_publisher  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} test_pkg_generate_message_cpp)
