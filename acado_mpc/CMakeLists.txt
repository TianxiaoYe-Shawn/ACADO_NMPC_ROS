cmake_minimum_required(VERSION 3.0.2)
project(acado_mpc)

add_compile_options(-std=c++11 -O3)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf
  tf2
  
  std_msgs
  visualization_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES acado_mpc
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf2 message_runtime
#  DEPENDS system_lib
)

include_directories(
  src/Eigen-3.3
  acado_mpc_export
  ${catkin_INCLUDE_DIRS}
)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(${EIGEN3_INCLUDE_DIR})



add_library(ACADO STATIC IMPORTED)

add_executable(mpc_node src/mpc_node.cpp)
set_target_properties(ACADO PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/acado_mpc_export/libacado_exported_rti.a")
target_link_libraries(mpc_node ${catkin_LIBRARIES} ACADO)

add_executable(trajectory_publisher src/trajectory_publisher.cpp)
target_link_libraries(trajectory_publisher ${catkin_LIBRARIES})
add_dependencies(trajectory_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(odom_publisher src/odom_publisher.cpp)
target_link_libraries(odom_publisher ${catkin_LIBRARIES})
add_dependencies(odom_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
