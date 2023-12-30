cmake_minimum_required(VERSION 2.8.3)
project(offboard)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
add_definitions(-std=c++11 -g -O1 -ftree-vectorize -march=native)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
    tf
    std_msgs
    sensor_msgs
    geometry_msgs
    mavros_msgs
    visualization_msgs
    trajectory_msgs
    nav_msgs
    message_generation
)

find_package(Eigen3 REQUIRED)

 add_message_files(
    FILES
    PosVelAcc.msg
    DebugPid.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

add_definitions(${EIGEN_DEFINITIONS})

include_directories(
        ../../devel/include/offboard/
        ${PROJECT_SOURCE_DIR}/include
)


add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES})

# DEMO
add_executable(posctl_hover src/posctl_hover.cpp)
target_link_libraries(posctl_hover ${catkin_LIBRARIES})

add_executable(posctl_circle src/posctl_circle.cpp)
target_link_libraries(posctl_circle ${catkin_LIBRARIES})

add_executable(attctl_circle src/attctl_circle.cpp)
target_link_libraries(attctl_circle ${catkin_LIBRARIES})

add_executable(attctl_traj src/attctl_traj.cpp)
add_dependencies(attctl_traj  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(attctl_traj ${catkin_LIBRARIES} )

add_executable(attctl_perching src/attctl_perching.cpp)
target_link_libraries(attctl_perching ${catkin_LIBRARIES})
