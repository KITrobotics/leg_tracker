cmake_minimum_required(VERSION 2.8.3)
project(leg_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roslint
  roscpp
  roslib
  rosconsole
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  laser_geometry
  nav_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
  tf2_geometry_msgs
  visualization_msgs
  iirob_filters
  message_generation
)

# find_package(Eigen3 REQUIRED)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
else()
  set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
  set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES}) 
endif()

# find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED)

add_message_files(DIRECTORY msg FILES
  LegTrackerMessage.msg # deprecated
  LegMsg.msg
  LegMsgStamped.msg
  PersonMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs tf2_geometry_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

add_executable(${PROJECT_NAME} src/munkres.cpp src/leg_tracker.cpp src/leg_tracker_node.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  ${PROJECT_NAME}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

### INSTALL ###
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### LINT ###
roslint_cpp(src/matrix.cpp src/munkres.cpp src/leg_tracker.cpp src/leg_tracker_node.cpp)
