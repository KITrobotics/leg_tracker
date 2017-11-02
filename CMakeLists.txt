cmake_minimum_required(VERSION 2.8.3)
project(cob_leg_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS 
#   actionlib 
#   cob_srvs 
#   control_msgs 
#   diagnostic_msgs 
#   libntcan 
#   libpcan 
#   message_generation 
  roslint 
  roscpp 
  sensor_msgs 
  std_msgs 
  std_srvs 
  geometry_msgs
  laser_geometry
  pcl_conversions 
  pcl_ros
  tf2 
  tf2_ros
  tf2_msgs
  tf2_sensor_msgs
#   trajectory_msgs 
#   urdf
)
find_package( OpenCV REQUIRED )

find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS std_msgs
)


include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
# include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/cob_leg_tracker.cpp)
# set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DOSNAME_LINUX -DWITH_ESD_CAN")
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${Boost_INCLUDE_DIRS} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} ${Boost_INCLUDE_DIRS} ${catkin_LIBRARIES})


### INSTALL ###
install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(FILES ${PROJECT_SOURCE_DIR}/common/lib/${MYARCH}/libSDHLibrary-CPP.so DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

### LINT ###
roslint_cpp(src/cob_leg_tracker.cpp)