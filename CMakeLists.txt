cmake_minimum_required(VERSION 2.4.6)
project(grasp_utilities)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
find_package(catkin REQUIRED
  roscpp rospy
  std_msgs message_generation
  tf tf2 tf2_ros
 pluginlib cmake_modules
  pr2_controllers_msgs control_toolbox
  moveit_core moveit_ros_planning moveit_ros_planning_interface
  dynamic_reconfigure
  )

find_package(Boost REQUIRED system filesystem date_time thread)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/grasp_reconfiguration.cfg
  #...
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

## Generate messages in the 'msg' folder
add_message_files(
   FILES
    jacobian_matrix.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   getFingerJacobianMatrix.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pr2_moveit_tutorials
  CATKIN_DEPENDS 
    tf2 tf2_ros
    std_msgs message_runtime
    pr2_controller_manager  pr2_mechanism_msgs
    moveit_core moveit_ros_planning_interface
)
find_package(Eigen REQUIRED)


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(object_marker src/object_visualization.cpp)
add_executable(get_grasp_map src/get_grasp_map.cpp)
add_executable(sensor_tf_broadcaster src/sensor_tf_broadcaster.cpp)
add_executable(finger_jacobian_service src/finger_jacobian_service.cpp)
add_executable(get_young_modulus src/get_young_modulus.cpp)
add_executable(grasp_reconfiguration src/grasp_reconfiguration.cpp)


target_link_libraries(object_marker ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(get_grasp_map ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(sensor_tf_broadcaster ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(finger_jacobian_service ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(get_young_modulus ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(grasp_reconfiguration ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(TARGETS object_marker DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS get_grasp_map DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS sensor_tf_broadcaster DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS finger_jacobian_service DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS get_young_modulus DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS grasp_reconfiguration DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})







#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
