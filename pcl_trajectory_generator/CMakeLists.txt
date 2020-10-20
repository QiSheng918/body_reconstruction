cmake_minimum_required(VERSION 3.0.2)
project(pcl_trajectory_generator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  geometry_msgs
  visualization_msgs
  tf
)

# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_trajectory_generator
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}  
)


add_executable(cloud_preprocess_node ${PROJECT_SOURCE_DIR}/src/cloud_preprocess.cpp)
target_link_libraries(cloud_preprocess_node
   ${PCL_LIBRARIES} 
   ${catkin_LIBRARIES}
)

add_executable(cloud_normal_calculate_node ${PROJECT_SOURCE_DIR}/src/cloud_normal_calculate.cpp)
target_link_libraries(cloud_normal_calculate_node
   ${PCL_LIBRARIES} 
   ${catkin_LIBRARIES}
)

add_executable(cloud_cmd_generator_node ${PROJECT_SOURCE_DIR}/src/cloud_cmd_generator.cpp)
target_link_libraries(cloud_cmd_generator_node
   ${PCL_LIBRARIES} 
   ${catkin_LIBRARIES}
)


add_executable(interpret_test_node src/interpret_test.cpp)
target_link_libraries(interpret_test_node
   ${PCL_LIBRARIES} 
   ${catkin_LIBRARIES}
   ${EIGEN_LIBRARIES}
)



