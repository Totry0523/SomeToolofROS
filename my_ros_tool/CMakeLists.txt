cmake_minimum_required(VERSION 3.0.2)
project(my_ros_tool)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_conversions
  sensor_msgs
)

find_package(PCL REQUIRED QUIET)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_ros_tool
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(demo_imu_read src/demo_imu_read.cpp)
add_dependencies(demo_imu_read ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(demo_imu_read ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(KittiCleanGround_demo src/KittiCleanGround_demo.cpp)
add_dependencies(KittiCleanGround_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(KittiCleanGround_demo ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(KittiCleanGroundAndSave src/KittiCleanGroundAndSave.cpp)
add_dependencies(KittiCleanGroundAndSave ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(KittiCleanGroundAndSave ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(KittiMovingVisualize src/KittiMovingVisualize.cpp)
add_dependencies(KittiMovingVisualize ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(KittiMovingVisualize ${catkin_LIBRARIES} ${PCL_LIBRARIES})



