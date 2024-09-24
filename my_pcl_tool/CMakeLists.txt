cmake_minimum_required(VERSION 3.0.2)
project(my_pcl_tool)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_conversions
  sensor_msgs
)

find_package(PCL REQUIRED QUIET)


catkin_package(
#  INCLUDE_DIRS includ
#  LIBRARIES my_pcl_tool
#  CATKIN_DEPENDS roscpp rospy sensor_smgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(viewpcd src/viewpcd.cpp)
add_dependencies(viewpcd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(viewpcd ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

add_executable(viewbin src/viewbin.cpp)
add_dependencies(viewbin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(viewbin ${catkin_LIBRARIES}  ${PCL_LIBRARIES})
