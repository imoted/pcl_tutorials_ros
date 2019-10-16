cmake_minimum_required(VERSION 2.8.3)
project(pcl_tutorials_ros)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
  roscpp
  sensor_msgs
  tf2_ros
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(filtering src/filtering.cpp)
add_executable(segmentation src/segmentation.cpp)

target_link_libraries(filtering
  ${catkin_LIBRARIES}
)
target_link_libraries(segmentation
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/test_example.test ARGS example:=filtering number:=0 use_file:=true file:=table_scene_lms400.pcd)
endif()
