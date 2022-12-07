cmake_minimum_required(VERSION 2.8.3)
project(generate_lane)

find_package(catkin REQUIRED
  COMPONENTS
  geometry_msgs
  grid_map_core
  grid_map_ros
  message_filters
  nav_msgs
  pcl_conversions
  pcl_ros
  tf
  )

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(Eigen3 QUIET)

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()


catkin_package(
  INCLUDE_DIRS include
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
 ${CUDA_INCLUDE_DIRS}
)

add_library(lane_helper_lib src/helper.cpp)
add_executable(aggregate src/aggregate.cpp)
add_executable(image src/lidar_image.cpp)
add_executable(init src/gps_init.cpp)

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(aggregate
  lane_helper_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  )

target_link_libraries(image
  lane_helper_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

target_link_libraries(init
  lane_helper_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
