cmake_minimum_required(VERSION 3.8)
project(rasd_ransac_speedbump)

# ---------------------------------------------------------
# Compiler settings
# ---------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# ---------------------------------------------------------
# Dependencies
# ---------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(pcl_conversions REQUIRED)

# ---------------------------------------------------------
# Include dirs
# ---------------------------------------------------------
include_directories(
  include                          # plane_detection.h
  ${PCL_INCLUDE_DIRS}              # PCL headers
  ${EIGEN3_INCLUDE_DIRS}           # Eigen headers (correct name)
)

add_definitions(${PCL_DEFINITIONS})

# ---------------------------------------------------------
# Build executable
# ---------------------------------------------------------
add_executable(ransac_speedbump_node
  src/ransac_speedbump_node.cpp
)

ament_target_dependencies(ransac_speedbump_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
)

target_link_libraries(ransac_speedbump_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# ---------------------------------------------------------
# Install
# ---------------------------------------------------------
install(TARGETS
  ransac_speedbump_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# ---------------------------------------------------------
# Export
# ---------------------------------------------------------
ament_export_include_directories(include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

ament_package()

