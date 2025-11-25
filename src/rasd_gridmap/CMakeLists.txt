cmake_minimum_required(VERSION 3.8)
project(rasd_gridmap)

# 1️⃣ Tell CMake we are using modern C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2️⃣ Find all ROS 2 and external packages we depend on
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS io filters)

# 3️⃣ Build our node from its source file
add_executable(cloud_to_grid_node src/cloud_to_grid_node.cpp)

# 4️⃣ Link the libraries and make sure the dependencies are available
ament_target_dependencies(cloud_to_grid_node
  rclcpp
  sensor_msgs
  grid_map_ros
  grid_map_msgs
  tf2_ros
  pcl_conversions
)

# 5️⃣ Link against PCL system libraries (for point cloud ops)
target_link_libraries(cloud_to_grid_node ${PCL_LIBRARIES})

# 6️⃣ Install the executable into ROS 2’s expected path
install(TARGETS
  cloud_to_grid_node
  DESTINATION lib/${PROJECT_NAME}
)

# 7️⃣ Declare the package for colcon/ament
ament_package()

