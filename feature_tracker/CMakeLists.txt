cmake_minimum_required(VERSION 3.5)
project(feature_tracker)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

######################
# PACKAGE VISUAL
###################### 
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED) 
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(camera_model REQUIRED)
include_directories(${camera_model_INCLUDE_DIRS})

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

######################
# LIDAR
###################### 
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

######################
# BỔ SUNG CHO FUSION (GTSAM & GeographicLib)
###################### 
find_package(GTSAM REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GeographicLib REQUIRED geographiclib)

######################
# CUSTOM MESSAGES GENERATION
######################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CloudInfo.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

######################
# EXECUTABLE
###################### 
add_executable(feature_tracker_node
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
)

add_executable(${PROJECT_NAME}_imageProjection 
   src/imageProjection.cpp
)

# ✅ BỔ SUNG: 2 node mới của bạn
add_executable(pose_graph_fusion_node src/pose_graph_fusion_node.cpp)
add_executable(gps_to_local_node src/gps_to_local_node.cpp)

######################
# LINK LIBRARIES (Non-ROS2 packages)
###################### 
target_link_libraries(feature_tracker_node 
  ${camera_model_LIBRARIES} 
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_imageProjection
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

# ✅ BỔ SUNG: Link library cho 2 node mới
target_include_directories(pose_graph_fusion_node PUBLIC ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(pose_graph_fusion_node gtsam ${GeographicLib_LIBRARIES} ${PCL_LIBRARIES})

target_link_libraries(gps_to_local_node Eigen3::Eigen)

######################
# AMENT DEPENDENCIES (ROS2 packages)
###################### 
ament_target_dependencies(feature_tracker_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  nav_msgs              
  std_msgs              
  visualization_msgs
  pcl_conversions
  pcl_msgs
  tf2
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  tf2_ros
)

ament_target_dependencies(${PROJECT_NAME}_imageProjection
  rclcpp
  sensor_msgs
  cv_bridge
  nav_msgs              
  std_msgs              
  visualization_msgs
  pcl_conversions
  pcl_msgs
  tf2
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  tf2_ros
)

# ✅ BỔ SUNG: Dependencies cho 2 node mới
ament_target_dependencies(pose_graph_fusion_node 
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros pcl_conversions
)

ament_target_dependencies(gps_to_local_node 
  rclcpp sensor_msgs nav_msgs geometry_msgs
)

rosidl_target_interfaces(feature_tracker_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(${PROJECT_NAME}_imageProjection ${PROJECT_NAME} "rosidl_typesupport_cpp")

######################
# INSTALL
###################### 
install(
  TARGETS 
    feature_tracker_node
    ${PROJECT_NAME}_imageProjection
    pose_graph_fusion_node
    gps_to_local_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()