cmake_minimum_required(VERSION 3.8)
project(hdl_people_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(hdl_people_tracking_msgs REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io filters kdtree search sample_consensus segmentation)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(hdl_people_tracking_component SHARED
  apps/hdl_people_detection_node.cpp
  apps/hdl_people_tracking_node.cpp
  src/kidono_human_classifier.cpp
  src/people_detector.cpp
)
target_compile_features(hdl_people_tracking_component PUBLIC cxx_std_14)
target_link_libraries(hdl_people_tracking_component
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(hdl_people_tracking_component
  ament_index_cpp
  rclcpp
  rclcpp_components
  pcl_conversions
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  hdl_people_tracking_msgs
)

rclcpp_components_register_nodes(hdl_people_tracking_component
  "hdl_people_tracking::HdlPeopleDetectionNode"
  "hdl_people_tracking::HdlPeopleTrackingNode"
)

install(TARGETS hdl_people_tracking_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(hdl_people_tracking_component)
ament_export_dependencies(
  ament_index_cpp
  rclcpp
  rclcpp_components
  pcl_conversions
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  hdl_people_tracking_msgs
)

ament_package()
