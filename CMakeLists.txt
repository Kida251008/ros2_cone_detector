cmake_minimum_required(VERSION 3.5)
project(cone_detection)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/log_img_pub.cpp
 # src/log_pcd_pub.cpp
  src/cone_detection.cpp
)

target_link_libraries(${PROJECT_NAME}
  rs_common
  cone_core
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp OpenCV 
  cv_bridge 
  sensor_msgs 
  rclcpp_components 
)

rclcpp_components_register_nodes(${PROJECT_NAME} "cone_detector::LogImgPublisher")
#rclcpp_components_register_nodes(${PROJECT_NAME} "cone_detector::LogPcdPublisher")
rclcpp_components_register_nodes(${PROJECT_NAME} "cone_detector::Recognition")

# Register component
ament_export_dependencies(rclcpp)
ament_export_include_directories(include)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch cfg
  DESTINATION share/${PROJECT_NAME}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_package()