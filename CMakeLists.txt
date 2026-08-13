cmake_minimum_required(VERSION 3.8)
project(vins_mah)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(OpenCV REQUIRED HINTS /usr/local/lib)

find_package(Ceres REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  ament_index_cpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  Eigen3
  glog
  yaml-cpp
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

# =====================================================================
# Libraries
# =====================================================================

add_library(vins_mah SHARED
  src/estimator.cpp

  src/feature_manager.cpp
  src/feature_tracker.cpp

  src/camera_model/pinhole_camera.cpp

  src/factor/pose_local_parameterization.cpp
  src/factor/projection_two_frame_one_cam.cpp
  src/factor/projection_two_frame_two_cam.cpp
  src/factor/projection_one_frame_two_cam.cpp
  src/factor/marginalization.cpp

  src/utils/rotations.cpp
  src/utils/camera_pose_visualization.cpp

  src/initial/solve_5pts.cpp
  src/initial/alignment.cpp
  src/initial/sfm.cpp
  src/initial/ex_rotation.cpp
  )

target_include_directories(vins_mah PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
  )

ament_target_dependencies(vins_mah
  ${DEPENDENCIES}
  )

target_link_libraries(vins_mah
  glog::glog
  ${CERES_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
  )

target_compile_definitions(${PROJECT_NAME} PRIVATE
  GLOG_USE_GLOG_EXPORT
  )

# =====================================================================
# Composable nodes
# =====================================================================

add_library(estimator_node_component SHARED
  src/estimator_node.cpp
  src/vio_options_manager.cpp
  )

target_include_directories(estimator_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

ament_target_dependencies(estimator_node_component
  ${DEPENDENCIES}
  )

target_link_libraries(estimator_node_component
  vins_mah
  ${OpenCV_LIBRARIES}
  yaml-cpp
  )

rclcpp_components_register_node(
  estimator_node_component
  PLUGIN "vins_mah::EstimatorNode"
  EXECUTABLE estimator_node
  )

# =====================================================================
# Install
# =====================================================================

install(TARGETS vins_mah estimator_node_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(
  DIRECTORY include/
  DESTINATION include
  )

install(
  DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
  )

# =====================================================================
# Export
# =====================================================================

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_include_directories(include)

ament_export_dependencies(${DEPENDENCIES})

ament_package()
