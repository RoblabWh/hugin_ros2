cmake_minimum_required(VERSION 3.8)
project(virtcam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_info_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)

find_package(Eigen3 REQUIRED)
add_subdirectory(thirdparty/PanoWeave)

# build component
add_library(${PROJECT_NAME} SHARED src/perspective.cpp src/spherical.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  "rclcpp"
  "rclcpp_components"
  "cv_bridge"
  "sensor_msgs"
  "image_info_msgs"
  "tf2"
  "tf2_ros"
  "tf2_eigen"
  "message_filters"
)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
  PanoWeave
)

# generate nodes
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "virtcam::PerspectiveCamera" EXECUTABLE perspective_camera_node EXECUTOR SingleThreadedExecutor)
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "virtcam::SphericalCamera" EXECUTABLE spherical_camera_node EXECUTOR SingleThreadedExecutor)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
