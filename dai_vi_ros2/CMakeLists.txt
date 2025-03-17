cmake_minimum_required(VERSION 3.8)
project(dai_vi_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_info_msgs REQUIRED)
find_package(depthai CONFIG REQUIRED)

# build thirdparty
set(BUILD_EXEC OFF CACHE BOOL "Only build the library" FORCE)
add_subdirectory(thirdparty/dai-vi)

# build component
add_library(${PROJECT_NAME} SHARED src/dai_vi_ros2.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "image_info_msgs"
)
target_link_libraries(${PROJECT_NAME} PRIVATE dai_vi depthai::opencv)

# generate node
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "dai_vi::ROS2Wrapper" EXECUTABLE ${PROJECT_NAME}_node EXECUTOR SingleThreadedExecutor)

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
