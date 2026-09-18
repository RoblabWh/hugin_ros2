cmake_minimum_required(VERSION 3.22)
project(cairn)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# thirdparty
# cuVSLAM, built as an ExternalProject -> IMPORTED target `cuvslam`.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/BuildCuvslam.cmake)
set(BASALT_BUILTIN_EIGEN OFF CACHE BOOL "" FORCE)
set(SOPHUS_USE_BASIC_LOGGING ON CACHE BOOL "" FORCE)
set(SOPHUS_INSTALL OFF CACHE BOOL "" FORCE)
set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "" FORCE)
set(JUST_INSTALL_CEREAL ON CACHE BOOL "" FORCE)
set(_cairn_build_testing ${BUILD_TESTING})
add_subdirectory(thirdparty/basalt-headers EXCLUDE_FROM_ALL)
set(BUILD_TESTING ${_cairn_build_testing} CACHE BOOL "Build the testing tree." FORCE)

# library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/calibration.cpp
  src/frame_sync.cpp
  src/tracker.cpp
)
target_link_libraries(${PROJECT_NAME} cuvslam basalt::basalt-headers)
add_dependencies(${PROJECT_NAME} cuvslam_ext)
# libcuvslam.so is installed next to the component, resolve it via RPATH in both trees.
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
  INSTALL_RPATH "$ORIGIN"
)

# generate node
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "cairn::Tracker" EXECUTABLE ${PROJECT_NAME}_node EXECUTOR SingleThreadedExecutor)

# install dependencies
install(FILES ${CAIRN_CUVSLAM_LIB} DESTINATION lib)

# tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_calibration test/test_calibration.cpp)
  if(TARGET test_calibration)
    target_link_libraries(test_calibration ${PROJECT_NAME})
    target_compile_definitions(test_calibration PRIVATE
      CAIRN_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
    set_target_properties(test_calibration PROPERTIES
      BUILD_WITH_INSTALL_RPATH TRUE INSTALL_RPATH "$ORIGIN")
  endif()

  ament_add_gtest(test_frame_sync test/test_frame_sync.cpp)
  if(TARGET test_frame_sync)
    target_link_libraries(test_frame_sync ${PROJECT_NAME})
    set_target_properties(test_frame_sync PROPERTIES
      BUILD_WITH_INSTALL_RPATH TRUE INSTALL_RPATH "$ORIGIN")
  endif()
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
