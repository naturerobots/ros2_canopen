cmake_minimum_required(VERSION 3.8)
project(canopen_scanreco_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(canopen_base_driver REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(four_wheel_steering_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  canopen_core
  canopen_interfaces
  canopen_base_driver
  canopen_proxy_driver
  lely_core_libraries
  geometry_msgs
  four_wheel_steering_msgs
  std_msgs
  std_srvs
)

# Functionality library
add_library(node_canopen_scanreco_driver
src/node_interfaces/node_canopen_scanreco_driver.cpp
)
target_compile_features(node_canopen_scanreco_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(node_canopen_scanreco_driver PUBLIC -Wl,--no-undefined)
target_include_directories(node_canopen_scanreco_driver PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  node_canopen_scanreco_driver
  ${dependencies}
)

# Non lifecycle driver
add_library(scanreco_driver
  src/scanreco_driver.cpp
)
target_compile_features(scanreco_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(scanreco_driver PUBLIC -Wl,--no-undefined)
target_include_directories(scanreco_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(scanreco_driver
  node_canopen_scanreco_driver
)

ament_target_dependencies(
  scanreco_driver
  ${dependencies}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(scanreco_driver PRIVATE "CANOPEN_SCANRECO_DRIVER_BUILDING_LIBRARY")

rclcpp_components_register_nodes(scanreco_driver "ros2_canopen::ScanrecoDriver")
set(node_plugins "${node_plugins}ros2_canopen::ScanrecoDriver;$<TARGET_FILE:scanreco_driver >\n")

install(
DIRECTORY include/
DESTINATION include
)

install(
TARGETS scanreco_driver node_canopen_scanreco_driver
EXPORT export_${PROJECT_NAME}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
endif()

ament_export_include_directories(
include
)
ament_export_libraries(
scanreco_driver
node_canopen_scanreco_driver
)
ament_export_targets(
export_${PROJECT_NAME}
)

ament_package()
