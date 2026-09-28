cmake_minimum_required(VERSION 3.8)
project(canopen_thomson_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wpedantic -Wextra -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(canopen_base_driver REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(dependencies
  canopen_base_driver
  canopen_core
  canopen_interfaces
  canopen_proxy_driver
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
)

add_library(node_canopen_thomson_driver
  src/node_interfaces/node_canopen_thomson_driver.cpp
)
target_compile_features(node_canopen_thomson_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(node_canopen_thomson_driver PUBLIC -Wl,--no-undefined)
target_include_directories(node_canopen_thomson_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  node_canopen_thomson_driver
  ${dependencies}
)

add_library(thomson_driver
  src/thomson_driver.cpp
)
target_compile_features(thomson_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(thomson_driver PUBLIC -Wl,--no-undefined)
target_include_directories(thomson_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(thomson_driver
  node_canopen_thomson_driver
)
ament_target_dependencies(
  thomson_driver
  ${dependencies}
)

rclcpp_components_register_nodes(thomson_driver "ros2_canopen::ThomsonDriver")
set(node_plugins "${node_plugins}ros2_canopen::ThomsonDriver;$<TARGET_FILE:thomson_driver>\n")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS thomson_driver node_canopen_thomson_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  thomson_driver
  node_canopen_thomson_driver
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(
  ${dependencies}
)

ament_package()
