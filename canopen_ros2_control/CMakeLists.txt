cmake_minimum_required(VERSION 3.8)
project(canopen_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  canopen_402_driver
  canopen_core
  canopen_proxy_driver
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_srvs
)

find_package(ros2_control_test_assets)
find_package(ament_cmake_gmock REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AdjustPositionOffset.srv"
)

add_library(
  ${PROJECT_NAME}_lib
  SHARED
  src/canopen_system.cpp
  src/cia402_system.cpp
  src/robot_system.cpp
)

set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME}_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(${PROJECT_NAME}_lib PUBLIC -fPIC -Wl,--no-undefined)

target_include_directories(${PROJECT_NAME}_lib PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_lib ${THIS_PACKAGE_INCLUDE_DEPENDS})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME}_lib PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface canopen_ros2_control.xml)

install(
  TARGETS
  ${PROJECT_NAME}_lib
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  #find_package(ament_cmake_gmock REQUIRED)
  #find_package(ros2_control_test_assets REQUIRED)

  #ament_add_gmock(test_canopen_system test/test_canopen_system.cpp)
  #target_include_directories(test_canopen_system PRIVATE include)
  #ament_target_dependencies(
    #test_canopen_system ${THIS_PACKAGE_INCLUDE_DEPENDS} ros2_control_test_assets)
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}_lib
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
