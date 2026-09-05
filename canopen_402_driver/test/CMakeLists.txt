ament_add_gtest(test_driver_component
test_driver_component.cpp
)
ament_target_dependencies(test_driver_component
  ${dependencies}
)
target_include_directories(test_driver_component PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/
)

ament_add_gtest(test_motor
  test_motor.cpp
)
ament_target_dependencies(test_motor
  ${dependencies}
)
target_link_libraries(test_motor
  lely_motion_controller_bridge
)
