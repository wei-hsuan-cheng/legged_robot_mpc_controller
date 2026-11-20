cmake_minimum_required(VERSION 3.14)
project(legged_robot_mpc_controller LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_index_cpp
  builtin_interfaces
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  legged_robot_state_estimator
  nav_msgs
  ocs2_core
  ocs2_mpc
  ocs2_msgs
  ocs2_oc
  ocs2_pinocchio_interface
  ocs2_robotic_tools
  ocs2_ros_interfaces
  ocs2_sqp
  pinocchio
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  std_srvs
  tf2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
  visualization_msgs
)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup)
find_package(yaml-cpp REQUIRED)

foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(humanoid_whole_body_inverse_dynamics_mpc_controller_parameters
  src/controller/parameters.yaml
)

file(GLOB_RECURSE HUMANOID_COMMON_CORE_SOURCES CONFIGURE_DEPENDS
  src/mpc/common/*.cpp
)
file(GLOB_RECURSE HUMANOID_WB_ID_CORE_SOURCES CONFIGURE_DEPENDS
  src/mpc/wb_id/*.cpp
)

add_library(legged_robot_humanoid_common_core SHARED
  ${HUMANOID_COMMON_CORE_SOURCES}
)
target_compile_features(legged_robot_humanoid_common_core PUBLIC cxx_std_20)
target_include_directories(legged_robot_humanoid_common_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(legged_robot_humanoid_common_core PUBLIC
  ament_index_cpp
  ocs2_core
  ocs2_sqp
  ocs2_msgs
  ocs2_mpc
  ocs2_pinocchio_interface
  ocs2_robotic_tools
  pinocchio
  rclcpp
  urdf
)
target_link_libraries(legged_robot_humanoid_common_core PUBLIC Eigen3::Eigen ${Boost_LIBRARIES})
target_compile_definitions(legged_robot_humanoid_common_core PUBLIC BOOST_MPL_LIMIT_LIST_SIZE=30)

# Whole-body inverse-dynamics MPC core.
add_library(legged_robot_humanoid_wb_id_core SHARED
  ${HUMANOID_WB_ID_CORE_SOURCES}
)
target_compile_features(legged_robot_humanoid_wb_id_core PUBLIC cxx_std_20)
target_include_directories(legged_robot_humanoid_wb_id_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(legged_robot_humanoid_wb_id_core PUBLIC
  ament_index_cpp
  ocs2_core
  ocs2_mpc
  ocs2_oc
  ocs2_pinocchio_interface
  ocs2_robotic_tools
  ocs2_sqp
  pinocchio
  rclcpp
)
target_link_libraries(legged_robot_humanoid_wb_id_core
  PUBLIC
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    legged_robot_humanoid_common_core
)
target_compile_definitions(legged_robot_humanoid_wb_id_core PUBLIC BOOST_MPL_LIMIT_LIST_SIZE=30)

add_library(${PROJECT_NAME} SHARED
  src/common/config_builder_utils.cpp
  src/common/ros2_procedural_mpc_motion_manager.cpp
  src/common/target/mpc_targets_parser.cpp
  src/controller/config_builder.cpp
  src/controller/controller.cpp
  src/visualization/optimized_state_trajectory_visualization.cpp
  src/visualization/future_robot_model_visualization.cpp
  src/visualization/collision_sphere_visualization.cpp
  src/visualization/performance_visualization.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    humanoid_whole_body_inverse_dynamics_mpc_controller_parameters
    legged_robot_humanoid_wb_id_core
    legged_robot_state_estimator::legged_robot_state_estimator
    yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(
  controller_interface
  legged_robot_mpc_controller_plugin.xml
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config docs description
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*.pyo" EXCLUDE
  PATTERN ".pytest_cache" EXCLUDE
  PATTERN ".DS_Store" EXCLUDE
)
install(PROGRAMS
  launch/base_command_gui.py
  launch/figure_eight_command.py
  launch/push_recovery_test.py
  launch/command/mpc_targets/joint_tracking_target.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES legged_robot_mpc_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  humanoid_whole_body_inverse_dynamics_mpc_controller_parameters
  legged_robot_humanoid_common_core
  legged_robot_humanoid_wb_id_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  add_executable(model_settings_test tests/model_settings_test.cpp)
  target_link_libraries(model_settings_test PRIVATE legged_robot_humanoid_wb_id_core)
  target_compile_options(model_settings_test PRIVATE -UNDEBUG)
  add_test(NAME model_settings_test COMMAND model_settings_test
    ${CMAKE_CURRENT_SOURCE_DIR}/description/urdf/robots)
  add_executable(wheel_model_consistency_test tests/wheel_model_consistency_test.cpp)
  target_link_libraries(wheel_model_consistency_test PRIVATE legged_robot_humanoid_wb_id_core)
  add_test(NAME wheel_model_consistency_test COMMAND wheel_model_consistency_test
    ${CMAKE_CURRENT_SOURCE_DIR}/description/urdf/robots/upkie/upkie.urdf
    ${CMAKE_CURRENT_BINARY_DIR}/test_codegen)
  foreach(test_name IN ITEMS wheel_contact_kinematics_test arm_body_collision_test
      swing_flight_test gait_swing_height_test running_gait_transition_test wb_id_torque_window_test)
    add_executable(${test_name} tests/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE legged_robot_humanoid_wb_id_core)
    # These regression executables use assert; keep their checks active in Release builds.
    target_compile_options(${test_name} PRIVATE -UNDEBUG)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(
  ${PROJECT_NAME}
  legged_robot_humanoid_common_core
  legged_robot_humanoid_wb_id_core
)

ament_package()
