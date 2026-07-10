cmake_minimum_required(VERSION 3.14)
project(legged_robot_mpc_controller LANGUAGES CXX)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_index_cpp
  controller_interface
  generate_parameter_library
  hardware_interface
  nav_msgs
  ocs2_centroidal_model
  ocs2_core
  ocs2_ddp
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
  tf2_ros
  urdf
)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup)
find_package(yaml-cpp REQUIRED)

foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(humanoid_wb_mpc_controller_parameters
  src/humanoid_wb_mpc_controller_parameter.yaml
)

file(GLOB_RECURSE HUMANOID_COMMON_CORE_SOURCES CONFIGURE_DEPENDS
  src/core/humanoid_common_mpc/src/*.cpp
)
file(GLOB_RECURSE HUMANOID_WB_CORE_SOURCES CONFIGURE_DEPENDS
  src/core/humanoid_wb_mpc/src/*.cpp
)
file(GLOB_RECURSE HUMANOID_CENTROIDAL_CORE_SOURCES CONFIGURE_DEPENDS
  src/core/humanoid_centroidal_mpc/src/*.cpp
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

add_library(legged_robot_humanoid_wb_core SHARED
  ${HUMANOID_WB_CORE_SOURCES}
)
target_compile_features(legged_robot_humanoid_wb_core PUBLIC cxx_std_20)
target_include_directories(legged_robot_humanoid_wb_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(legged_robot_humanoid_wb_core PUBLIC
  ament_index_cpp
  ocs2_centroidal_model
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_oc
  ocs2_pinocchio_interface
  ocs2_robotic_tools
  ocs2_sqp
  pinocchio
  rclcpp
)
target_link_libraries(legged_robot_humanoid_wb_core
  PUBLIC
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    legged_robot_humanoid_common_core
)
target_compile_definitions(legged_robot_humanoid_wb_core PUBLIC BOOST_MPL_LIMIT_LIST_SIZE=30)

add_library(legged_robot_humanoid_centroidal_core SHARED
  ${HUMANOID_CENTROIDAL_CORE_SOURCES}
)
target_compile_features(legged_robot_humanoid_centroidal_core PUBLIC cxx_std_20)
target_include_directories(legged_robot_humanoid_centroidal_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(legged_robot_humanoid_centroidal_core PUBLIC
  ament_index_cpp
  ocs2_centroidal_model
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_oc
  ocs2_pinocchio_interface
  ocs2_robotic_tools
  ocs2_sqp
  pinocchio
  rclcpp
)
target_link_libraries(legged_robot_humanoid_centroidal_core
  PUBLIC
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    legged_robot_humanoid_common_core
)
target_compile_definitions(legged_robot_humanoid_centroidal_core PUBLIC BOOST_MPL_LIMIT_LIST_SIZE=30)

add_library(${PROJECT_NAME} SHARED
  src/humanoid_wb_mpc_controller.cpp
  src/config/wb_mpc_config_builder.cpp
  src/ros2_procedural_mpc_motion_manager.cpp
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
    humanoid_wb_mpc_controller_parameters
    legged_robot_humanoid_wb_core
    legged_robot_humanoid_centroidal_core
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
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(PROGRAMS
  launch/base_velocity_controller_gui.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)
install(DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)
install(FILES legged_robot_mpc_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  humanoid_wb_mpc_controller_parameters
  legged_robot_humanoid_common_core
  legged_robot_humanoid_wb_core
  legged_robot_humanoid_centroidal_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
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
  legged_robot_humanoid_wb_core
  legged_robot_humanoid_centroidal_core
)

ament_package()
