cmake_minimum_required(VERSION 3.5.0)
project(ridgeback_base)
# Default to C++14

if(NOT CMAKE_CXX_STANDARD)

  set(CMAKE_CXX_STANDARD 14)

endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  add_compile_options(-Wall -Wextra -Wpedantic)

endif()


find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  generate_parameter_library
  diagnostic_updater
  geometry_msgs
  hardware_interface
  puma_motor_driver
  puma_motor_msgs
  realtime_tools
  ridgeback_msgs
  rclcpp
  tf2
  tf2_msgs
  sensor_msgs
  std_msgs
  Boost
  Eigen3)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

generate_parameter_library(
  ridgeback_base_parameters
  config/ridgeback_base_parameters.yaml
)
add_executable(ridgeback_base
  src/ridgeback_base.cpp
  src/ridgeback_cooling.cpp
  src/ridgeback_diagnostic_updater.cpp
  src/ridgeback_hardware.cpp
  src/ridgeback_lighting.cpp
  )

target_link_libraries(ridgeback_base PUBLIC ridgeback_base_parameters)

ament_target_dependencies(ridgeback_base PUBLIC
${THIS_PACKAGE_INCLUDE_DEPENDS}
controller_interface
hardware_interface
rclcpp)

target_include_directories(ridgeback_base SYSTEM PUBLIC
  include)
#  ${controller_interface_INCLUDE_DIRS}
#  ${diagnostic_updater_INCLUDE_DIRS}
#  ${realtime_tools_INCLUDE_DIRS}
#  ${geometry_msgs_INCLUDE_DIRS}
#  ${nav_msgs_INCLUDE_DIRS}
#  ${sensor_msgs_INCLUDE_DIRS}
#  ${tf2_INCLUDE_DIRS}
#  ${tf2_msgs_INCLUDE_DIRS}
#  ${hardware_interface_INCLUDE_DIRS}
#  ${puma_motor_driver_INCLUDE_DIRS}
#  ${puma_motor_msgs_INCLUDE_DIRS}
#  ${ridgeback_msgs_INCLUDE_DIRS}
#  ${rclcpp_INCLUDE_DIRS}
#  ${Boost_INCLUDE_DIRS})

target_compile_definitions(ridgeback_base PRIVATE "RIDGEBACK_BASE_BUILDING_DLL")

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ridgeback_base
  DESTINATION include)

install(PROGRAMS scripts/calibrate_compass scripts/compute_calibration
 DESTINATION bin)

install(TARGETS ridgeback_base ridgeback_base_parameters
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

#if(BUILD_TESTING)
#  find_package(roslaunch REQUIRED)
#  roslaunch_add_file_check(launch/base.launch)

#  find_package(roslint REQUIRED)
#  roslint_cpp()
#  roslint_add_test()
#endif()

pluginlib_export_plugin_description_file(hardware_interface ridgeback_hardware.xml)

#ament_export_targets(export_ridgeback_node HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface realtime_tools hardware_interface) ## transitive dependencies - you shouldn't need to findpackage these libararies yourself if you're just using *this, here*
#ament_export_include_directories(include) ## what other packages need when linking against this library (headers)
ament_export_libraries(ridgeback_base)  ## what other packages need when linking against this library (source files)

ament_package()
