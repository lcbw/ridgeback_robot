cmake_minimum_required(VERSION 3.16)
project(ridgeback_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES scripts/can-udp-bridge.conf scripts/can-udp-bridge.sh
  DESTINATION bin
)

install(PROGRAMS scripts/install
  DESTINATION bin
)

#if (BUILD_TESTING)
#  find_package(roslaunch REQUIRED)
#  roslaunch_add_file_check(launch/accessories.launch)
#endif()

ament_package()
