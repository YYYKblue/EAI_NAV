cmake_minimum_required(VERSION 3.0.2)
project(ydlidar)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
)

catkin_package()

set(YDLIDAR_SDK_SOURCES
  sdk/src/CYdLidar.cpp
  sdk/src/serial.cpp
  sdk/src/ydlidar_driver.cpp
  sdk/src/impl/unix/unix_serial.cpp
  sdk/src/impl/unix/unix_timer.cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  sdk/include
  sdk/src
)

add_executable(ydlidar_node
  src/ydlidar_node.cpp
  ${YDLIDAR_SDK_SOURCES}
)
target_link_libraries(ydlidar_node
  ${catkin_LIBRARIES}
  pthread
  rt
)

install(TARGETS ydlidar_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
