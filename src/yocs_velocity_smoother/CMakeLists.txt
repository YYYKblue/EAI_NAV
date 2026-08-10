cmake_minimum_required(VERSION 3.0.2)
project(yocs_velocity_smoother)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  nodelet
  pluginlib
  roscpp
)

generate_dynamic_reconfigure_options(cfg/params.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_nodelet
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs nav_msgs nodelet pluginlib roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_nodelet src/velocity_smoother_nodelet.cpp)
add_dependencies(${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
