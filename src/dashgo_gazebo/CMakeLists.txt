cmake_minimum_required(VERSION 3.0.2)
project(dashgo_gazebo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/sim_safety_cmd_gate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch maps meshes models third_party_licenses urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
