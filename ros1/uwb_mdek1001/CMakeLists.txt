cmake_minimum_required(VERSION 2.8.3)
project(uwb_mdek1001)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS scripts/uwb_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES scripts/lec_parser.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
