cmake_minimum_required(VERSION 3.0.2)
project(imu_calibration)

find_package(catkin REQUIRED)

catkin_package()

add_custom_target(${PROJECT_NAME}_install_python_package ALL COMMAND python3 -m pip install AllanTools plotly)

install(
  PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
