cmake_minimum_required(VERSION 3.8)
project(edgeai_rescue)

# ament_cmake rather than ament_python: colcon's --symlink-install for setup.py
# packages needs 'setup.py develop', which the Pixi setuptools no longer has.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/robot_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_allocation.py)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_allocation test/test_allocation.py)
endif()

ament_package()
