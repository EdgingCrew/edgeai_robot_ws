cmake_minimum_required(VERSION 3.10)
project(edgeai_rescue)

# ament_cmake rather than ament_python: colcon's --symlink-install for setup.py
# packages needs 'setup.py develop', which the Pixi setuptools no longer has.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/robot_controller_node
    scripts/planner_node
    scripts/rescue_coordinator
    scripts/summarize_run
    scripts/preview_scenario
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

# Plain pytest, no ROS needed: pixi run python -m pytest src/edgeai_rescue
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_edgeai_rescue test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 300)
endif()

ament_package()
