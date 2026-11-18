cmake_minimum_required(VERSION 3.16)
project(edgeai_bt_plugins)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Loaded by bt_navigator through plugin_lib_names (config/nav2_params.yaml)
add_library(edgeai_angle_towards_path_bt_node SHARED src/angle_towards_path.cpp)
target_compile_features(edgeai_angle_towards_path_bt_node PRIVATE cxx_std_17)
target_compile_definitions(edgeai_angle_towards_path_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(edgeai_angle_towards_path_bt_node
  behaviortree_cpp::behaviortree_cpp
  ${nav_msgs_TARGETS}
  tf2_ros::tf2_ros
)

install(TARGETS edgeai_angle_towards_path_bt_node LIBRARY DESTINATION lib)

ament_package()
