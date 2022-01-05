cmake_minimum_required(VERSION 3.8)
project(spraybot_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_localization REQUIRED)

include_directories(
  include
)

set(dependencies
  behaviortree_cpp_v3
  geometry_msgs
  nav2_behavior_tree
  rclcpp
  robot_localization
)

add_library(next_goal_bt_node SHARED plugins/next_goal.cpp)
list(APPEND plugin_libs next_goal_bt_node)

add_library(convert_ll2goal_bt_node SHARED plugins/convert_ll2goal.cpp)
list(APPEND plugin_libs convert_ll2goal_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${library_name}
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

install(FILES spraybot_tree_nodes.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Disable copyright and license checks
  set(ament_cmake_copyright_FOUND TRUE)
  # Disable cpplint and manually call it with filters later
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  ament_cpplint(FILTERS "-legal/copyright" "-build/header_guard" "-build/include_order")
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()
