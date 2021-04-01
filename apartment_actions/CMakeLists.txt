cmake_minimum_required(VERSION 3.5)
project(apartment_actions)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_action
  plansys2_msgs
  plansys2_executor
  nav2_msgs
  plansys2_problem_expert
  plansys2_domain_expert
)

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

add_executable(grab_object_action_node src/grab_object_action_node.cpp)
ament_target_dependencies(grab_object_action_node ${dependencies})

add_executable(release_object_action_node src/release_object_action_node.cpp)
ament_target_dependencies(release_object_action_node ${dependencies})

add_executable(apartment_controller_node src/apartment_controller_node.cpp)
ament_target_dependencies(apartment_controller_node ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_action_node
  grab_object_action_node
  release_object_action_node
  apartment_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
