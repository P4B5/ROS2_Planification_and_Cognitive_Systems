cmake_minimum_required(VERSION 3.5)
project(cognitive_apartment)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(blackboard REQUIRED)
find_package(blackboard_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(image_transport REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common io console)
find_package(pcl_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  rclcpp_action
  plansys2_msgs
  plansys2_executor
  geometry_msgs
  nav2_msgs
  plansys2_problem_expert
  plansys2_domain_expert
  std_msgs
  rclcpp_cascade_lifecycle
  lifecycle_msgs
  blackboard
  blackboard_msgs
  tf2_ros
  image_transport
  cv_bridge
  sensor_msgs
  pcl_msgs
)

include_directories(fsm)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PCL_COMMON_INCLUDE_DIRS})

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_executable(sync_node src/sync_node.cpp)
ament_target_dependencies(sync_node ${dependencies})

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

add_executable(grab_object_action_node src/grab_object_action_node.cpp)
ament_target_dependencies(grab_object_action_node ${dependencies})

add_executable(release_object_action_node src/release_object_action_node.cpp)
ament_target_dependencies(release_object_action_node ${dependencies})

add_executable(explore_action_node src/explore_action_node.cpp)
ament_target_dependencies(explore_action_node ${dependencies})

add_executable(object_recognition src/object_recognition.cpp)
ament_target_dependencies(object_recognition ${dependencies})


add_executable(hfsm_cognitive_apartment 
  src/hfsm_imp_cognitive_apartment.cpp
  fsm/hfsm_cognitive_apartment.cpp
)
ament_target_dependencies(hfsm_cognitive_apartment ${dependencies})

target_link_libraries(object_recognition
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
  ${PCL_LIBRARIES}
)

install(DIRECTORY launch pddl config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  explore_action_node
  move_action_node
  grab_object_action_node
  release_object_action_node
  hfsm_cognitive_apartment
  sync_node
  object_recognition
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
