cmake_minimum_required(VERSION 3.5)
project(blackboard)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(blackboard_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap_msgs)

set(dependencies
    rclcpp
    std_msgs
    blackboard_msgs
    geometry_msgs
    octomap_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/blackboard/BlackBoard.cpp
  src/blackboard/BlackBoardNode.cpp
  src/blackboard/BlackBoardClient.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(blackboard_main
  src/blackboard_main.cpp
)
ament_target_dependencies(blackboard_main ${dependencies})
target_link_libraries(blackboard_main ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  blackboard_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)

endif()

ament_package()
