cmake_minimum_required(VERSION 3.8)
project(forward_turn_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
    rclcpp
    geometry_msgs
    tf2_geometry_msgs
    tf2_ros
    tf2
)
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/forward_turn_cpp/ForwardNode.cpp
  src/forward_turn_cpp/TurnNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(forward src/forward.cpp)
ament_target_dependencies(forward ${dependencies})
target_link_libraries(forward ${PROJECT_NAME})

add_executable(turn src/turn.cpp)
ament_target_dependencies(turn ${dependencies})
target_link_libraries(turn ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  forward
  turn
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
