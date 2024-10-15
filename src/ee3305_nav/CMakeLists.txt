cmake_minimum_required(VERSION 3.8)
project(ee3305_nav)

# g++ warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# behavior node
add_executable(behavior src/behavior.cpp)
ament_target_dependencies(behavior
  geometry_msgs
  nav_msgs
  rclcpp
)
target_include_directories(behavior PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
install(
  TARGETS behavior
  DESTINATION lib/${PROJECT_NAME}
)

# map_server node
add_executable(map_server src/map_server.cpp)
ament_target_dependencies(map_server
  nav_msgs
  rclcpp
)
target_include_directories(map_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
install(
  TARGETS map_server
  DESTINATION lib/${PROJECT_NAME}
)

# estimator node
add_executable(estimator src/estimator.cpp)
ament_target_dependencies(estimator
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
)
target_include_directories(estimator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
install(
  TARGETS estimator
  DESTINATION lib/${PROJECT_NAME}
)

# planner node
add_executable(planner src/planner.cpp)
ament_target_dependencies(planner
  geometry_msgs
  nav_msgs
  rclcpp
)
target_include_directories(planner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
install(
  TARGETS planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
