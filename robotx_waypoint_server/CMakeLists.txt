cmake_minimum_required(VERSION 3.5)
project(robotx_waypoint_server)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robotx_waypoint_msgs REQUIRED)

include_directories(include ${CMAKE_CURRENT_BINARY_DIR})
#include_directories(${CMAKE_CURRENT_BINARY_DIR})

##################################################
######             COMPONENT               #######
##################################################
add_library(waypoint_server_component SHARED
        src/waypoint_server_component.cpp)

target_compile_definitions(waypoint_server_component
        PRIVATE "WAYPOINT_SERVER_COMPONENT_BUILDING_DLL")

ament_target_dependencies(waypoint_server_component
        rclcpp
        rclcpp_action
        rclcpp_components
        robotx_waypoint_msgs
)

rclcpp_components_register_nodes(waypoint_server_component
        "waypoint_server::WaypointServerComponent")

##################################################
######                NODE                 #######
##################################################
add_executable(waypoint_server_node
        src/waypoint_server_node.cpp
)

ament_target_dependencies(
  waypoint_server_node
  rclcpp
  rclcpp_action
  geometry_msgs
  robotx_waypoint_msgs
)

target_link_libraries(waypoint_server_node waypoint_server_component)

install(TARGETS waypoint_server_node
        DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS waypoint_server_component
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)
ament_export_libraries(waypoint_server_component)

install(DIRECTORY "include/"
        DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
