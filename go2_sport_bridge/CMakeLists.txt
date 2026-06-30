cmake_minimum_required(VERSION 3.10)
project(go2_sport_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# unitree_sdk2 is installed by Unitree's install.sh under /opt/unitree_robotics.
# Add it to the search path so find_package() locates unitree_sdk2Config.cmake.
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_sdk2 REQUIRED)

add_executable(sport_mode_adapter_node src/sport_mode_adapter_node.cpp)
ament_target_dependencies(sport_mode_adapter_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)
target_link_libraries(sport_mode_adapter_node unitree_sdk2)

install(TARGETS sport_mode_adapter_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
