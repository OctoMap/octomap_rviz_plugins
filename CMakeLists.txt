cmake_minimum_required(VERSION 3.5z)
project(octomap_rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)

set(octomap_rviz_plugins_headers_to_moc
  include/octomap_rviz_plugins/occupancy_grid_display.hpp
  include/octomap_rviz_plugins/occupancy_map_display.hpp
)

foreach(header "${octomap_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(octomap_rviz_plugins_moc_files "${header}")
endforeach()

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${octomap_rviz_plugins_moc_files}
  src/occupancy_grid_display.cpp
  src/occupancy_map_display.cpp
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  ${OCTOMAP_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "OCTOMAP_RVIZ_PLUGINS_BUILDING_LIBRARY")

# Check version of rclcpp to switch between features (message_lost_callback):
#  - Foxy is on rclcpp 2.x
#  - Galactic is on rclcpp 9.x
#  - Rolling (currently) is on rclcpp 14.x
if(${rclcpp_VERSION_MAJOR} VERSION_LESS "3.0.0")
  target_compile_definitions(${PROJECT_NAME} PRIVATE "FOXY")
endif()

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    icons
)


