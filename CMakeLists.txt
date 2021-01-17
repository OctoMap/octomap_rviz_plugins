cmake_minimum_required(VERSION 3.0.2)
project(octomap_rviz_plugins)

find_package(catkin REQUIRED COMPONENTS octomap_msgs
                                        roscpp
                                        rviz
                                        
)

find_package(octomap REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread )

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(QT_LIBRARIES Qt5::Widgets)

set(CMAKE_AUTOMOC ON)

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS octomap_msgs
                   roscpp
                   rviz
    DEPENDS OCTOMAP
)
                                        

include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    ${Boost_INCLUDE_DIRS} 
                    ${OCTOMAP_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS}
                 ${Boost_LIBRARY_DIRS}
                 ${OCTOMAP_LIBRARY_DIRS}) 


set(SOURCE_FILES
  src/occupancy_grid_display.cpp
  src/occupancy_map_display.cpp 
)

set(HEADER_FILES
  include/octomap_rviz_plugins/occupancy_grid_display.h
  include/octomap_rviz_plugins/occupancy_map_display.h
)

add_library(${PROJECT_NAME} MODULE ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC "-Wno-register") # Avoid OGRE deprecaton warnings under C++17

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


