cmake_minimum_required(VERSION 2.8.3)
project(leo_path_follower)

find_package(
    catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs move_base_msgs message_generation
    PCL REQUIRED
    Eigen3 REQUIRED NO_MODULE
)

add_message_files(FILES
    Costmap.msg
    CostmapRow.msg
)

##generate messages must be called before catkin_package but after add_message_files
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    LIBRARIES leo_path_follower
    CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

##costmap publisher node

add_executable(
    costmap_publisher_node src/headers.h src/costmap_publisher.cpp
)

target_link_libraries(
    costmap_publisher_node ${catkin_LIBRARIES} ${PCL_LIBRARIES}
)

add_dependencies(costmap_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

##path builder node
add_executable(
    path_builder_node src/path_builder.cpp
)

target_link_libraries(
    path_builder_node ${catkin_LIBRARIES} 
)

add_dependencies(path_builder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})