set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.0.2)
project(ants_paths_follower)

set(CMAKE_BUILD_TYPE Debug)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        geometry_msgs
        common_private_msgs
)

find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        geometry_msgs
        common_private_msgs
)

###########
## Build ##
###########
include_directories(
        ${catkin_INCLUDE_DIRS}
        include/ants_paths_follower
        ${PCL_INCLUDE_DIRS}
        "${PROJECT_SOURCE_DIR}/include"
        /usr/local/include # Location when using 'make system_install'
        /usr/include       # More usual location (e.g. when installing using a package)
)

link_directories(
        /usr/local/lib # Location when using 'make system_install'
        /usr/lib       # More usual location (e.g. when installing using a package)
)
add_library(cs_lib2 src/cubic_spline.cpp)
add_library(control src/control.cpp)


add_executable(ants_paths_follower src/pathFollower.cpp)
target_link_libraries(ants_paths_follower ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(ants_paths_follower common_private_msgs_generate_messages_cpp)

add_executable(rosUdpBridge src/rosUdpBridge.cpp)
target_link_libraries(rosUdpBridge ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(rosUdpBridge common_private_msgs_generate_messages_cpp)

 add_executable(joy_control src/joy_control.cpp)
 target_link_libraries(joy_control ${catkin_LIBRARIES} ${PCL_LIBRARIES})
 add_dependencies(joy_control common_private_msgs_generate_messages_cpp)

add_executable(targetPoint src/targetPoint.cpp)
target_link_libraries(targetPoint ${catkin_LIBRARIES} ${PCL_LIBRARIES} control)
add_dependencies(targetPoint common_private_msgs_generate_messages_cpp)

# add_executable(ants_paths_follower_swarm src/pathFollower_swarm.cpp)
# target_link_libraries(ants_paths_follower_swarm ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(liner_speed_regulator src/liner_speed_regulator.cpp)
# target_link_libraries(liner_speed_regulator cs_lib2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(my_test src/my_test.cpp)
target_link_libraries(my_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(my_test common_private_msgs_generate_messages_cpp)
#############
## Install ##
#############
# install(TARGETS ants_paths_follower
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
# install(DIRECTORY paths/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/paths
# )