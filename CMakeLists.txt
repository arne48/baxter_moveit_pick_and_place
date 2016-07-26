cmake_minimum_required(VERSION 2.8.3)
project(baxter_moveit_pick_and_place)

SET(CMAKE_CXX_FLAGS "-std=c++0x")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
             COMPONENTS
             moveit_core
             moveit_ros_planning
             moveit_ros_planning_interface
             pluginlib
             cmake_modules
             geometric_shapes
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pr2_moveit_tutorials
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)
find_package(Eigen REQUIRED)

###########
## Build ##
###########


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(baxter_moveit_pick_and_place src/baxter_moveit_pick_and_place.cpp)
target_link_libraries(baxter_moveit_pick_and_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS baxter_moveit_pick_and_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
