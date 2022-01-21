cmake_minimum_required(VERSION 3.0.2)
project(cartesian_planner)

add_compile_options(-std=c++11 -O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
)

find_package(Eigen3 REQUIRED NO_MODULE)

add_message_files(
  FILES
  CenterLinePoint.msg
  CenterLine.msg
  DynamicObstacle.msg
  DynamicObstacles.msg
  Obstacles.msg
  DynamicTrajectoryPoint.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cartesian_planner
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

option(VISUALIZATION "Compile Visualization Code" ON)

set(SRC
  src/cartesian_planner/math/aabox2d.cpp
  src/cartesian_planner/math/box2d.cpp
  src/cartesian_planner/math/line_segment2d.cpp
  src/cartesian_planner/math/math_utils.cpp
  src/cartesian_planner/math/polygon2d.cpp
  src/cartesian_planner/math/vec2d.cpp
  src/cartesian_planner/cartesian_planner.cpp
  src/cartesian_planner/discretized_trajectory.cpp
  src/cartesian_planner/environment.cpp
  src/cartesian_planner/dp_planner.cpp
  src/cartesian_planner/trajectory_nlp.cpp
  src/cartesian_planner/trajectory_optimizer.cpp)

if(VISUALIZATION)
  add_definitions(-DROS1_VISUALIZATION)
  list(APPEND SRC
    src/cartesian_planner/visualization/plot.cpp
    src/cartesian_planner/visualization/color.cpp)
endif()

add_library(${PROJECT_NAME} ${SRC})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen casadi)

add_executable(${PROJECT_NAME}_node src/cartesian_planner_node.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)
