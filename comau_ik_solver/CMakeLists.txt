cmake_minimum_required(VERSION 3.0.2)
project(comau_ik_solver)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(catkin REQUIRED COMPONENTS roscpp ik_solver_core)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  comau_nj_ik_solver
  comau_nj_kin
  CATKIN_DEPENDS
  roscpp
  ik_solver_core)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Kinematics libraries
add_library(comau_nj_kin src/${PROJECT_NAME}/comau_kin.cpp)
target_link_libraries(comau_nj_kin ${catkin_LIBRARIES})

# IK Solver
add_library(comau_nj_ik_solver src/${PROJECT_NAME}/comau_ik_solver.cpp)
add_dependencies(comau_nj_ik_solver comau_nj_kin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_nj_ik_solver comau_nj_kin ${catkin_LIBRARIES})

# Nodes
add_executable(comau_nj_solver_node src/node.cpp)
add_dependencies(comau_nj_solver_node comau_nj_ik_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_nj_solver_node comau_nj_ik_solver ${catkin_LIBRARIES})
