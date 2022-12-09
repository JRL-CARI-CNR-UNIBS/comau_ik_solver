cmake_minimum_required(VERSION 3.0.2)
project(comau_ik_solver)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(catkin REQUIRED COMPONENTS
  roscpp ik_solver
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES comau_ik_solver
  CATKIN_DEPENDS roscpp ik_solver
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(comau_ik_solver
  src/${PROJECT_NAME}/comau_kin.cpp
  src/${PROJECT_NAME}/comau_ik_solver.cpp
)
add_dependencies(comau_ik_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_ik_solver
    ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_node src/node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
