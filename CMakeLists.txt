cmake_minimum_required(VERSION 3.0.2)
project(comau_ik_solver)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
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
  LIBRARIES comau_nj_370_27_kin comau_nj_220_27_kin comau_nj_370_27_ik_solver comau_nj_220_27_ik_solver
  CATKIN_DEPENDS roscpp ik_solver
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Kinematics libraries

add_library(comau_nj_370_27_kin src/${PROJECT_NAME}/comau_kin.cpp)
set_target_properties(comau_nj_370_27_kin PROPERTIES COMPILE_DEFINITIONS "COMAU_NJ_370_27")
#add_dependencies(comau_nj_370_27_kin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_nj_370_27_kin ${catkin_LIBRARIES})

add_library(comau_nj_220_27_kin src/${PROJECT_NAME}/comau_kin.cpp)
set_target_properties(comau_nj_220_27_kin PROPERTIES COMPILE_DEFINITIONS "COMAU_NJ_220_27")
#add_dependencies(comau_nj_220_27_kin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_nj_220_27_kin ${catkin_LIBRARIES})

# IK Solver

add_library(comau_nj_370_27_ik_solver
  src/${PROJECT_NAME}/comau_ik_solver.cpp
)
add_dependencies(comau_nj_370_27_ik_solver comau_nj_370_27_kin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_nj_370_27_ik_solver
    comau_nj_370_27_kin
    ${catkin_LIBRARIES}
)

add_library(comau_nj_220_27_ik_solver
  src/${PROJECT_NAME}/comau_ik_solver.cpp
)
add_dependencies(comau_nj_220_27_ik_solver comau_nj_220_27_kin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_nj_220_27_ik_solver
    comau_nj_220_27_kin
    ${catkin_LIBRARIES}
)


# Nodes

#add_executable(test_node src/test.cpp)
#add_dependencies(test_node comau_nj_370_27_ik_solver comau_nj_370_27_kin)
#target_link_libraries(test_node
#  comau_nj_370_27_kin
#  comau_nj_370_27_ik_solver
#  ${catkin_LIBRARIES}
#)

add_executable(comau_nj_370_27_ik_solver_node src/node.cpp)
add_dependencies(comau_nj_370_27_ik_solver_node comau_nj_370_27_ik_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_nj_370_27_ik_solver_node
  comau_nj_370_27_kin
  comau_nj_370_27_ik_solver
  ${catkin_LIBRARIES}
)

add_executable(comau_nj_220_27_ik_solver_node src/node.cpp)
add_dependencies(comau_nj_220_27_ik_solver_node comau_nj_220_27_ik_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(comau_nj_220_27_ik_solver_node
  comau_nj_220_27_kin
  comau_nj_220_27_ik_solver
  ${catkin_LIBRARIES}
)
