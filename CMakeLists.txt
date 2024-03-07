cmake_minimum_required(VERSION 3.8)
project(comau_ik_solver)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ROS Version identification
set(ROS_VERSION $ENV{ROS_VERSION})
if(ROS_VERSION STREQUAL "")
  message(FATAL_ERROR "No supported version of ROS found!")
endif()
if(NOT((ROS_VERSION EQUAL 1) OR (ROS_VERSION EQUAL 2)))
  message(FATAL_ERROR "No supported version of ROS found!")
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

if(ROS_VERSION EQUAL 1) # ROS 1
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ik_solver
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
elseif(ROS_VERSION EQUAL 2) # ROS 2
  find_package(ament_cmake REQUIRED)
  find_package(ik_solver REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(pluginlib REQUIRED)
endif()

#############
## TARGETS ##
#############

# Kinematics libraries
add_library(comau_nj_370_27_kin SHARED include/comau_ik_solver/comau_kin.h src/${PROJECT_NAME}/comau_kin.cpp)
target_compile_definitions(comau_nj_370_27_kin PUBLIC COMAU_NJ_370_27)

add_library(comau_nj_220_27_kin SHARED include/comau_ik_solver/comau_kin.h src/${PROJECT_NAME}/comau_kin.cpp)
target_compile_definitions(comau_nj_220_27_kin PUBLIC COMAU_NJ_220_27)

add_library(comau_nj_generic_kin SHARED include/comau_ik_solver/comau_kin.h src/${PROJECT_NAME}/comau_kin.cpp)
target_compile_definitions(comau_nj_generic_kin PUBLIC COMAU_NJ_GENERIC)

# IK Solver
add_library(comau_nj_370_27_ik_solver SHARED include/comau_ik_solver/comau_kin.h src/${PROJECT_NAME}/comau_ik_solver.cpp)
target_compile_definitions(comau_nj_370_27_ik_solver PUBLIC -DROS_X=${ROS_VERSION} -DCOMAU_NJ_370_27)

add_library(comau_nj_220_27_ik_solver SHARED include/comau_ik_solver/comau_kin.h src/${PROJECT_NAME}/comau_ik_solver.cpp)
target_compile_definitions(comau_nj_220_27_ik_solver PUBLIC -DROS_X=${ROS_VERSION} -DCOMAU_NJ_220_27)

add_library(comau_nj_generic_ik_solver SHARED include/comau_ik_solver/comau_kin.h src/${PROJECT_NAME}/comau_ik_solver.cpp)
target_compile_definitions(comau_nj_generic_ik_solver PUBLIC -DROS_X=${ROS_VERSION} -DCOMAU_NJ_GENERIC)

##########
## LINK ##
##########

if(ROS_VERSION EQUAL 1) # ROS 1

  target_link_libraries(comau_nj_370_27_kin ${catkin_LIBRARIES})
  target_link_libraries(comau_nj_220_27_kin ${catkin_LIBRARIES})
  target_link_libraries(comau_nj_generic_kin ${catkin_LIBRARIES})

  add_dependencies(comau_nj_370_27_ik_solver comau_nj_370_27_kin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(comau_nj_370_27_ik_solver comau_nj_370_27_kin ${catkin_LIBRARIES})
  add_dependencies(comau_nj_220_27_ik_solver comau_nj_220_27_kin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(comau_nj_220_27_ik_solver comau_nj_220_27_kin ${catkin_LIBRARIES})
  add_dependencies(comau_nj_generic_ik_solver comau_nj_generic_kin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(comau_nj_generic_ik_solver comau_nj_generic_kin ${catkin_LIBRARIES})

elseif(ROS_VERSION EQUAL 2) # ROS 2

  ## Kinematics
  target_link_libraries(comau_nj_370_27_kin PUBLIC Eigen3::Eigen)
  target_include_directories(comau_nj_370_27_kin PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )

  target_link_libraries(comau_nj_220_27_kin PUBLIC Eigen3::Eigen)
  target_include_directories(comau_nj_220_27_kin PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )

  target_link_libraries(comau_nj_generic_kin PUBLIC Eigen3::Eigen)
  target_include_directories(comau_nj_generic_kin PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )

  ## Plugins
  add_dependencies(comau_nj_370_27_ik_solver comau_nj_370_27_kin)
  ament_target_dependencies(comau_nj_370_27_ik_solver PUBLIC ik_solver pluginlib rclcpp)
  target_link_libraries(comau_nj_370_27_ik_solver PUBLIC comau_nj_370_27_kin)
  target_include_directories(comau_nj_370_27_ik_solver PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )

  add_dependencies(comau_nj_220_27_ik_solver comau_nj_220_27_kin)
  ament_target_dependencies(comau_nj_220_27_ik_solver PUBLIC ik_solver pluginlib rclcpp)
  target_link_libraries(comau_nj_220_27_ik_solver PUBLIC comau_nj_220_27_kin)
  target_include_directories(comau_nj_220_27_ik_solver PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )

  add_dependencies(comau_nj_generic_ik_solver comau_nj_generic_kin)
  ament_target_dependencies(comau_nj_generic_ik_solver PUBLIC ik_solver pluginlib rclcpp)
  target_link_libraries(comau_nj_generic_ik_solver PUBLIC comau_nj_generic_kin)
  target_include_directories(comau_nj_generic_ik_solver PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    )

  pluginlib_export_plugin_description_file(ik_solver plugin2.xml)

endif()

#############
## INSTALL ##
#############

if(ROS_VERSION EQUAL 2)
  install(TARGETS comau_nj_370_27_ik_solver
                  comau_nj_220_27_ik_solver
                  comau_nj_generic_ik_solver
          EXPORT export_${PROJECT_NAME}
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          RUNTIME DESTINATION bin)

  install(TARGETS comau_nj_370_27_kin
                  comau_nj_220_27_kin
                  comau_nj_generic_kin
          EXPORT export_${PROJECT_NAME}_kin
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          RUNTIME DESTINATION bin)

  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

  install(DIRECTORY include
    DESTINATION include/${PROJECT_NAME})

  ament_export_targets(export_${PROJECT_NAME} export_${PROJECT_NAME}_kin)
  ament_export_dependencies(${PROJECT_NAME} pluginlib ik_solver)
  ament_package()
endif()

##########
## TEST ##
##########
if(FALSE)
  # NOT UPDATED
  add_executable(comau_nj_370_27_ik_solver_node src/node.cpp)
  set_target_properties(comau_nj_370_27_ik_solver_node PROPERTIES COMPILE_DEFINITIONS "COMAU_NJ_370_27")
  add_dependencies(comau_nj_370_27_ik_solver_node comau_nj_370_27_ik_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(comau_nj_370_27_ik_solver_node
    comau_nj_370_27_kin
    comau_nj_370_27_ik_solver
    ${catkin_LIBRARIES}
  )

  add_executable(comau_nj_220_27_ik_solver_node src/node.cpp)
  set_target_properties(comau_nj_220_27_ik_solver_node PROPERTIES COMPILE_DEFINITIONS "COMAU_NJ_220_27")
  add_dependencies(comau_nj_220_27_ik_solver_node comau_nj_220_27_ik_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(comau_nj_220_27_ik_solver_node
    comau_nj_220_27_kin
    comau_nj_220_27_ik_solver
    ${catkin_LIBRARIES}
  )

  add_executable(comau_nj_generic_solver_node src/node.cpp)
  set_target_properties(comau_nj_generic_solver_node PROPERTIES COMPILE_DEFINITIONS "COMAU_NJ_GENERIC")
  add_dependencies(comau_nj_generic_solver_node comau_nj_generic_ik_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(comau_nj_generic_solver_node
    comau_nj_generic_kin
    comau_nj_generic_ik_solver
    ${catkin_LIBRARIES}
  )
endif()

