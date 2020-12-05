cmake_minimum_required(VERSION 3.1)
project(ipopt-pino CXX)
set(CMAKE_CXX_STANDARD 11)

###########
## Build ##
###########
if(NOT CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "WARNING: CMAKE_BUILD_TYPE not set to Release, which can decrease performance significantly")
endif()
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
# find Eigen
find_package(Eigen3 REQUIRED)
# find Boost
find_package(Boost COMPONENTS system filesystem REQUIRED)
# find pinocchio
find_package(pinocchio REQUIRED)
# find OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
# find ifopt
find_package(ifopt)

# build ipopt-pino
add_library(
  ${PROJECT_NAME} 
  SHARED
  src/robot/robot.cpp
  src/robot/floating_base.cpp
  src/variables/configuration.cpp
  src/variables/velocity.cpp
  src/variables/torques.cpp
  src/constraints/initial_state.cpp
  src/constraints/state_equation.cpp
  src/cost/stage_cost.cpp
  src/cost/terminal_cost.cpp
  src/ocp_factory.cpp
)
target_link_libraries(
  ${PROJECT_NAME} 
  PUBLIC 
  ifopt::ifopt_ipopt
  pthread
  ${Boost_LIBRARIES}
  ${PINOCCHIO_LIBRARIES}
  urdfdom_model
)
target_include_directories(
  ${PROJECT_NAME} 
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${PINOCCHIO_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#############
## Install ##
#############
include(GNUInstallDirs)
# Install lib files
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-config
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)
# Install header files
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
# Install config file 
set(config_package_location "share/${PROJECT_NAME}/cmake")
install(
  EXPORT ${PROJECT_NAME}-config
  FILE ${PROJECT_NAME}-config.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${config_package_location}
)
# Install alias
add_library(
  ${PROJECT_NAME}::${PROJECT_NAME} 
  ALIAS ${PROJECT_NAME}
)