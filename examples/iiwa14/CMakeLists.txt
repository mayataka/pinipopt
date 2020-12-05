cmake_minimum_required(VERSION 3.5)
project(iiwa14 CXX)

set(CMAKE_CXX_STANDARD 11)

find_package(pinipopt REQUIRED)
find_package(ifopt REQUIRED)

add_executable(
  iiwa14
  main.cpp
)
target_link_libraries(
  iiwa14
  PRIVATE
  pinipopt::pinipopt
  ifopt::ifopt_ipopt
  ifopt::ifopt_core
)
target_include_directories(
  iiwa14
  PRIVATE
  ${PINIPOPT_INCLUDE_DIR}
)

# message(STATUS "*** dump start cmake variables ***")
# get_cmake_property(_variableNames VARIABLES)
# foreach(_variableName ${_variableNames})
#         message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()
# message(STATUS "*** dump end ***")