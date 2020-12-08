cmake_minimum_required(VERSION 3.5)
project(iiwa14 CXX)

set(CMAKE_CXX_STANDARD 11)

find_package(pinipopt REQUIRED)
find_package(ifopt REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)
pkg_check_modules(COINHSL REQUIRED coinhsl)
# pkg_check_modules(COINMUMPS REQUIRED coinmumps)

link_directories(
  /usr/local/lib
)
add_executable(
  iiwa14
  main.cpp
)
target_link_libraries(
  iiwa14
  PRIVATE
  pinipopt::pinipopt
  ifopt::ifopt_core
  ifopt::ifopt_ipopt
  ${IPOPT_LIBRARIES}
  ${COINHSL_LIBRARIES}
  # ${COINMUMPS_LIBRARIES}
)
target_include_directories(
  iiwa14
  PRIVATE
  ${IPOPT_INCLUDE_DIRS}
  ${PINIPOPT_INCLUDE_DIRS}
)