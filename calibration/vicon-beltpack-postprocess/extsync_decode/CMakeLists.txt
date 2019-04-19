cmake_minimum_required(VERSION 2.8)
project(extsync_decode)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

#find_package(Boost COMPONENTS program_options system filesystem REQUIRED)
#find_package(netpbm REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -pedantic")

set(CMAKE_BUILD_TYPE Release)


#include_directories(
#  common
#)


set(extsync_decode_SOURCES
  src/extsync_decode.cc
)

#set(extsync_decode_HEADERS
#  extsync_decode.h
#)

#include_directories("${PROJECT_BINARY_DIR}")
#include_directories(${Boost_INCLUDE_DIR})
#include_directories(${netpbm_INCLUDE_DIR})

add_executable(extsync_decode ${extsync_decode_SOURCES})
#target_link_libraries(mirage ${Boost_PROGRAM_OPTIONS_LIBRARY})
#target_link_libraries(mirage ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})
#target_link_libraries(mirage ${netpbm_LIBRARIES})
#target_link_libraries(mirage ${CMAKE_SOURCE_DIR}/libnetpbm.a)

# Fixed Point Unit Tests
####
#add_executable(fp_unit_tests common/fp_unit_tests.cc)
#set_target_properties(fp_unit_tests PROPERTIES COMPILE_FLAGS "-ftemplate-depth=781")
####


#add_executable(test_header test_header.cc mirage/GenericPort.cc mirage/Port.cc mirage/ClockedUnit.cc)
#target_link_libraries(test_header ${netpbm_LIBRARIES})
