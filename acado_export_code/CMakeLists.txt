#
# Project settings
#

# Minimum required version of cmake
cmake_minimum_required( VERSION 2.8 )

# Project name and programming languages used
project( acado_export_code CXX )

# CMake module(s) path
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

#
# Prerequisites
#
find_package( ACADO REQUIRED )

#
# Include directories
#
include_directories( . ${ACADO_INCLUDE_DIRS} )

#
# Build an executable
#
add_definitions( -std=c++11 )
add_executable( mpc symbolic_mpc.cpp  )
target_link_libraries( mpc ${ACADO_SHARED_LIBRARIES} )
