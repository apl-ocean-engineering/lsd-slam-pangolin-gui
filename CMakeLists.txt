cmake_minimum_required(VERSION 2.8.7)
project(lsd_slam_core)
include(ExternalProject)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
set(RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

cmake_minimum_required(VERSION 2.8.7)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(OpenCV 2.4 COMPONENTS core highgui imgproc calib3d REQUIRED)
find_package(Boost COMPONENTS thread filesystem system REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(TCLAP REQUIRED )
find_package(ZLIB REQUIRED)

include( ${PROJECT_SOURCE_DIR}/cmake/BuildG3Log.cmake )

## Command used to build the external projects
set( EXTERNAL_PROJECT_MAKE_COMMAND make )   # make -j

# FabMap
# uncomment this part to enable fabmap
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
#include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
#add_definitions("-DHAVE_FABMAP")
#set(FABMAP_LIB openFABMAP )

option( BUILD_LOCAL_G2O "Build a local copy of G2O" ON )
if( BUILD_LOCAL_G2O )
	option( G2O_USE_OPENMP "Configure local build of G2O to use OpenMP" ON )

	include( ${PROJECT_SOURCE_DIR}/cmake/BuildG2O.cmake )
elseif()
	find_package(G2O REQUIRED)
	find_package(SuiteParse REQUIRED) # Apparently needed by g2o

	# These are the G2O libraries I use
	set( G2O_LIBRARIES
		${G2O_CORE_LIBRARY}
		${G2O_STUFF_LIBRARY}
		${G2O_SOLVER_CSPARSE_EXTENSION}
		${G2O_SOLVER_CSPARSE}
		${G2O_TYPES_SBA}
		${G2O_TYPES_SIM3}
		cxsparse )
endif()


option( BUILD_LOCAL_PANGOLIN "Build a local copy of Pangolin" ON )
if( BUILD_LOCAL_PANGOLIN )
	include( ${PROJECT_SOURCE_DIR}/cmake/BuildPangolin.cmake )
elseif()
	find_package(Pangolin 0.1 REQUIRED)
endif()


## Enable support for the Zed stereo camera
option( USE_ZED_CAMERA "Use Zed Stereo camera" OFF )
include( ${PROJECT_SOURCE_DIR}/cmake/Zed.cmake )

option( USE_GOOGLE_SNAPPY "Use Google Snappy" ON )
include( ${PROJECT_SOURCE_DIR}/cmake/BuildGoogleSnappy.cmake )

## Architecture detection
EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if( ${ARCHITECTURE} STREQUAL "aarch64" )
  set(CMAKE_CXX_FLAGS
      ## Unclear how many of these command line options are actually effective
      ## Args current for gcc 4.8.4
     "${CMAKE_CXX_FLAGS} -march=armv8-a -O3 -std=c++11"
  )
elseif( ${ARCHITECTURE} STREQUAL "x86_64" )
  # SSE flags
  add_definitions("-DENABLE_SSE")
  # set(CMAKE_CXX_FLAGS
  #    "${CMAKE_CXX_FLAGS} -msse4.1 -msse3 -msse2 -msse -march=native -O3 -std=c++11"
  # )
  set(CMAKE_CXX_FLAGS
     "${CMAKE_CXX_FLAGS} -msse4.1 -msse3 -msse2 -msse -O3 -march=native -ggdb -std=c++11"
  )
endif()


## Set global include dirs
set( LSDSLAM_INCLUDE_DIRS
  ${CMAKE_SOURCE_DIR}/lib
  ${CMAKE_SOURCE_DIR}/lib/lsd_core
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  ${G2O_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
  ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
  ${ZED_CAMERA_INCLUDE_DIRS}
  ${G3LOG_INCLUDE_DIR}
)
include_directories( ${LSDSLAM_INCLUDE_DIRS} )


add_subdirectory("lib/active_object")
add_subdirectory("lib/logger")
add_subdirectory("lib/lsd_core")

add_subdirectory("tools")

option( BUILD_UNIT_TESTS "Build unit tests" ON )
if( BUILD_UNIT_TESTS )
  ## Build a local copy of gtest (as per the GTest FAQ)
  include( ${CMAKE_SOURCE_DIR}/cmake/BuildGTestLibraries.cmake )

  add_subdirectory( test/unit/active_object )
endif()
