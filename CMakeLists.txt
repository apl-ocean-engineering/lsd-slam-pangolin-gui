cmake_minimum_required(VERSION 2.8.7)
project(lsd_slam_conan)
include(ExternalProject)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

find_package(Boost COMPONENTS thread filesystem system REQUIRED)
find_package(Eigen3 REQUIRED)

##====  Build local copy of Pangolin? ======================
option( BUILD_LOCAL_PANGOLIN "Build a local copy of Pangolin" ON )
if( BUILD_LOCAL_PANGOLIN )
	list( APPEND EXTERNAL_PROJECTS Pangolin )
	include( ${PROJECT_SOURCE_DIR}/cmake/BuildPangolin.cmake )
elseif()
	find_package(Pangolin 0.1 REQUIRED)
endif()

##==== Check and define the architecture
## Architecture-specific optimizations
## Need this workaround for CUDA 8.0
find_package( CUDA )
if( CUDA_VERSION )
	message("Found CUDA version ${CUDA_VERSION}")
	if(${CUDA_VERSION} VERSION_EQUAL "8.0")
		message("Disabling CUDA_USE_STATIC_CUDA_RUNTIME")
		set( CUDA_USE_STATIC_CUDA_RUNTIME OFF CACHE BOOL "" FORCE  )
	endif()
	find_package( CUDA )
endif()



EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )
if( ${ARCHITECTURE} STREQUAL "aarch64" )
	# Jetson version uses OpenCV4Tegra from NVidia
	find_package(OpenCV 2.4 COMPONENTS core highgui imgproc calib3d REQUIRED)

	## For compilation on Jetson TX1
	set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a") # -DENABLE_NEON" )

elseif( ${ARCHITECTURE} STREQUAL "x86_64" )
	# if( ZED_VERSION VERSION_EQUAL "1.0.0")
	# 	find_package(OpenCV 3.1 COMPONENTS core highgui imgproc calib3d videoio imgcodecs REQUIRED)
	# else()
		find_package(OpenCV 2.4 COMPONENTS core highgui imgproc calib3d REQUIRED)
#	endif()

	# add_definitions( -DZED_1_0 )

  # -DENABLE_SSE enables Jakob's original SSE optimizations in his code,
	# it has nothing to do with compiler behavior
  set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse4.1 -msse3 -msse2 -msse -march=native -DENABLE_SSE" )
endif()


## C++ flags applied to all builds
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


set( LSDSLAM_INCLUDE_DIRS
	${EIGEN3_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${Pangolin_INCLUDE_DIRS}
	${G2O_INCLUDE_DIR}
  ${CONAN_INCLUDE_DIRS})

include_directories( ${LSDSLAM_INCLUDE_DIRS} )

# link_directories( ${ZED_LIBRARY_DIRS}
#  									${CUDA_LIBRARY_DIRS} )

#add_subdirectory( lib/ )
add_subdirectory( tools/ )

option( BUILD_UNIT_TESTS "Build unit tests" OFF )
if( BUILD_UNIT_TESTS )
  ## Build a local copy of gtest (as per the GTest FAQ)
	include( ${CMAKE_SOURCE_DIR}/cmake/BuildGTestLibraries.cmake )
	list( APPEND EXTERNAL_PROJECTS gtest )

  add_subdirectory( test/unit/ )
endif()
