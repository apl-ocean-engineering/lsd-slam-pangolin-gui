
if( NOT FIPS_IMPORT)
  cmake_minimum_required(VERSION 3.5)
  get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
  include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

  fips_setup(PROJECT fips-lsd-slam)
endif()

find_package(Boost COMPONENTS thread filesystem system REQUIRED)
find_package(Eigen3 REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package( SuiteParse REQUIRED )
find_package( TinyXML2 REQUIRED )

## This is hackish, needs to be expressed in the conanfile for G2O
# find_package(SuiteParse REQUIRED)
# list( APPEND G2O_LIBRARIES
#   ${CSPARSE_LIBRARY} )
# list( APPEND G2O_INCLUDE_DIR
#   ${CSPARSE_INCLUDE_DIR}
#   ${CHOLMOD_INCLUDE_DIR} )

##==== Check and define the architecture
## Architecture-specific optimizations
## Need this workaround for CUDA 8.0
# find_package( CUDA )
# if( CUDA_VERSION )
# 	message("Found CUDA version ${CUDA_VERSION}")
# 	if(${CUDA_VERSION} VERSION_EQUAL "8.0")
# 		message("Disabling CUDA_USE_STATIC_CUDA_RUNTIME")
# 		set( CUDA_USE_STATIC_CUDA_RUNTIME OFF CACHE BOOL "" FORCE  )
# 	endif()
# 	find_package( CUDA )
# endif()

## Configuration options
option( USE_LIBVIDEIO_GO "Enable the Go-based Libvideio" OFF)
if( USE_LIBVIDEIO_GO )
  list( APPEND FIPS_DEPS videoio_go )
  add_definitions( -DVIDEOIO_GO )
endif()


set( OPENCV_COMPONENTS core highgui imgproc video)
find_package(OpenCV COMPONENTS ${OPENCV_COMPONENTS} REQUIRED)

if( OpenCV_VERSION_MAJOR EQUAL "3" )
  message( STATUS "Using OpenCV 3.x")
  add_definitions( -DOPENCV3 )
else()
  message( STATUS "Using OpenCV 2.x")
  add_definitions( -DOPENCV2 )
endif()


EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )
if( ${ARCHITECTURE} STREQUAL "aarch64" )
	## For compilation on Jetson TX1
	set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a") # -DENABLE_NEON" )
elseif( ${ARCHITECTURE} STREQUAL "x86_64" )
  # -DENABLE_SSE enables Jakob's original SSE optimizations in his code,
	# it has nothing to do with compiler behavior
  set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse4.1 -msse3 -msse2 -msse -march=native -DENABLE_SSE" )
endif()

## C++ flags applied to all builds
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

fips_include_directories( LSDSLAM_INCLUDE_DIRS
	${CMAKE_SOURCE_DIR}/lib
	${EIGEN3_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
)

fips_include_directories( third_party/CLI11/ )

fips_add_subdirectory( lib/ )
fips_add_subdirectory( tools/ )

if( FIPS_UNITTESTS )
  fips_add_subdirectory( test/unit/ )

  if( FIPS_UNITTESTS_RUN_AFTER_BUILD )
    ## TODO
  endif()
endif()

if( NOT FIPS_IMPORT )
  fips_finish()
endif()
