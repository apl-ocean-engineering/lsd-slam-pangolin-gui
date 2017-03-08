cmake_minimum_required(VERSION 2.8.7)
project(lsd_slam_conan)
include(ExternalProject)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

## Pull in cmake files from the LSD_SLAM package
list(APPEND CMAKE_MODULE_PATH ${CONAN_LSD_SLAM_ROOT}/../../export/cmake )

find_package(Boost COMPONENTS thread filesystem system REQUIRED)
find_package(Eigen3 REQUIRED)


## This is hackish, needs to be expressed in the conanfile for Pangolin
## Check for dependencies for Pangolin
find_package( GLEW REQUIRED )
find_package( GLUT REQUIRED )
find_package( GLM REQUIRED )  # n.b. we provide the FindGLM.cmake file
find_package( OpenGL REQUIRED )
find_package( JPEG REQUIRED )
find_package( PNG REQUIRED )
find_package( PythonLibs REQUIRED )
find_package( OpenEXR QUIET )

set( Pangolin_LIBRARIES
			${OPENGL_LIBRARIES}
			${JPEG_LIBRARIES}
			${PNG_LIBRARIES}
			${PYTHON_LIBRARIES}
			${GLEW_LIBRARIES}
			${GLUT_LIBRARIES}
			${OpenEXR_LIBRARIES} )

# if(APPLE)
# 	## Pangolin prefers FreeGLUT over the GLUT Framework from XCode
# 	list( APPEND Pangolin_LIBRARIES glut  )
# else()
# 	list( APPEND Pangolin_LIBRARIES    )
# endif()

## This is hackish, needs to be expressed in the conanfile for G2O
find_package(SuiteParse REQUIRED)
list( APPEND G2O_LIBRARIES
  ${CSPARSE_LIBRARY} )
list( APPEND G2O_INCLUDE_DIR
  ${CSPARSE_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR} )

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

list(APPEND CONAN_LIBS ${G2O_LIBRARIES}
			pthread
			${Pangolin_LIBRARIES}
	    ${OpenCV_LIBS}
    	${Boost_LIBRARIES})

include_directories( LSDSLAM_INCLUDE_DIRS
	${CMAKE_SOURCE_DIR}/lib
	${EIGEN3_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
  ${CONAN_INCLUDE_DIRS})

add_subdirectory( lib/ )
add_subdirectory( tools/ )

option( BUILD_UNIT_TESTS "Build unit tests" OFF )
if( BUILD_UNIT_TESTS )
  add_subdirectory( test/unit/ )
endif()
