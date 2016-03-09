cmake_minimum_required(VERSION 2.8.7)
project(lsd_slam_core)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
set(RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

cmake_minimum_required(VERSION 2.8.7)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(OpenCV 2.4 COMPONENTS core highgui imgproc calib3d REQUIRED)
find_package(Boost COMPONENTS thread filesystem system REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin 0.1 REQUIRED)
find_package(G2O REQUIRED)
find_package(SuiteParse REQUIRED) # Apparently needed by g2o
find_package(TCLAP REQUIRED )

# g3log
set( ADD_FATAL_EXAMPLE OFF )
add_subdirectory( ${PROJECT_SOURCE_DIR}/thirdparty/g3log )
set( G3LOG_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/thirdparty/g3log/src)
set( G3LOG_LIB g3logger_shared )

# FabMap
# uncomment this part to enable fabmap
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
#include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
#add_definitions("-DHAVE_FABMAP")
#set(FABMAP_LIB openFABMAP )

## Enable support for the Zed stereo camera
option( USE_ZED_CAMERA "Use Zed Stereo camera" OFF )
if( USE_ZED_CAMERA )
  add_definitions( -DUSE_ZED )

  IF(WIN32) # Windows
    SET(ZED_INCLUDE_DIRS $ENV{ZED_INCLUDE_DIRS})
        if (CMAKE_CL_64) # 64 bits
            SET(ZED_LIBRARIES $ENV{ZED_LIBRARIES_64})
        else(CMAKE_CL_64) # 32 bits
            message("32bits compilation is no more available with CUDA7.0")
        endif(CMAKE_CL_64)
    SET(ZED_LIBRARY_DIR $ENV{ZED_LIBRARY_DIR})
    SET(OPENCV_DIR $ENV{OPENCV_DIR})
    find_package(CUDA 7.0 REQUIRED)
  ELSE() # Linux
    find_package(ZED REQUIRED)
    find_package(CUDA 6.5 REQUIRED)
  ENDIF(WIN32)

  set( ZED_CAMERA_INCLUDE_DIRS ${ZED_INCLUDE_DIRS}
                              ${CUDA_INCLUDE_DIRS} )

  set( ZED_CAMERA_LIBRARY_DIRS ${ZED_LIBRARY_DIR}
                                ${CUDA_LIBRARY_DIRS})

  link_directories( ${ZED_CAMERA_LIBRARY_DIRS} )

  set( ZED_CAMERA_LIBRARIES ${ZED_LIBRARIES}
                            ${CUDA_LIBRARIES}
                            ${CUDA_npps_LIBRARY}
                            ${CUDA_nppi_LIBRARY} )

endif()

## Architecture detection
EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if( ${ARCHITECTURE} STREQUAL "aarch64" )
  set(CMAKE_CXX_FLAGS
     "${CMAKE_CXX_FLAGS} -O3 -std=c++11"
  )
elseif( ${ARCHITECTURE} STREQUAL "x86_64" )
  # SSE flags
  add_definitions("-DENABLE_SSE")
  set(CMAKE_CXX_FLAGS
     "${CMAKE_CXX_FLAGS} -msse4.1 -msse3 -msse2 -msse -march=native -O3 -std=c++11"
  )
endif()



add_subdirectory("lsd_slam_core/src")

add_subdirectory("tools")
