cmake_minimum_required(VERSION 2.8.7)
project(lsd_slam_core)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

find_package(OpenCV 2.4 REQUIRED)
find_package(Boost COMPONENTS thread filesystem system REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin 0.1 REQUIRED)
find_package(G2O REQUIRED)
find_package(SuiteParse REQUIRED) # Apparently needed by g2o

# FabMap
# uncomment this part to enable fabmap
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
#include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
#add_definitions("-DHAVE_FABMAP")
#set(FABMAP_LIB openFABMAP )

## Enable support for the Zed stereo camera
option( USE_ZED_CAMERA "Use Zed Stereo camera" ON )
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

  # find_package(OpenCV 2.4 COMPONENTS core highgui imgproc REQUIRED)

  #add_subdirectory( "${aplcam_SOURCE_DIR}/third_party/apriltags" )

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

# SSE flags
add_definitions("-DENABLE_SSE")

add_definitions("-DNDEBUG")

# Also add some useful compiler flag
set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} -msse4.1 -msse3 -msse2 -msse -march=native -O3 -std=c++0x"
)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# Set source files
set(lsd_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/DataStructures/Frame.cpp
  ${PROJECT_SOURCE_DIR}/DataStructures/FramePoseStruct.cpp
  ${PROJECT_SOURCE_DIR}/DataStructures/FrameMemory.cpp
  ${PROJECT_SOURCE_DIR}/SlamSystem.cpp
  ${PROJECT_SOURCE_DIR}/GUI.cpp
  ${PROJECT_SOURCE_DIR}/LiveSLAMWrapper.cpp
  ${PROJECT_SOURCE_DIR}/DepthEstimation/DepthMap.cpp
  ${PROJECT_SOURCE_DIR}/DepthEstimation/DepthMapPixelHypothesis.cpp
  ${PROJECT_SOURCE_DIR}/util/globalFuncs.cpp
  ${PROJECT_SOURCE_DIR}/util/SophusUtil.cpp
  ${PROJECT_SOURCE_DIR}/util/settings.cpp
  ${PROJECT_SOURCE_DIR}/util/Undistorter.cpp
  ${PROJECT_SOURCE_DIR}/util/RawLogReader.cpp
  ${PROJECT_SOURCE_DIR}/Tracking/Sim3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/Tracking/Relocalizer.cpp
  ${PROJECT_SOURCE_DIR}/Tracking/SE3Tracker.cpp
  ${PROJECT_SOURCE_DIR}/Tracking/TrackingReference.cpp
  ${PROJECT_SOURCE_DIR}/IOWrapper/Timestamp.cpp
  ${PROJECT_SOURCE_DIR}/GlobalMapping/FabMap.cpp
  ${PROJECT_SOURCE_DIR}/GlobalMapping/KeyFrameGraph.cpp
  ${PROJECT_SOURCE_DIR}/GlobalMapping/g2oTypeSim3Sophus.cpp
  ${PROJECT_SOURCE_DIR}/GlobalMapping/TrackableKeyFrameSearch.cpp
)
set(SOURCE_FILES
  ${lsd_SOURCE_FILES}
  ${PROJECT_SOURCE_DIR}/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
  ${PROJECT_SOURCE_DIR}/IOWrapper/Pangolin/PangolinOutput3DWrapper.cpp
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  ${G2O_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
  ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
  ${ZED_CAMERA_INCLUDE_DIRS}
)

# build shared library.
add_library(lsdslam SHARED ${SOURCE_FILES})
target_link_libraries(lsdslam
  ${FABMAP_LIB}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_TYPES_SBA}
  ${G2O_TYPES_SIM3}
  csparse
  cxsparse )

# build image node
add_executable(LSD LSD.cpp)
target_link_libraries(LSD lsdslam ${Pangolin_LIBRARIES})

if( USE_ZED_CAMERA )
  add_executable(LSD_Zed LSD_Zed.cpp)

  target_link_libraries(LSD_Zed lsdslam
                            ${Pangolin_LIBRARIES}
                            ${ZED_CAMERA_LIBRARIES} )

endif()
