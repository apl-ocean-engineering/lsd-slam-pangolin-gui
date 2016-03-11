
set(GUI_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/IOWrapper/Pangolin/PangolinOutput3DWrapper.cpp
)

if( USE_ZED_CAMERA )
  set( ZED_SRCS
      ${CMAKE_CURRENT_SOURCE_DIR}/util/Undistorter/UndistorterZED.cpp )
endif()

# Set source files
set(lsdslam_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/DataStructures/Frame.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DataStructures/FramePoseStruct.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DataStructures/FrameMemory.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SlamSystem.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GUI.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DepthEstimation/DepthMap.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DepthEstimation/DepthMapPixelHypothesis.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/FileUtils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/globalFuncs.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/SophusUtil.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/settings.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/Undistorter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/Undistorter/UndistorterOpenCV.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/Undistorter/UndistorterPTAM.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/Undistorter/UndistorterLogger.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Tracking/Sim3Tracker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Tracking/Relocalizer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Tracking/SE3Tracker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Tracking/TrackingReference.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/IOWrapper/Timestamp.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GlobalMapping/FabMap.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GlobalMapping/KeyFrameGraph.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GlobalMapping/g2oTypeSim3Sophus.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GlobalMapping/TrackableKeyFrameSearch.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SlamSystem/OptimizationThread.cpp
  ${ZED_SRCS}
  ${GUI_SRCS}
)

set( LSDSLAM_INCLUDE_DIRS
  ${CMAKE_SOURCE_DIR}/lib
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
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

# build shared library.
add_library(lsdslam SHARED ${lsdslam_SOURCE_FILES})
target_link_libraries(lsdslam
  ${FABMAP_LIB}
  ${Pangolin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${ZED_CAMERA_LIBRARIES}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_TYPES_SBA}
  ${G2O_TYPES_SIM3}
  cxsparse
  ${G3LOG_LIB}
  logger
  active_object )

# Not on (my) OS X w/ suitesparse from Homebrew
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  target_link_libraries( lsdslam
                         csparse )
ENDIF()

# build image node
add_executable(LSD LSD.cpp
  LSDGuiThread.cpp
  LSDInputThread.cpp)
target_link_libraries(LSD lsdslam )
