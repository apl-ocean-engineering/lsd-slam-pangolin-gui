
include_directories(
  ../lsd_slam_core/src
  ${TCLAP_INCLUDE_DIR}
  ${G3LOG_INCLUDE_DIR}
  ${ZED_CAMERA_INCLUDE_DIRS}
)

if( USE_ZED_CAMERA )
  add_executable(ZedRecorder ZedRecorder.cpp)
  target_link_libraries(ZedRecorder lsdslam )
endif()
