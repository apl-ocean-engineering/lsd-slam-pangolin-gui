
include_directories(
  ../lsd_slam_core/src
  ../lib
  ${TCLAP_INCLUDE_DIR}
  ${G3LOG_INCLUDE_DIR}
  ${ZED_CAMERA_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} -ggdb"
)

add_executable(ZedRecorder ZedRecorder.cpp)
target_link_libraries(ZedRecorder lsdslam logger )
set_target_properties( ZedRecorder PROPERTIES COMPILE_FLAGS "-ggdb")
