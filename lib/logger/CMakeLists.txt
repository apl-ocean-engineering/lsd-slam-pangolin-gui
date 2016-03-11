
# Set source files
set(logger_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/LogReader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/LogWriter.cpp
)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/lib
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${G3LOG_INCLUDE_DIR}
  ${SNAPPY_INCLUDE_DIRS}
)

# build shared library.
add_library(logger SHARED ${logger_SOURCE_FILES})
target_link_libraries(logger
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  # ${ZED_CAMERA_LIBRARIES}
  ${G3LOG_LIB}
  ${SNAPPY_LIBRARIES}
  active_object
)
