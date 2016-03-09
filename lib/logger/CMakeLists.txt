


if( USE_ZED_CAMERA )
  # set( ZED_SRCS
  #     ${CMAKE_CURRENT_SOURCE_DIR}/util/Undistorter/UndistorterZED.cpp )
endif()


# Set source files
set(logger_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/RawLogReader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/LogWriter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/active.cpp
  # ${ZED_SRCS}
)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  # ${ZED_CAMERA_INCLUDE_DIRS}
  ${G3LOG_INCLUDE_DIR}
)

# build shared library.
add_library(logger SHARED ${logger_SOURCE_FILES})
target_link_libraries(logger
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  # ${ZED_CAMERA_LIBRARIES}
  ${G3LOG_LIB}
)

set_target_properties( logger PROPERTIES COMPILE_FLAGS "-ggdb")
