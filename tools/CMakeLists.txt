
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# set(CMAKE_CXX_FLAGS
#    "${CMAKE_CXX_FLAGS} -ggdb"
# )

if( USE_ZED_CAMERA )
add_executable(ZedRecorder ZedRecorder.cpp)
target_link_libraries(ZedRecorder lsdslam logger )
set_target_properties( ZedRecorder PROPERTIES COMPILE_FLAGS "-ggdb")
endif()


# build image node
add_executable(LSD LSD.cpp
  LSDGuiThread.cpp
  LSDInputThread.cpp)
target_link_libraries(LSD lsdslam )
