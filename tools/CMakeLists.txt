
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# set(CMAKE_CXX_FLAGS
#    "${CMAKE_CXX_FLAGS} -ggdb"
# )

link_directories( )

# build image node
add_executable(LSD LSD.cpp
  LSD/GuiThread.cpp
  LSD/InputThread.cpp)
target_link_libraries( LSD lsdslam ${CONAN_LIBS} )
