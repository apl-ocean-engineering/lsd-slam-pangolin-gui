
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# build image node
add_executable( LSD LSD.cpp )
target_link_libraries( LSD
    ${CONAN_LIBS}
    GLEW
    lsdgui )
