
SET( PANGOLIN_PREFIX_DIR ${PROJECT_BINARY_DIR}/Pangolin )
SET( PANGOLIN_INSTALL_DIR ${PANGOLIN_PREFIX_DIR} )

find_package( GLUT REQUIRED )
find_package( GLM REQUIRED )

ExternalProject_Add( Pangolin
                      GIT_REPOSITORY https://github.com/stevenlovegrove/Pangolin.git
                      PREFIX Pangolin
                      BUILD_COMMAND ${EXTERNAL_PROJECT_MAKE_COMMAND}
                      CMAKE_CACHE_ARGS -DCMAKE_BUILD_TYPE:string=Release
                              -DCMAKE_INSTALL_PREFIX:path=${PANGOLIN_INSTALL_DIR}
                              -DBUILD_EXAMPLES:bool=OFF
                              -DFORCE_GLUT:bool=ON )

set( Pangolin_LIBRARIES
      -L${PANGOLIN_INSTALL_DIR}/lib
      pangolin )

if( ${CMAKE_SYSTEM_NAME} MATCHES "Darwin" )
  LIST( APPEND Pangolin_LIBRARIES "-framework OpenGL")
elseif()
  LIST( APPEND Pangolin_LIBRARIES -lgl )
endif()

set( Pangolin_INCLUDE_DIRS
    ${PANGOLIN_INSTALL_DIR}/include )
