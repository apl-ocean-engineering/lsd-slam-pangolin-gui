
SET( PANGOLIN_PREFIX_DIR ${PROJECT_BINARY_DIR}/Pangolin )
SET( PANGOLIN_INSTALL_DIR ${PANGOLIN_PREFIX_DIR} )


ExternalProject_Add( Pangolin
                      GIT_REPOSITORY https://github.com/stevenlovegrove/Pangolin.git
                      PREFIX Pangolin
                      BUILD_COMMAND ${EXTERNAL_PROJECT_MAKE_COMMAND}
                      CMAKE_CACHE_ARGS -DCMAKE_BUILD_TYPE:string=Release
                              -DCMAKE_INSTALL_PREFIX:path=${PANGOLIN_INSTALL_DIR}
                              -DBUILD_EXAMPLES:bool=OFF )

set( PANGOLIN_ROOT ${PANGOLIN_INSTALL_DIR} )
