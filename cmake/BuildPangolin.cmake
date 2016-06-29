
SET( PANGOLIN_PREFIX_DIR ${PROJECT_BINARY_DIR}/Pangolin )
SET( PANGOLIN_INSTALL_DIR ${PANGOLIN_PREFIX_DIR} )

## Check for dependencies (Pangolin doesn't seem do this very well)
find_package( GLEW REQUIRED )
find_package( GLUT REQUIRED )
find_package( GLM REQUIRED )  # n.b. we provide the FindGLM.cmake file
find_package( OpenGL REQUIRED )
find_package( JPEG REQUIRED )
find_package( PNG REQUIRED )
find_package( PythonLibs REQUIRED )
find_package( OpenEXR QUIET )

ExternalProject_Add( Pangolin
                      GIT_REPOSITORY https://github.com/amarburg/Pangolin.git
                      PREFIX Pangolin
                      BUILD_COMMAND ${EXTERNAL_PROJECT_MAKE_COMMAND}
                      CMAKE_CACHE_ARGS -DCMAKE_BUILD_TYPE:string=Release
                              -DCMAKE_INSTALL_PREFIX:path=${PANGOLIN_INSTALL_DIR}
                              -DBUILD_EXAMPLES:bool=OFF
                              -DBUILD_SHARED_LIBS:bool=OFF
                              -DBUILD_PANGOLIN_VIDEO:bool=OFF
                              -DFORCE_GLUT:bool=ON )

set( Pangolin_LIBRARIES
      -L${PANGOLIN_INSTALL_DIR}/lib
      pangolin
      ${OPENGL_LIBRARIES}
      ${JPEG_LIBRARIES}
      ${PNG_LIBRARIES}
      ${PYTHON_LIBRARIES}
      ${GLUT_LIBRARIES}
      ${GLEW_LIBRARIES}
      ${OpenEXR_LIBRARIES})

set( Pangolin_INCLUDE_DIRS
    ${PANGOLIN_INSTALL_DIR}/include )


set_target_properties(Pangolin PROPERTIES EXCLUDE_FROM_ALL TRUE)
