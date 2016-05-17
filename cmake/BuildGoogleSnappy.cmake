

if( USE_GOOGLE_SNAPPY )
  add_definitions( -DUSE_SNAPPY )

  SET( SNAPPY_PREFIX_DIR ${PROJECT_BINARY_DIR}/google-snappy )
  SET( SNAPPY_INSTALL_DIR ${SNAPPY_PREFIX_DIR} )
  SET( SNAPPY_SOURCE_DIR ${SNAPPY_PREFIX_DIR}/src/snappy )
  ExternalProject_Add( snappy
                      GIT_REPOSITORY https://github.com/google/snappy.git
                      PREFIX google-snappy
                      SOURCE_DIR ${SNAPPY_SOURCE_DIR}
                      BINARY_DIR ${SNAPPY_SOURCE_DIR}
                      UPDATE_COMMAND ""
                      CONFIGURE_COMMAND ./autogen.sh && ./configure --prefix=${SNAPPY_INSTALL_DIR}
                      BUILD_COMMAND ${EXTERNAL_PROJECT_MAKE_COMMAND} -C ${SNAPPY_SOURCE_DIR} )

  set( SNAPPY_INCLUDE_DIRS ${SNAPPY_INSTALL_DIR}/include )

  ## This doesn't seem right.
  if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set( SNAPPY_LIBRARIES ${SNAPPY_INSTALL_DIR}/lib/libsnappy.dylib )
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set( SNAPPY_LIBRARIES ${SNAPPY_INSTALL_DIR}/lib/libsnappy.so )
  endif()
endif()
