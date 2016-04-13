
SET( G3LOG_PREFIX_DIR ${PROJECT_BINARY_DIR}/g3log )
SET( G3LOG_INSTALL_DIR ${G3LOG_PREFIX_DIR} )
SET( G3LOG_SOURCE_DIR ${G3LOG_PREFIX_DIR}/src/g3log )
ExternalProject_Add( g3log
										GIT_REPOSITORY https://github.com/KjellKod/g3log.git
										PREFIX g3log
										CMAKE_CACHE_ARGS -DADD_FATAL_EXAMPLE:bool=OFF
  									INSTALL_COMMAND "" )

										# SOURCE_DIR ${SNAPPY_SOURCE_DIR}
										# BINARY_DIR ${SNAPPY_SOURCE_DIR}
										# UPDATE_COMMAND ""
										# CONFIGURE_COMMAND ./autogen.sh && ./configure --prefix=${SNAPPY_INSTALL_DIR}
										# BUILD_COMMAND make -j -C ${SNAPPY_SOURCE_DIR} )

## g3log doesn't have an "install" target
set( G3LOG_INCLUDE_DIR ${G3LOG_SOURCE_DIR}/src )
set( G3LOG_LIB_DIR ${G3LOG_PREFIX_DIR}/src/g3log-build/ )
link_directories(
  ${G3LOG_LIB_DIR}
)

set( G3LOG_LIB g3logger )
