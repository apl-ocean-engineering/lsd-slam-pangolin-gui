
SET( G3LOG_PREFIX_DIR ${PROJECT_BINARY_DIR}/g3log )
SET( G3LOG_INSTALL_DIR ${G3LOG_PREFIX_DIR} )
SET( G3LOG_SOURCE_DIR ${G3LOG_PREFIX_DIR}/src/g3log )

## Uses my fork which doesn't create src/g3log/generated_definitions.hpp
## And thus doesn't need to be re-built every time...
ExternalProject_Add( g3log
										GIT_REPOSITORY https://github.com/amarburg/g3log.git
										PREFIX g3log
										BUILD_COMMAND ${EXTERNAL_PROJECT_MAKE_COMMAND}
										CMAKE_CACHE_ARGS -DADD_FATAL_EXAMPLE:bool=OFF
  									INSTALL_COMMAND "" )

## g3log doesn't have an "install" target
set( G3LOG_INCLUDE_DIR ${G3LOG_SOURCE_DIR}/src )
set( G3LOG_LIB_DIR ${G3LOG_PREFIX_DIR}/src/g3log-build/ )
link_directories(
  ${G3LOG_LIB_DIR}
)

set( G3LOG_LIB g3logger )
