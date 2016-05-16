
SET( G2O_PREFIX_DIR ${PROJECT_BINARY_DIR}/G2O )
SET( G2O_INSTALL_DIR ${G2O_PREFIX_DIR} )

ExternalProject_Add( G2O
              GIT_REPOSITORY https://github.com/RainerKuemmerle/g2o.git
              PREFIX G2O
              BUILD_COMMAND ${EXTERNAL_PROJECT_MAKE_COMMAND}
              CMAKE_CACHE_ARGS  -DCMAKE_BUILD_TYPE:string=Release
                      -DCMAKE_INSTALL_PREFIX:path=${G2O_INSTALL_DIR}
                      -DG2O_BUILD_APPS:bool=OFF
                      -DG2O_BUILD_EXAMPLES:bool=OFF
                      -DBUILD_CSPARSE:bool=OFF
                      -DG2O_USE_OPENMP:bool=${G2O_USE_OPENMP})

set( G2O_LIBRARIES
		-L${G2O_INSTALL_DIR}/lib
		g2o_core
		g2o_stuff
		g2o_csparse_extension
		g2o_solver_csparse
		g2o_types_sba
		g2o_types_sim3 )
