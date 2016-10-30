

IF( DEFINED LOCAL_LIBACTIVE_OBJECT )
	message("Building local copy of LibActiveObject")

	include_directories( ${LOCAL_LIBACTIVE_OBJECT}/include )
	add_subdirectory( ${LOCAL_LIBACTIVE_OBJECT}/lib libactive_object )

	set( LIBACTIVE_OBJECT_INCLUDE_DIRS ${LOCAL_LIBACTIVE_OBJECT}/include )

ELSE()
	message("Building LibActiveObject from Git.")
	SET( LIBACTIVE_OBJECT_PREFIX_DIR ${PROJECT_BINARY_DIR}/libactive_object )
	SET( LIBACTIVE_OBJECT_INSTALL_DIR ${LIBACTIVE_OBJECT_PREFIX_DIR} )
	# SET( G3LOG_SOURCE_DIR ${G3LOG_PREFIX_DIR}/src/g3log )
	#
 SET( LIBACTIVE_OBJECT_CMAKE_OPTS  )
	IF( DEFINED CMAKE_BUILD_TYPE )
		LIST(APPEND LIBACTIVE_OBJECT_CMAKE_OPTS -DCMAKE_BUILD_TYPE:string=${CMAKE_BUILD_TYPE} )
	ENDIF()

	LIST(APPEND LIBACTIVE_OBJECT_CMAKE_OPTS -DBUILD_UNIT_TESTS:bool=false)

	ExternalProject_Add( libactive_object
											GIT_REPOSITORY https://github.com/amarburg/libactive_object
											PREFIX libactive_object
											UPDATE_COMMAND git pull origin master
											BUILD_COMMAND ${EXTERNAL_PROJECT_MAKE_COMMAND}
											CMAKE_ARGS ${LIBACTIVE_OBJECT_CMAKE_OPTS}
	  									INSTALL_COMMAND "" )
	#
	# ## g3log doesn't have an "install" target
	# set( G3LOG_INCLUDE_DIR ${G3LOG_SOURCE_DIR}/src )
	# set( G3LOG_LIB_DIR ${G3LOG_PREFIX_DIR}/src/g3log-build/ )
	# link_directories(
	#   ${G3LOG_LIB_DIR}
	# )
	#
	# set( G3LOG_LIB g3logger )

	set_target_properties(libactive_object PROPERTIES EXCLUDE_FROM_ALL TRUE)
	set( LIBACTIVE_OBJECT_INCLUDE_DIRS ${LIBACTIVE_OBJECT_INSTALL_DIR}/src/libactive_object/include )
	set( LIBACTIVE_OBJECT_LIB_DIR ${LIBACTIVE_OBJECT_INSTALL_DIR}/src/libactive_object-build/lib )

	link_directories(
	  ${LIBACTIVE_OBJECT_LIB_DIR}
	)

	list(APPEND EXTERNAL_PROJECTS libactive_object )

ENDIF()

set( LIBACTIVE_OBJECTS_LIBS active_object )
