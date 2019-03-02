
fips_begin_app( LSD cmdline )
  # build image node
  fips_files( LSD.cpp )

  if(UNIX AND NOT APPLE)
	  set( ADDL_LIBS uuid )
   endif()


   fips_libs( lsdgui ${ADDL_LIBS}  )
  fips_deps( lsdslam )

fips_end_app()
