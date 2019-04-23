
fips_begin_app( LSD cmdline )
  # build image node
  fips_files( LSD.cpp )
  fips_libs( lsdgui )
  fips_deps( lsdslam )

fips_end_app()

fips_begin_app( LSD_stereo cmdline )
  # build image node
  fips_files( LSD_stereo.cpp )
  fips_libs( lsdgui ${YAML_CPP_LIBRARIES} )
  fips_deps( lsdslam )

fips_end_app()
