
fips_begin_app( LSD cmdline )
  # build image node
  fips_files( LSD.cpp )
  fips_libs( lsdgui )
  fips_deps( lsdslam videoio_go )

fips_end_app()
