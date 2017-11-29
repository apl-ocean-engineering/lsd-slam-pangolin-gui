
fips_begin_app( LSD cmdline )

  fips_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

  # build image node
  fips_files( LSD.cpp )
  fips_libs( lsdgui )
  fips_deps( lsdslam )

fips_end_app()
