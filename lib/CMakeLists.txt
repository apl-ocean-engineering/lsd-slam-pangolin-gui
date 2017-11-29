
fips_begin_module( lsdgui )
  fips_files(
    GUI.cpp
    LSDArgs.cpp
 )

  fips_dir(Pangolin_IOWrapper)

  fips_files (
    PangolinOutput3DWrapper.cpp
    PangolinOutputIOWrapper.cpp
  )

  fips_deps( lsdslam )

fips_end_module()
