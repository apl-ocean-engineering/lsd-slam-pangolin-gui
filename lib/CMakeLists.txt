
fips_begin_module( lsdgui )
  fips_files(
    App/InputThread.cpp
    App/StereoInputThread.cpp
    GUI.cpp
    Input.cpp
   )

  fips_dir(Pangolin_IOWrapper)

  fips_files (
    PangolinOutputIOWrapper.cpp
    TextOutputIOWrapper.cpp
  )

  fips_deps( lsdslam pangolin videoio ${FIPS_DEPS} )

fips_end_module()
