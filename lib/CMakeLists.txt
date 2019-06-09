
fips_begin_module( lsdgui )
  fips_files(
    App/InputThread.cpp
    App/MakeInput.cpp
    GUI.cpp
    TextOutputIOWrapper.cpp
    StateSaver.cpp
    GuiKeyframe.cpp
  )

  fips_deps( lsdslam pangolin videoio ${FIPS_DEPS} )

fips_end_module()
