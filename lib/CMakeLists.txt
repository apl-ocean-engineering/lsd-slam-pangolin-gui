# Set source files
set(lsdgui_SOURCE_FILES
  GUI.cpp
  GuiThread.cpp
  InputThread.cpp
  Pangolin_IOWrapper/PangolinOutput3DWrapper.cpp
)

# build shared library.
add_library( lsdgui STATIC ${lsdgui_SOURCE_FILES})
target_link_libraries( lsdgui lsdslam )
