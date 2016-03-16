

# Set source files
set(active_object_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/active.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/active_idle.cpp
)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${Boost_INCLUDE_DIR}
)

# build shared library.
add_library(active_object SHARED ${active_object_SOURCE_FILES})
target_link_libraries(active_object
  ${Boost_LIBRARIES}
)
