gtest_begin( sdgui )

  fips_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

  # build image node
  fips_files( test_test.cpp )
  fips_libs( lsdgui )
  fips_deps( lsdslam  )

gtest_end()

# include_directories( ${GTEST_INCLUDE_DIRS}
#                      ${CMAKE_SOURCE_DIR}/lib )
#
# fips_add_executable( lsdslam_test
#                     test_test.cpp )
#
# target_link_libraries( lsdslam_test
#     ${CONAN_LIBS}
#         lsdgui )

# # # Don't use ctest, instead just execture the Google test binaries
# add_custom_target( unit_test
#                 COMMAND ./lsdslam_test
#                 WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/bin
#                     DEPENDS lsdslam_test )
