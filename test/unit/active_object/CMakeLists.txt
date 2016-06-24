include_directories( ${GTEST_INCLUDE_DIRS}
                     ${CMAKE_SOURCE_DIR}/lib )

add_executable( active_object_test
              active_idle_test.cpp )

link_directories( ${GTEST_LIBRARY_DIR} )

target_link_libraries( active_object_test
          ${GTEST_BOTH_LIBRARIES}
          active_object )

# Don't use ctest, instead just execture the Google test binaries
add_custom_target( unit_test ${PROJECT_BINARY_DIR}/active_object_test
                    DEPENDS active_object_test )
