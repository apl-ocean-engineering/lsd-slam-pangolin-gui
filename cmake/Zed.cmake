
if( USE_ZED_CAMERA )
  add_definitions( -DUSE_ZED )

  IF(WIN32) # Windows
    SET(ZED_INCLUDE_DIRS $ENV{ZED_INCLUDE_DIRS})
        if (CMAKE_CL_64) # 64 bits
            SET(ZED_LIBRARIES $ENV{ZED_LIBRARIES_64})
        else(CMAKE_CL_64) # 32 bits
            message("32bits compilation is no more available with CUDA7.0")
        endif(CMAKE_CL_64)
    SET(ZED_LIBRARY_DIR $ENV{ZED_LIBRARY_DIR})
    SET(OPENCV_DIR $ENV{OPENCV_DIR})
    find_package(CUDA 7.0 REQUIRED)
  ELSE() # Linux
    find_package(ZED REQUIRED)
    find_package(CUDA 6.5 REQUIRED)
  ENDIF(WIN32)

  set( ZED_CAMERA_INCLUDE_DIRS ${ZED_INCLUDE_DIRS}
                              ${CUDA_INCLUDE_DIRS} )

  set( ZED_CAMERA_LIBRARY_DIRS ${ZED_LIBRARY_DIR}
                                ${CUDA_LIBRARY_DIRS})

  link_directories( ${ZED_CAMERA_LIBRARY_DIRS} )

  set( ZED_CAMERA_LIBRARIES ${ZED_LIBRARIES}
                            ${CUDA_LIBRARIES}
                            ${CUDA_npps_LIBRARY}
                            ${CUDA_nppi_LIBRARY} )
endif()
