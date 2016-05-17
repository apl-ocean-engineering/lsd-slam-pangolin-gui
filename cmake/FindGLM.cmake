#.rst:
# FindGLM
# --------
#
# Based on Kitware's FindGLM.cmake
#
# try to find GLM library and include files.
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the :prop_tgt:`IMPORTED` targets:
#
# ``GLM::GLM``
#  Defined if the system has GLM.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This module sets the following variables:
#
# ::
#
#   GLM_INCLUDE_DIR, where to find GL/GLM.h, etc.
#   GLM_LIBRARIES, the libraries to link against
#   GLM_FOUND, If false, do not try to use GLM.
#
# Also defined, but not for general use are:
#

#=============================================================================
# Copyright 2001-2009 Kitware, Inc.
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)

# glm is header-only (thankfully)

find_path( GLM_INCLUDE_DIR NAMES glm/glm.hpp
    PATHS  /usr/include
					/usr/local/include )

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GLM REQUIRED_VARS GLM_INCLUDE_DIR)

if( GLM_FOUND )
  mark_as_advanced(GLM_INCLUDE_DIR)
endif()
