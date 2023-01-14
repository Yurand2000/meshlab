#############################################################################
# MeshLab                                                           o o     #
# A versatile mesh processing toolbox                             o     o   #
#                                                                _   O  _   #
# Copyright(C) 2005                                                \/)\/    #
# Visual Computing Lab                                            /\/|      #
# ISTI - Italian National Research Council                           |      #
#                                                                    \      #
# All rights reserved.                                                      #
#                                                                           #
# This program is free software; you can redistribute it and/or modify      #
# it under the terms of the GNU General Public License as published by      #
# the Free Software Foundation; either version 2 of the License, or         #
# (at your option) any later version.                                       #
#                                                                           #
# This program is distributed in the hope that it will be useful,           #
# but WITHOUT ANY WARRANTY; without even the implied warranty of            #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             #
# GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          #
# for more details.                                                         #
#                                                                           #
#############################################################################

option(MESHLAB_ALLOW_DOWNLOAD_SOURCE_LIB3MF "Allow download and use of lib3MF source" ON)

if(MESHLAB_ALLOW_DOWNLOAD_SOURCE_LIB3MF)
  set(LIB3MF_DIR ${MESHLAB_EXTERNAL_DOWNLOAD_DIR}/lib3mf-2.2.0)
  set(LIB3MF_CHECK ${LIB3MF_DIR}/CMakeLists.txt) 

  if(NOT EXISTS ${LIB3MF_CHECK})
    set(LIB3MF_LINK https://github.com/3MFConsortium/lib3mf/archive/refs/tags/v2.2.0.zip)
    set(LIB3MF_MD5 31c6dd3e2599f6f32c0784d8f46480bb)
    download_and_unzip(
      NAME "Lib3MF"
      MD5  ${LIB3MF_MD5}
      LINK ${LIB3MF_LINK}
      DIR  ${MESHLAB_EXTERNAL_DOWNLOAD_DIR})
    if(NOT download_and_unzip_SUCCESS)
      message(STATUS "- Lib3MF - download failed")
    endif()
  endif()

  if(EXISTS ${LIB3MF_CHECK})
    message(STATUS " - Lib3MF - using downloaded source")
    set(MESSAGE_QUIET ON)
    set(LIB3MF_TESTS OFF)
    add_subdirectory(${LIB3MF_DIR} EXCLUDE_FROM_ALL)
    unset(MESSAGE_QUIET)
  endif()

  add_library(external-lib3mf ALIAS lib3mf)
  install(TARGETS lib3mf DESTINATION ${MESHLAB_LIB_INSTALL_DIR})

endif()
