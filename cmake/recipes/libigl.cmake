## libigl MPL

#
# Copyright 2020 Adobe. All rights reserved.
# This file is licensed to you under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License. You may obtain a copy
# of the License at http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software distributed under
# the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR REPRESENTATIONS
# OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.
#
if(TARGET igl::core)
    return()
endif()

message(STATUS "Third-party: creating target 'igl::core'")

# Options for libigl modules
option(LIBIGL_USE_STATIC_LIBRARY     "Use libigl as static library" OFF)
option(LIBIGL_WITH_COMISO            "Use CoMiso"                   OFF)
option(LIBIGL_WITH_EMBREE            "Use Embree"                   OFF)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"                   OFF)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"                     OFF)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"                    OFF)
option(LIBIGL_WITH_PNG               "Use PNG"                      OFF)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"                   OFF)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"                 OFF)
option(LIBIGL_WITH_PREDICATES        "Use exact predicates"         ON)
option(LIBIGL_WITH_XML               "Use XML"                      OFF)

include(FetchContent)
FetchContent_Declare(
    libigl
    URL https://github.com/libigl/libigl/archive/refs/tags/v2.3.0.zip
    URL_HASH MD5=6f17257bf1c898872091e5095d477cbf
)
FetchContent_GetProperties(libigl)
if(libigl_POPULATED)
    return()
endif()
FetchContent_Populate(libigl)

include(eigen)

set(LIBIGL_WITH_PREDICATES ON CACHE BOOL "Use exact predicates" FORCE)

list(APPEND CMAKE_MODULE_PATH ${libigl_SOURCE_DIR}/cmake)
include(${libigl_SOURCE_DIR}/cmake/libigl.cmake ${libigl_BINARY_DIR})

# Install rules
set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME libigl)
set_target_properties(igl PROPERTIES EXPORT_NAME core)
install(DIRECTORY ${libigl_SOURCE_DIR}/include/igl DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS igl igl_common EXPORT Libigl_Targets)
install(EXPORT Libigl_Targets DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/igl NAMESPACE igl::)
