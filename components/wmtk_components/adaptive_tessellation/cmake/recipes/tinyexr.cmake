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
# Modified:
# 2023-11-16 Daniel Zint (daniel.zint@mail.de)
#
if(TARGET tinyexr::tinyexr)
    return()
endif()

message(STATUS "Third-party (external): creating target 'tinyexr::tinyexr'")

include(CPM)
CPMAddPackage(
    NAME tinyexr
    GITHUB_REPOSITORY syoyo/tinyexr
    GIT_TAG bb751eb
)
FetchContent_MakeAvailable(tinyexr)

# target_sources(tinyexr
#     PUBLIC
#     ${tinyexr_SOURCE_DIR}/tinyexr.h
#     PRIVATE
#     ${tinyexr_SOURCE_DIR}/tinyexr.cc
# )
target_include_directories(tinyexr
    PUBLIC
    ${tinyexr_SOURCE_DIR}
)

# Removes the specified compile flag from the specified target.
#   _target     - The target to remove the compile flag from
#   _flag       - The compile flag to remove
#
# Pre: apply_global_cxx_flags_to_all_targets() must be invoked.
#
macro(remove_flag_from_target _target _flag)
    get_target_property(_target_cxx_flags ${_target} COMPILE_OPTIONS)
    if(_target_cxx_flags)
        list(REMOVE_ITEM _target_cxx_flags ${_flag})
        set_target_properties(${_target} PROPERTIES COMPILE_OPTIONS "${_target_cxx_flags}")
    endif()
endmacro()

# Increase warning level for clang.
IF (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    #target_compile_options(tinyexr PUBLIC -Wno-error)
    remove_flag_from_target(tinyexr -Werror)
ENDIF ()

#target_compile_features(tinyexr PUBLIC cxx_std_17)

add_library(tinyexr::tinyexr ALIAS tinyexr)
