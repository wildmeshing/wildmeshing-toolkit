#
# Copyright 2021 Adobe. All rights reserved.
# This file is licensed to you under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License. You may obtain a copy
# of the License at http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software distributed under
# the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR REPRESENTATIONS
# OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.
#
if(TARGET Tracy::TracyClient)
    return()
endif()

message(STATUS "Third-party: creating target 'Tracy::TracyClient'")

option(TRACY_ENABLE "Enable profiling with Tracy" OFF)

include(FetchContent)
FetchContent_Declare(
    tracy
    GIT_REPOSITORY https://github.com/wolfpld/tracy.git
    GIT_TAG  v0.9.1
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(tracy)

################################################################################
# Global flags
################################################################################

function(tracy_filter_flags flags)
    include(CheckCXXCompilerFlag)
    set(output_flags)
    foreach(FLAG IN ITEMS ${${flags}})
        string(REPLACE "=" "-" FLAG_VAR "${FLAG}")
        if(NOT DEFINED IS_SUPPORTED_${FLAG_VAR})
            check_cxx_compiler_flag("${FLAG}" IS_SUPPORTED_${FLAG_VAR})
        endif()
        if(IS_SUPPORTED_${FLAG_VAR})
            list(APPEND output_flags ${FLAG})
        endif()
    endforeach()
    set(${flags} ${output_flags} PARENT_SCOPE)
endfunction()

if(TRACED_ENABLE)
    set(TRACY_GLOBAL_FLAGS
        "-fno-omit-frame-pointer"
    )
    tracy_filter_flags(TRACY_GLOBAL_FLAGS)
    message(STATUS "Adding global flags: ${TRACY_GLOBAL_FLAGS}")
    # add_compile_options(${TRACY_GLOBAL_FLAGS})
    target_compile_options(TracyClient PUBLIC ${TRACY_GLOBAL_FLAGS})
endif()

set_target_properties(TracyClient PROPERTIES FOLDER third_party)