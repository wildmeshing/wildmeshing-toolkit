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
    GIT_TAG 79e5df59a97c8aa29172b155897734cdb4ee7b81
)
FetchContent_MakeAvailable(tinyexr)

target_include_directories(tinyexr
    SYSTEM
    PUBLIC
    ${tinyexr_SOURCE_DIR}
)

add_library(tinyexr::tinyexr ALIAS tinyexr)
