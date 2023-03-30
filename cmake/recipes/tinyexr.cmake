# ###########################################################################
# ADOBE CONFIDENTIAL
# ___________________
# Copyright 2022 Adobe
# All Rights Reserved.
# * NOTICE:  All information contained herein is, and remains
# the property of Adobe and its suppliers, if any. The intellectual
# and technical concepts contained herein are proprietary to Adobe
# and its suppliers and are protected by all applicable intellectual
# property laws, including trade secret and copyright laws.
# Dissemination of this information or reproduction of this material
# is strictly forbidden unless prior written permission is obtained
# from Adobe.
# ###########################################################################

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
if(TARGET tinyexr::tinyexr)
    return()
endif()

message(STATUS "Third-party (external): creating target 'tinyexr::tinyexr'")

include(FetchContent)
FetchContent_Declare(
    tinyexr
    GIT_REPOSITORY https://github.com/syoyo/tinyexr.git
    GIT_TAG bb751eb
)
FetchContent_Populate(tinyexr)

add_library(tinyexr)
add_library(tinyexr::tinyexr ALIAS tinyexr)

include(miniz)
target_sources(tinyexr
    PUBLIC
    ${tinyexr_SOURCE_DIR}/tinyexr.h
    PRIVATE
    ${tinyexr_SOURCE_DIR}/tinyexr.cc
)
target_include_directories(tinyexr
    PUBLIC
    ${tinyexr_SOURCE_DIR})
target_link_libraries(tinyexr
    PRIVATE
    miniz)

target_compile_features(tinyexr PUBLIC cxx_std_17)
