
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

if(NOT DEFINED CPM_SOURCE_CACHE) 
    if(NOT DEFINED ENV{CPM_SOURCE_CACHE})
    # Set CPM cache folder if unset
    file(REAL_PATH "${CMAKE_BINARY_DIR}/CPM" CPM_PATH EXPAND_TILDE)

    # we want to store this variable relatively globally so it's cache
    # Here we're completely disallowing CPM to be turned off, so if no
    # SOURCE_CACHE is chosen then we are forcing it to be something
    set(CPM_SOURCE_CACHE
        ${CPM_PATH}
        CACHE PATH "Directory to download CPM dependencies" FORCE
)
endif()
endif()
message(STATUS "Using CPM cache folder: ${CPM_SOURCE_CACHE}")
