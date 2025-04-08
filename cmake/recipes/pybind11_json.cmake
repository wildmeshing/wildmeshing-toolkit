# pybind11_json
# License: MIT

if(TARGET pybind11::json)
    return()
endif()

message(STATUS "Third-party: creating target 'pybind11::json'")

set(BUILD_TESTS		OFF CACHE BOOL "" FORCE)
set(DOWNLOAD_GTEST	OFF CACHE BOOL "" FORCE)

include(FetchContent)
FetchContent_Declare(
    pybind11_json
    GIT_REPOSITORY https://github.com/pybind/pybind11_json.git
    GIT_TAG 0.2.11
    GIT_SHALLOW FALSE
)
FetchContent_MakeAvailable(pybind11_json)

add_library(pybind11::json ALIAS pybind11_json)
