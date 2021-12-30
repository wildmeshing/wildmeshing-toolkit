# data
# License: MIT

message(STATUS "Third-party: fetching 'meshes'")

include(FetchContent)
FetchContent_Declare(
    wmt_data
    GIT_REPOSITORY https://github.com/wildmeshing/data.git
    GIT_TAG 1484054abbac36e9c8340c3b32d87ad6eee45016
    GIT_SHALLOW FALSE
    SOURCE_DIR ${WMT_DATA_ROOT}/data
)

FetchContent_GetProperties(wmt_data)
if(NOT wmt_data_POPULATED)
  FetchContent_Populate(wmt_data)
  SET(WMT_DATA_DIR ${wmt_data_SOURCE_DIR})
endif()