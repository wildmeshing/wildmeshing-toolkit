# Sanitizers MIT (optional)

message(STATUS "Third-party: creating 'Sanitizers'")

include(CPM)
CPMAddPackage(
    NAME sanitizers-cmake
    GITHUB_REPOSITORY arsenm/sanitizers-cmake
    GIT_TAG 6947cff3a9c9305eb9c16135dd81da3feb4bf87f
    DOWNLOAD_ONLY ON
)

list(APPEND CMAKE_MODULE_PATH ${sanitizers-cmake_SOURCE_DIR}/cmake)

find_package(Sanitizers)