
if(TARGET simple_bvh::simple_bvh)
    return()
endif()

message(STATUS "Third-party (external): creating target 'simple_bvh::simple_bvh'")
option(SIMPLE_BVH_BUILD_TESTS "Build Tests" OFF)
set(SIMPLE_BVH_BUILD_TESTS OFF CACHE BOOL "" FORCE)

include(CPM)
CPMAddPackage(
    NAME simple_bvh
    GITHUB_REPOSITORY geometryprocessing/SimpleBVH
    GIT_TAG 2b08a6782922a25721cf889bd1ef47946e553ece
)

set_target_properties(simple_bvh PROPERTIES FOLDER third_party)

