
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
    GIT_TAG df8812b0c876376886cf73cc64d29e3a7e7be83c
)

set_target_properties(simple_bvh PROPERTIES FOLDER third_party)

