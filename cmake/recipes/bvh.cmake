
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
    GIT_TAG 995e179142cc30799464b8124b02ff509709bf06
)

set_target_properties(simple_bvh PROPERTIES FOLDER third_party)

