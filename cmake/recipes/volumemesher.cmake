# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage(
    NAME VolumeRemesher
    GITHUB_REPOSITORY wildmeshing/VolumeRemesher
    GIT_TAG 6ac1839d372457bf1c7f8083213742388e645145
    OPTIONS
    "VOLREM_ENABLE_AVX2 OFF"
    )

set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)