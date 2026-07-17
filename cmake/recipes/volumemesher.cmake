# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage(
    NAME VolumeRemesher
    GITHUB_REPOSITORY wildmeshing/VolumeRemesher
    GIT_TAG 9e32fe8329890d564e179ed3176f266d8f519f12
    )

set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)