# VolumeMesher (From Marco Attene)

if(TARGET mesh_generator_lib)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage(
    NAME volumemesher
    GITHUB_REPOSITORY JcDai/VolumeRemesher
    GIT_TAG 87ac225118ad8c969beb530f837491e356266e52
)
set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)