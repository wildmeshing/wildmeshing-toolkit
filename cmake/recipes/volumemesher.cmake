# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage(
    NAME VolumeRemesher
    GITHUB_REPOSITORY JcDai/VolumeRemesher
    GIT_TAG 3a38afab245089798f30415be6ee480232f570aa
    OPTIONS
    "VOLREM_ENABLE_AVX2 OFF"
    )

set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)