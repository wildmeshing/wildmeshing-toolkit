# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage("gh:JcDai/VolumeRemesher#87ac225118ad8c969beb530f837491e356266e52")

set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)

