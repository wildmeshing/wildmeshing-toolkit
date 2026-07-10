# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage("gh:JcDai/VolumeRemesher#212ee6cfb68b9d1cbdf76cf78bcec8f69a5f9faf")

set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)