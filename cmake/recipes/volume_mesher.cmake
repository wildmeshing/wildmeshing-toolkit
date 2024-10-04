# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage("gh:JcDai/VolumeRemesher#2d123095dc0ac3d8387a41daa499057913dcfe96")

set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)

