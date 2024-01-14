# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage("gh:JcDai/VolumeRemesher#ac88ed128374cb35900a5a3262a45d416d2e46c9")

set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)

