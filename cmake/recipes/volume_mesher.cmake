# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage("gh:JcDai/VolumeRemesher#a5e6d9f0df1ccd7070360cf957aa74d6e6a9d7d6")

set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)

