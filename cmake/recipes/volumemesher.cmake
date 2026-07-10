# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage("gh:JcDai/VolumeRemesher#a42ad61931bcae84fd723e6c49cdcac4b2eb234c")

set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)