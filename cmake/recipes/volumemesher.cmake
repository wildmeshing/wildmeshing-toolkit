# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage("gh:JcDai/VolumeRemesher#8927a1e057c643f1193ed50117bb3fb311b81b09")

set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)