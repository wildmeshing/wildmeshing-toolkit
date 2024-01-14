# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage("gh:JcDai/VolumeRemesher#9cfcbc64b115ad91f08ffbdbc0a1d59344aa4dbe")

set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)

