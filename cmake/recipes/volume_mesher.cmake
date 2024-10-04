# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage("gh:JcDai/VolumeRemesher#dffb9ca9ab14a0d8fd4aa3eaa21fafed3980ccfb")

set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)

