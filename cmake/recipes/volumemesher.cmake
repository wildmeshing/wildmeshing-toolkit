# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage("gh:JcDai/VolumeRemesher#4543ab03e4852df319afb378e873aa629d546cf6")

set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)