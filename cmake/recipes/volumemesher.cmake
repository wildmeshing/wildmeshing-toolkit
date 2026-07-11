# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage("gh:JcDai/VolumeRemesher#f3fcff14871ee3be9ff266382b49d83f9155a9e5")

set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)