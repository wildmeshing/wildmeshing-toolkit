# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage("gh:JcDai/VolumeRemesher#c9b2e0c856d8a05d9fd351724116f989ec41fcee")

set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)