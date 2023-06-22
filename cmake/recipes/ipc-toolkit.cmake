if(TARGET ipc::toolkit)
    return()
endif()


message(STATUS "Third-party (external): creating target 'ipc::toolkit'")
include(FetchContent)
FetchContent_Declare(
    ipc_toolkit
    GIT_REPOSITORY https://github.com/ipc-sim/ipc-toolkit.git
    GIT_TAG fda38f65f69e42c6811d2a133d12c31c06e1f2e9
)
FetchContent_MakeAvailable(ipc_toolkit)

set_target_properties(ipc_toolkit PROPERTIES FOLDER third_party)