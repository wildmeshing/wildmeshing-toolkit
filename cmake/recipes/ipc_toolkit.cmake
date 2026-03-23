# IPC Toolkit (https://github.com/ipc-sim/ipc-toolkit)
# License: MIT

if(TARGET ipc::toolkit)
    return()
endif()

message(STATUS "Third-party: creating target 'ipc::toolkit'")

include(CPM)
CPMAddPackage(
    NAME ipc_toolkit
    GITHUB_REPOSITORY ipc-sim/ipc-toolkit
    GIT_TAG e94c11d174de8f9805fcebc3b0cfe3b9f1306ab5
    OPTIONS
    "IPC_TOOLKIT_WITH_CUDA OFF"
    "IPC_TOOLKIT_WITH_SIMD OFF"
)

set_target_properties(ipc_toolkit PROPERTIES FOLDER third_party)