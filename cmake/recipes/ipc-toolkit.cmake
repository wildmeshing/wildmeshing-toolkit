include(FetchContent)
FetchContent_Declare(
    ipc_toolkit
    GIT_REPOSITORY https://github.com/ipc-sim/ipc-toolkit.git
    GIT_TAG fda38f65f69e42c6811d2a133d12c31c06e1f2e9
)
FetchContent_MakeAvailable(ipc_toolkit)