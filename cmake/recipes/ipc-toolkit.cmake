include(FetchContent)
FetchContent_Declare(
    ipc_toolkit
    GIT_REPOSITORY https://github.com/ipc-sim/ipc-toolkit.git
    GIT_TAG 6aaf0ca84917d595a16a1d37dc1e6cd98ca0467c
)
FetchContent_MakeAvailable(ipc_toolkit)