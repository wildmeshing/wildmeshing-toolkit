if(TARGET ipc::toolkit)
    return()
endif()


message(STATUS "Third-party (external): creating target 'ipc::toolkit'")
include(CPM)
CPMAddPackage(
    NAME ipc_toolkit
    GITHUB_REPOSITORY ipc-sim/ipc-toolkit
    GIT_TAG fda38f65f69e42c6811d2a133d12c31c06e1f2e9
)

set_target_properties(ipc_toolkit PROPERTIES FOLDER third_party)