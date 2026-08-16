# IPC Toolkit (https://github.com/wildmeshing/ipc-toolkit)
# License: MIT
#
# Public mirror of fsichetti/ipc-toolkit@upstream-merge, published in the wildmeshing
# organisation with the author's permission. What wmtk uses from it is the `high_order_contact`
# subtree: the smooth (offset geometric contact) potential and its analytic gradient and
# Hessian, which the topological_offset component evaluates as its offset field.
#
# Only the topological_offset component links this. Core wmtk::toolkit does not, so nothing
# here reaches tetwild, triwild or simwild.

if(TARGET ipc::toolkit)
    return()
endif()

message(STATUS "Third-party: creating target 'ipc::toolkit'")

# wmtk asks for C++17 per target (target_compile_features), leaving CMAKE_CXX_STANDARD unset.
# Abseil, which ipc-toolkit pulls in for its hash functions, probes the compiler's DEFAULT
# standard at configure time and hard-errors when that is below 17 -- so a per-target request is
# invisible to it. Set the directory-scoped default for the subtree ipc-toolkit is configured in.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CPM)
CPMAddPackage(
    NAME ipc_toolkit
    GITHUB_REPOSITORY wildmeshing/ipc-toolkit
    GIT_TAG 5f301a258bb7cf9cd4e8fb7a348d1089aafcb0e7
    OPTIONS
    "IPC_TOOLKIT_BUILD_TESTS OFF"
    "IPC_TOOLKIT_BUILD_PYTHON OFF"
    "IPC_TOOLKIT_WITH_CUDA OFF"
)

# ipc-toolkit links its hash-map backends PRIVATE, but leaks them through the PUBLIC header
# ipc/utils/unordered_map_and_set.hpp, so any consumer including it fails to find
# <tsl/robin_map.h> / <absl/hash/hash.h>. Upstream's own test target works around this by
# linking them again; do it once here instead, so every wmtk consumer inherits the fix.
foreach(IPC_LEAKED_DEP IN ITEMS tsl::robin_map absl::hash)
    if(TARGET ${IPC_LEAKED_DEP})
        target_link_libraries(ipc_toolkit PUBLIC ${IPC_LEAKED_DEP})
    endif()
endforeach()
