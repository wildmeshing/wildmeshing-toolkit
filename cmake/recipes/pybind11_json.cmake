if(TARGET pybind11_json)
    return()
endif()

message(STATUS "Third-party (external): creating target 'pybind11_json'")

include(CPM)
CPMAddPackage(
    NAME pybind11_json
    GITHUB_REPOSITORY   pybind/pybind11_json
    GIT_TAG             d0bf434be9d287d73a963ff28745542daf02c08f
)
# set_target_properties(pybind11_json PROPERTIES FOLDER third_party)