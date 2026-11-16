if(TARGET pybind11::module)
    return()
endif()

message(STATUS "Third-party (external): creating target 'pybind11::module'")

include(CPM)
CPMAddPackage(
    NAME pybind
    GITHUB_REPOSITORY   pybind/pybind11
    GIT_TAG             v3.0.3 # polyfem's pin
)
# set_target_properties(pybind PROPERTIES FOLDER third_party)