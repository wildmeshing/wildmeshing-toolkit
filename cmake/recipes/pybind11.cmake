if(TARGET pybind11::module)
    return()
endif()

message(STATUS "Third-party (external): creating target 'pybind11::module'")

include(CPM)
CPMAddPackage(
    NAME pybind
    GITHUB_REPOSITORY   pybind/pybind11
    GIT_TAG             ca1d996461d75ec01dbfa37f01b8c17aac957151
)
# set_target_properties(pybind PROPERTIES FOLDER third_party)