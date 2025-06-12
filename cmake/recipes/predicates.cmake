if(TARGET predicates::predicates)
    return()
endif()

message(STATUS "Third-party: creating target 'predicates::predicates'")

include(CPM)
CPMAddPackage(
    NAME predicates
    GITHUB_REPOSITORY wildmeshing/libigl-predicates
    GIT_TAG 83b924928a12d9b578968508b303f90c738d6deb
)

FetchContent_MakeAvailable(predicates)
add_library(predicates::predicates ALIAS predicates)

set_target_properties(predicates PROPERTIES FOLDER third_party)