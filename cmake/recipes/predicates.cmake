if(TARGET predicates::predicates)
    return()
endif()

message(STATUS "Third-party: creating target 'predicates::predicates'")

include(CPM)
    CPMAddPackage(
    NAME predicates
    GITHUB_REPOSITORY wildmeshing/libigl-predicates
    GIT_TAG        f1fcce5d20d5a8e887cb694dc6aeaafab9ca1e29
)

FetchContent_MakeAvailable(predicates)
add_library(predicates::predicates ALIAS predicates)

set_target_properties(predicates PROPERTIES FOLDER third_party)