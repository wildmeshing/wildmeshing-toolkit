if (TARGET mshio::mshio)
    return()
endif()

message(STATUS "Third-party (external): creating target 'mshio::mshio'")

include(CPM)
CPMAddPackage(
    NAME mshio
    GITHUB_REPOSITORY qnzhou/MshIO
    GIT_TAG        dbfe01f072a90d067a25c5e962ea1f87e34c4fd3
)
set_target_properties(mshio PROPERTIES FOLDER third_party)