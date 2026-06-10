if (TARGET mshio::mshio)
    return()
endif()

message(STATUS "Third-party (external): creating target 'mshio::mshio'")

include(CPM)
CPMAddPackage(
    NAME mshio
    GITHUB_REPOSITORY qnzhou/MshIO
    GIT_TAG        e18c8a6c4140da0c16c0cba6273e40d9d1f2f91f
)
set_target_properties(mshio PROPERTIES FOLDER third_party)