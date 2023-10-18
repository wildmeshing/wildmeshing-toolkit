if(TARGET lagrange::core)
    return()
endif()

include(CPM)
CPMAddPackage(
    NAME lagrange
    GITHUB_REPOSITORY adobe/lagrange
    GIT_TAG v6.11.0
)
