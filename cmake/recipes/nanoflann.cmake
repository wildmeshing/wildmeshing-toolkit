if (TARGET nanoflann::nanoflann)
    return()
endif()

message(STATUS "Third-party (external): creating target 'nanoflann::nanoflann'")

include(CPM)
CPMAddPackage(
    NAME nanoflann
    GITHUB_REPOSITORY jlblancoc/nanoflann
    GIT_TAG        81cd02b64391cba29aa2da59ce1ff5be0ed7c6a7
    OPTIONS
    "NANOFLANN_BUILD_EXAMPLES OFF"
    "NANOFLANN_BUILD_TESTS OFF"
    "NANOFLANN_USE_SYSTEM_GTEST OFF"
    "MASTER_PROJECT_HAS_TARGET_UNINSTALL ON"
)
set_target_properties(nanoflann PROPERTIES FOLDER third_party)
if(WIN32)
    set_target_properties(nanoflann_uninstall PROPERTIES FOLDER third_party)
endif()