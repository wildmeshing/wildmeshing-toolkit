if (TARGET mshio::mshio)
    return()
endif()

message(STATUS "Third-party (external): creating target 'mshio::mshio'")

FetchContent_Declare(
    mshio
    GIT_REPOSITORY https://github.com/qnzhou/MshIO.git
    GIT_TAG        dbfe01f072a90d067a25c5e962ea1f87e34c4fd3
)

FetchContent_MakeAvailable(mshio)
set_target_properties(mshio PROPERTIES FOLDER third_party)
