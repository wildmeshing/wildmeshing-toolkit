if (TARGET mshio::mshio)
    return()
endif()

message(STATUS "Third-party (external): creating target 'mshio::mshio'")

FetchContent_Declare(
    mshio
    GIT_REPOSITORY https://github.com/qnzhou/MshIO.git
    GIT_TAG        main
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(mshio)
set_target_properties(mshio PROPERTIES FOLDER third_party)
