if (TARGET mshio::mshio)
    return()
endif()

message(STATUS "Third-party (external): creating target 'mshio::mshio'")

FetchContent_Declare(
    mshio
    GIT_REPOSITORY https://github.com/qnzhou/MshIO.git
    GIT_TAG        5fadb581700254f75e42269aacbd9800fd5064f2
)

FetchContent_MakeAvailable(mshio)
set_target_properties(mshio PROPERTIES FOLDER third_party)
