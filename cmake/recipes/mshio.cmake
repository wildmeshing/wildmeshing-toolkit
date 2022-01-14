if (TARGET mshio::mshio)
    return()
endif()

message(STATUS "Third-party (external): creating target 'mshio::mshio'")

FetchContent_Declare(
    mshio
    GIT_REPOSITORY https://github.com/qnzhou/MshIO.git
    GIT_TAG 995c28d6b615c74f4d5e27ac1185c040d5b57a33
    # GIT_TAG        5fadb581700254f75e42269aacbd9800fd5064f2
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(mshio)
set_target_properties(mshio PROPERTIES FOLDER third_party)
