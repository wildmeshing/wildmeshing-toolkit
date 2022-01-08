include_guard()

if (NOT mshio::mshio)
    FetchContent_Declare(
        MshIO
        GIT_REPOSITORY https://github.com/qnzhou/MshIO.git
        GIT_TAG        main
        GIT_SHALLOW TRUE
    )

    FetchContent_GetProperties(MshIO)
    if (NOT mshio_POPULATED)
        FetchContent_Populate(MshIO)
        option(MSHIO_EXT_NANOSPLINE "Enable nanospline extension" On)
        add_subdirectory(${mshio_SOURCE_DIR} ${mshio_BINARY_DIR})
    endif()

endif()
