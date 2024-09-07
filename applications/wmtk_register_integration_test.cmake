macro(wmtk_register_integration_test EXEC_NAME CONFIG_FILE GIT_REPOSITORY GIT_TAG)
    include(CPM)

    MESSAGE(STATUS "Registering integration test for ${EXEC_NAME}")
    string(MD5 REPO_HASH "${GIT_REPOSITORY}${GIT_TAG}")

    SET(WMTK_I_DATA_FOLDER ${CPM_SOURCE_CACHE}/wmtk_integration_data/${REPO_HASH})
    MESSAGE(STATUS "Test data stored at ${WMTK_I_DATA_FOLDER}")

    include(FetchContent)
    FetchContent_Populate(
        wmtk_id
        QUIET
        GIT_REPOSITORY ${GIT_REPOSITORY}
        GIT_TAG ${GIT_TAG}
        SOURCE_DIR ${WMTK_I_DATA_FOLDER}
    )
    # FetchContent_MakeAvailable(wmtk_id_${REPO_HASH})

    # message(STATUS "Test data folder: ${WMTK_I_DATA_FOLDER}")
    # message(STATUS "Test data name: wmtk_id_${REPO_HASH}")

    # ExternalProject_Add(
    #     wmtk_id_${REPO_HASH}
    #     SOURCE_DIR ${WMTK_I_DATA_FOLDER}
    #     GIT_REPOSITORY ${GIT_REPOSITORY}
    #     GIT_TAG ${GIT_TAG}
    #     CONFIGURE_COMMAND ""
    #     BUILD_COMMAND ""
    #     INSTALL_COMMAND ""
    #     LOG_DOWNLOAD ON
    # )
    # add_dependencies(${EXEC_NAME} wmtk_id_${REPO_HASH})
    # list (APPEND WMTK_TEST_CONFIG "\"${EXEC_NAME}\":{\"data_folder\":\"${WMTK_I_DATA_FOLDER}\", \"config_file\":\"${CONFIG_FILE}\"}")


    # CPMAddPackage(
    #     NAME wmtk_itd_${REPO_HASH}
    #     GIT_REPOSITORY ${GIT_REPOSITORY}
    #     GIT_TAG ${GIT_TAG}
    #     GIT_SHALLOW TRUE
    #     DOWNLOAD_ONLY TRUE
    # )
    # list (APPEND WMTK_TEST_CONFIG "\"${EXEC_NAME}\":{\"data_folder\":\"${wmtk_itd_${REPO_HASH}_SRC}\", \"config_file\":\"${CONFIG_FILE}\"}")


    SET(WMTK_TEST_CONFIG ${WMTK_TEST_CONFIG} PARENT_SCOPE)
endmacro()
