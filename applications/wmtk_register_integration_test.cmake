macro(wmtk_register_integration_test EXEC_NAME CONFIG_FILE GIT_REPOSITORY GIT_TAG)
    include(CPM)

    MESSAGE(STATUS "Registering integration test for ${EXEC_NAME}")

    string(MD5 REPO_HASH ${GIT_REPOSITORY})

    SET(WMTK_I_DATA_FOLDER ${CPM_SOURCE_CACHE}/wmtk_integration_data/${REPO_HASH}/${GIT_TAG})

    include(FetchContent)
    FetchContent_Declare(
        wmtk_integration_data_${GIT_TAG}
        GIT_REPOSITORY ${GIT_REPOSITORY}
        GIT_TAG ${GIT_TAG}
        SOURCE_DIR ${WMTK_I_DATA_FOLDER}
    )
    FetchContent_MakeAvailable(wmtk_integration_data_${GIT_TAG})

    list (APPEND WMTK_TEST_CONFIG "\"${EXEC_NAME}\":{\"data_folder\":\"${WMTK_I_DATA_FOLDER}\", \"config_file\":\"${CONFIG_FILE}\"}")

    SET(WMTK_TEST_CONFIG ${WMTK_TEST_CONFIG} PARENT_SCOPE)
endmacro()
