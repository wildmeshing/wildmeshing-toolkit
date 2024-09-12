function(wmtk_register_integration_test EXEC_NAME CONFIG_FILE GIT_REPOSITORY GIT_TAG)
    include(CPM)

    MESSAGE(STATUS "Registering integration test for ${EXEC_NAME}")
    string(MD5 REPO_HASH "${GIT_REPOSITORY}${GIT_TAG}")

    include(wmtk_download_data)
    wmtk_download_data(DATA_DIR ${GIT_REPOSITORY} ${GIT_TAG})

    list (APPEND WMTK_TEST_CONFIG "\"${EXEC_NAME}\":{\"data_folder\":\"${DATA_DIR}\", \"config_file\":\"${CONFIG_FILE}\"}")

    SET(WMTK_TEST_CONFIG ${WMTK_TEST_CONFIG} PARENT_SCOPE)
endfunction()
