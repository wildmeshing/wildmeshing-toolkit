macro(wmtk_register_integration_test EXEC_NAME CONFIG_FILE GIT_REPOSITORY GIT_TAG)
    include(CPM)

    CPMAddPackage(
        NAME wmtk_integration_data
        GIT_REPOSITORY ${GIT_REPOSITORY}
        GIT_TAG ${GIT_TAG}
        DOWNLOAD_ONLY True
    )

    list (APPEND WMTK_TEST_CONFIG "\"${EXEC_NAME}\":{\"data_folder\":\"${wmtk_${EXEC_NAME}_data_SOURCE_DIR}\", \"config_file\":\"${CONFIG_FILE}\"}")

    SET(WMTK_TEST_CONFIG ${WMTK_TEST_CONFIG} PARENT_SCOPE)

endmacro()
