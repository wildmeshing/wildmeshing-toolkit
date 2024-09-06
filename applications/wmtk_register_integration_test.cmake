macro(wmtk_register_integration_test EXEC_NAME CONFIG_FILE GIT_REPOSITORY GIT_TAG)
    include(CPM)
if(CPM_SOURCE_CACHE)
    set(CPM_TARGET_DIR wmtk_integration_data)
else()
    set(CPM_TARGET_DIR wmtk_${EXEC_NAME}_data)
endif()
    CPMAddPackage(
        NAME ${CPM_TARGET_DIR}
        GIT_REPOSITORY ${GIT_REPOSITORY}
        GIT_TAG ${GIT_TAG}
        DOWNLOAD_ONLY True
    )

    set(CPM_DATA_DIR ${CPM_TARGET_DIR}_SOURCE_DIR)

list (APPEND WMTK_TEST_CONFIG "\"${EXEC_NAME}\":{\"data_folder\":\"${CPM_DATA_DIR}\", \"config_file\":\"${CONFIG_FILE}\"}")

    SET(WMTK_TEST_CONFIG ${WMTK_TEST_CONFIG} PARENT_SCOPE)

endmacro()
