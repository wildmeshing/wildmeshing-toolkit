macro(wmtk_download_test_data APP_NAME EXEC_NAME GIT_REPO HASH)
    include(CPM)

    CPMAddPackage(
        NAME ${APP_NAME}_data
        GIT_REPOSITORY ${GIT_REPO}
        GIT_TAG ${HASH}
        DOWNLOAD_ONLY True
    )

    list (APPEND WMTK_TEST_CONFIG "\"${APP_NAME}\":{\"data_folder\":\"${${APP_NAME}_data_SOURCE_DIR}\", \"executable\":\"${EXEC_NAME}\"}")

    SET(WMTK_TEST_CONFIG ${WMTK_TEST_CONFIG} PARENT_SCOPE)

endmacro()
