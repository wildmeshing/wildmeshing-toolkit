macro(wmtk_register_integration_test )

 set(options )
 set(oneValueArgs EXEC_NAME CONFIG_FILE GIT_REPOSITORY GIT_TAG CONFIG_PATH)
 set(multiValueArgs)
 cmake_parse_arguments("" "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )
    include(CPM)


    MESSAGE(STATUS "Registering integration test for ${_EXEC_NAME}")

    if(_GIT_REPOSITORY)
        string(MD5 REPO_HASH "${_GIT_REPOSITORY}${_GIT_TAG}")

        include(wmtk_download_data)
        wmtk_download_data(DATA_DIR ${_GIT_REPOSITORY} ${_GIT_TAG})
        set(DATA_DIR_LINE  "\"data_folder\":\"${DATA_DIR}\"," )
    endif()

    if(NOT _CONFIG_PATH)
        set(_CONFIG_PATH ${DATA_DIR})
    endif()

    list (APPEND WMTK_TEST_CONFIG 
        "\"${_EXEC_NAME}\":
        {
        ${DATA_DIR_LINE}
        \"config_file\":\"${_CONFIG_FILE}\",
        \"config_folder\":\"${_CONFIG_PATH}\"
        }")

    SET(WMTK_TEST_CONFIG ${WMTK_TEST_CONFIG} PARENT_SCOPE)
endmacro()
