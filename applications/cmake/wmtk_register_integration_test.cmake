macro(wmtk_register_integration_test )

 set(options )
 set(oneValueArgs EXEC_NAME CONFIG_FILE GIT_REPOSITORY GIT_TAG CONFIG_PATH EXTRA_ARGUMENTS)
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

    list (APPEND WMTK_APPLICATION_TEST_NAMES "${_EXEC_NAME}")
    list (APPEND WMTK_TEST_CONFIG 
        "\"${_EXEC_NAME}\":
        {
        ${DATA_DIR_LINE}
        \"config_file\":\"${_CONFIG_FILE}\",
        \"config_folder\":\"${_CONFIG_PATH}\",
        \"extra_flags\":\"${_EXTRA_ARGUMENTS}\"
        }")

        set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/.." APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_CONFIG_PATH}")

    SET(WMTK_TEST_CONFIG ${WMTK_TEST_CONFIG} PARENT_SCOPE)
    SET(WMTK_APPLICATION_TEST_NAMES ${WMTK_APPLICATION_TEST_NAMES} PARENT_SCOPE)
endmacro()
