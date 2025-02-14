list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/applications/cmake")


# clear out the parent targets
set(WMTK_APPLICATION_HEADER_TARGETS )



include(wmtk_add_application)
include(wmtk_register_integration_test)
include(wmtk_generate_test_config)


add_subdirectory(utils)



macro(add_application APP_NAME DEFAULT_STATE)

    option(WMTK_ENABLE_APPLICATION_${APP_NAME} "Enables ${APP_NAME} Application" ${DEFAULT_STATE})
    if(WMTK_ENABLE_APPLICATION_${APP_NAME})
        add_subdirectory(${APP_NAME})
    endif()
endmacro()

add_application(delaunay ON)
add_application(marching ON)
add_application(procedural ON)
add_application(multimesh OFF)
add_application(isotropic_remeshing OFF)
add_application(tetwild_msh_converter ON)
add_application(tetwild_simplification ON)
add_application(triwild ON)
add_application(tetwild ON)
add_application(cdt_opt ON)
add_application(shortest_edge_collapse ON)
add_application(insertion ON)
add_application(convert ON)
add_application(longest_edge_split ON)


wmtk_generate_test_config("${WMTK_TEST_CONFIG}")


add_custom_target(wmtk_application_headers
    DEPENDS ${WMTK_APPLICATION_HEADER_TARGETS}
)

if(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
    set(INTEGRATION_TESTING_ENABLED_DEFAULT  ON)
else()
    set(INTEGRATION_TESTING_ENABLED_DEFAULT  OFF)
endif()

find_package(Python)
if(NOT Python_FOUND AND INTEGRATION_TESTING_ENABLED_DEFAULT)
    set(INTEGRATION_TESTING_ENABLED_DEFAULT  OFF)
    message(WARNING "Disabling Integration testing by default because python was not found, Install python and then enable WMTK_ENABLE_INTEGRATION_TESTING")
endif()

option(WMTK_ENABLE_INTEGRATION_TESTING "Whether integration testing is enabled, requires python" ${INTEGRATION_TESTING_ENABLED_DEFAULT})
if(WMTK_ENABLE_INTEGRATION_TESTING)
    enable_testing()

    set(WMTK_APPLICATION_RUN_ALL_TESTS "-a" )
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(WMTK_APPLICATION_RUN_ALL_TESTS "" )
    endif()

    find_package(Python REQUIRED)

    MESSAGE(STATUS "${WMTK_APPLICATION_TEST_NAMES}")
    foreach(APP_TEST ${WMTK_APPLICATION_TEST_NAMES})
        MESSAGE(STATUS "${APP_TEST}")

        add_test(
            NAME "wmtk_integration_test_${APP_TEST}"
            COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/integration_test.py
            -c "${CMAKE_BINARY_DIR}/test_config.json"
            -b "${CMAKE_BINARY_DIR}/applications"
            -t "${APP_TEST}"
            ${WMTK_APPLICATION_RUN_ALL_TESTS}
        )

    endforeach()
endif()
