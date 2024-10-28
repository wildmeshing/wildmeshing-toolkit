list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/applications/cmake")


# clear out the parent targets
set(WMTK_APPLICATION_HEADER_TARGETS )

include(wmtk_add_application)
include(wmtk_register_integration_test)
include(wmtk_generate_test_config)

add_subdirectory(utils)

option(WMTK_ENABLE_APPLICATION_delaunay "Runs delaunay application" ON)
if(WMTK_ENABLE_APPLICATION_delaunay)
add_subdirectory(delaunay)
endif()

option(WMTK_ENABLE_APPLICATION_marching "Runs marching application" ON)
if(WMTK_ENABLE_APPLICATION_marching)
add_subdirectory(marching)
endif()

option(WMTK_ENABLE_APPLICATION_procedural "Runs procedural application" ON)
if(WMTK_ENABLE_APPLICATION_procedural)
add_subdirectory(procedural)
endif()

option(WMTK_ENABLE_APPLICATION_multimesh "Runs multimesh application" OFF)
if(WMTK_ENABLE_APPLICATION_multimesh)
add_subdirectory(multimesh)
endif()

option(WMTK_ENABLE_APPLICATION_isotropic_remeshing "Runs isotropic_remeshing application" OFF)
if(WMTK_ENABLE_APPLICATION_isotropic_remeshing)
add_subdirectory(isotropic_remeshing)
endif()

option(WMTK_ENABLE_APPLICATION_tetwild_simplification "Runs tetwild_simplification application" ON)
if(WMTK_ENABLE_APPLICATION_tetwild_simplification)
add_subdirectory(tetwild_simplification)
endif()

option(WMTK_ENABLE_APPLICATION_tetwild "Runs tetwild application" ON)
if(WMTK_ENABLE_APPLICATION_tetwild)
add_subdirectory(tetwild)
endif()

option(WMTK_ENABLE_APPLICATION_triwild "Runs triwild application" ON)
if(WMTK_ENABLE_APPLICATION_triwild)
add_subdirectory(triwild)
endif()

option(WMTK_ENABLE_APPLICATION_cdt_sec "Runs cdt_sec application" ON)
if(WMTK_ENABLE_APPLICATION_cdt_sec)
add_subdirectory(cdt_sec)
endif()

option(WMTK_ENABLE_APPLICATION_shortest_edge_collapse "Runs shortest-edge collapse application" ON)
if(WMTK_ENABLE_APPLICATION_shortest_edge_collapse)
add_subdirectory(shortest_edge_collapse)
endif()

option(WMTK_ENABLE_APPLICATION_longest_edge_split "Runs longest-edge split application" ON)
if(WMTK_ENABLE_APPLICATION_longest_edge_split)
add_subdirectory(longest_edge_split)
endif()

option(WMTK_ENABLE_APPLICATION_insertion "Runs insertion application" ON)
if(WMTK_ENABLE_APPLICATION_insertion)
add_subdirectory(insertion)
endif()

wmtk_generate_test_config("${WMTK_TEST_CONFIG}")


add_custom_target(wmtk_application_headers
    DEPENDS ${WMTK_APPLICATION_HEADER_TARGETS}
)

if(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT AND NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
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

find_package(Python REQUIRED)


#add_test(
#    NAME wmtk_integration_test
#    COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/integration_test.py
#    -c ${CMAKE_BINARY_DIR}/test_config.json
#    -b ${CMAKE_BINARY_DIR}/applications
#)
MESSAGE(STATUS "${WMTK_APPLICATION_TEST_NAMES}")
foreach(APP_TEST ${WMTK_APPLICATION_TEST_NAMES})
    MESSAGE(STATUS "${APP_TEST}")

add_test(
    NAME "wmtk_integration_test_${APP_TEST}"
    COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/integration_test.py
    -c "${CMAKE_BINARY_DIR}/test_config.json"
    -b "${CMAKE_BINARY_DIR}/applications"
    -t "${APP_TEST}"
)

    endforeach()
endif()
