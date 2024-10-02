list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/applications")

include(wmtk_add_application)
include(wmtk_register_integration_test)
include(wmtk_generate_test_config)

option(WMTK_ENABLE_APPLICATION_delaunay "Runs delaunay application" ON)
if(WMTK_ENABLE_APPLICATION_delaunay)
add_subdirectory(delaunay)
endif()

option(WMTK_ENABLE_APPLICATION_marching "Runs marching application" ON)
if(WMTK_ENABLE_APPLICATION_marching)
add_subdirectory(marching)
endif()

option(WMTK_ENABLE_APPLICATION_procedural "Runs procedural application" OFF)
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

option(WMTK_ENABLE_APPLICATION_cdt_sec "Runs cdt_sec application" ON)
if(WMTK_ENABLE_APPLICATION_cdt_sec)
add_subdirectory(cdt_sec)
endif()

wmtk_generate_test_config("${WMTK_TEST_CONFIG}")


if(WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
    enable_testing()

    find_package(Python REQUIRED)

    add_test(
        NAME wmtk_integration_test
        COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/integration_test.py
        -c ${CMAKE_BINARY_DIR}/test_config.json
        -b ${CMAKE_BINARY_DIR}/applications
    )
endif()