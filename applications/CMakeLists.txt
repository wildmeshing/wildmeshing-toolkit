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

wmtk_generate_test_config("${WMTK_TEST_CONFIG}")
