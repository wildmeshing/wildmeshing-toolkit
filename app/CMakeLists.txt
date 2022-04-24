
include(wmtk_data)

option(WMTK_APP_TETWILD "tetwild application" ON)
option(WMTK_APP_HARMTET "Harmonic Triangulation app" ON)
option(WMTK_APP_UNIFORM_REMESH "uniform remesh for surfaces" ON)
option(WMTK_APP_SEC "surface shortest edge collapse" ON)
option(WMTK_APP_QSLIM "surface qslim simplification" ON)
option(WMTK_APP_UNIT_TESTS "unit tests for applications" ON)

if (WMTK_APP_TETWILD)
add_subdirectory(tetwild)
endif()

if (WMTK_APP_HARMTET)
add_subdirectory(harmonic_tet)
endif()

if (WMTK_APP_UNIFORM_REMESH)
add_subdirectory(Remeshing)
endif()

if (WMTK_APP_SEC)
add_subdirectory(ShortestEdgeCollapse)
endif()

if(WMTK_APP_QSLIM)
add_subdirectory(QSLIM)
endif()

################################################################################
# Tests
################################################################################

if (WMTK_APP_UNIT_TESTS)
add_subdirectory(tests)
endif()