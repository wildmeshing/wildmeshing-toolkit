
include(wmtk_data)

option(WMTK_APP_TETWILD "tetwild application" ON)
option(WMTK_APP_TRIWILD "triwild application" ON)
option(WMTK_APP_HARMTET "Harmonic Triangulation app" ON)
option(WMTK_APP_UNIFORM_REMESH "uniform remesh for surfaces" ON)
option(WMTK_APP_SEC "surface shortest edge collapse" ON)
option(WMTK_APP_QSLIM "surface qslim simplification" ON)
option(WMTK_APP_UNIT_TESTS "unit tests for applications" ON)
option(WMTK_APP_INTERIOR_TET_OPT "interior tet opt application" ON)
option(WMTK_APP_INCREMENTAL_TETWILD "incremental tetwild" OFF)

if(WMTK_APP_INTERIOR_TET_OPT)
    add_subdirectory(interior_tet_opt)
endif()

if(WMTK_APP_INCREMENTAL_TETWILD)
    add_subdirectory(incremental_tetwild)   
endif()



if(WMTK_APP_TETWILD)
    add_subdirectory(tetwild)
endif()

if(WMTK_APP_TRIWILD)
    add_subdirectory(triwild)
endif()

if(WMTK_APP_HARMTET)
    add_subdirectory(harmonic_tet)
endif()

if(WMTK_APP_UNIFORM_REMESH)
    add_subdirectory(remeshing)
endif()

if(WMTK_APP_SEC)
    add_subdirectory(shortest_edge_collapse)
endif()

if(WMTK_APP_QSLIM)
    add_subdirectory(qslim)
endif()

