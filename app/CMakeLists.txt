
include(wmtk_data)
include(jse)

option(WMTK_APP_HARMTET "Harmonic Triangulation app" ON)
option(WMTK_APP_UNIFORM_REMESH "uniform remesh for surfaces" ON)
option(WMTK_APP_SEC "surface shortest edge collapse" ON)
option(WMTK_APP_QSLIM "surface qslim simplification" ON)
option(WMTK_APP_INTERIOR_TET_OPT "interior tet opt application" ON)

option(WMTK_APP_UNIT_TESTS "unit tests for applications" ON)

if(WMTK_APP_INTERIOR_TET_OPT)
add_subdirectory(interior_tet_opt)
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

# ###############################################################################
# WMTK Application
# ###############################################################################

add_executable(wmtk_app main.cpp)
target_link_libraries(wmtk_app PRIVATE 
    nlohmann_json::nlohmann_json
    CLI11::CLI11
    FastEnvelope::FastEnvelope
)

if(WMTK_PYBIND)
    add_subdirectory(pywmtk)
endif()