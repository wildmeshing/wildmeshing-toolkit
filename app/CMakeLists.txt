
include(wmtk_data)
include(jse)
# Every executable under here links CLI11::CLI11. The target used to arrive as a side effect
# of fenvelope.cmake pulling cli11 in for fast-envelope's own app, which stopped being true
# when fast-envelope dropped that app -- so declare it where it is actually used.
include(cli11)

option(WMTK_APP_HARMTET "Harmonic Triangulation app" ON)
option(WMTK_APP_INTERIOR_TET_OPT "interior tet opt application" ON)

option(WMTK_APP_UNIT_TESTS "unit tests for applications" ON)

if(WMTK_APP_INTERIOR_TET_OPT)
add_subdirectory(interior_tet_opt)
endif()

if(WMTK_APP_HARMTET)
    add_subdirectory(harmonic_tet)
endif()

# ###############################################################################
# WMTK Application
# ###############################################################################

add_executable(wmtk_app main.cpp)
target_link_libraries(wmtk_app PUBLIC 
    wmtk::toolkit
    nlohmann_json::nlohmann_json
    CLI11::CLI11
)

wmtk_copy_dll(wmtk_app)

if(WMTK_PYBIND)
    add_subdirectory(pywildmeshing)
endif()