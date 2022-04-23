
include(wmtk_data)

option(WMTK_APP_TETWILD ON)
option(WMTK_APP_HARMTET ON)
option(WMTK_APP_UNIFORM_REMESH ON)
option(WMTK_APP_SEC ON)
option(WMTK_APP_QSLIM ON)
option(WMTK_APP_UNIT_TESTS ON)

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