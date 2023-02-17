################################################################################
# Tests
################################################################################


set(APP_UNIT_TEST_SOURCES
    main.cpp
    )

set(APP_UNIT_TEST_LIBS wmtk::toolkit)

if(WMTK_APP_TRIWILD)
    set(APP_UNIT_TEST_LIBS ${APP_UNIT_TEST_LIBS} wmtk::triwild)
    set(APP_UNIT_TEST_SOURCES ${APP_UNIT_TEST_SOURCES}
        test_triwild.cpp
        )
endif()
if(WMTK_APP_TETWILD)
    set(APP_UNIT_TEST_LIBS ${APP_UNIT_TEST_LIBS} wmtk::tetwild)
    set(APP_UNIT_TEST_SOURCES ${APP_UNIT_TEST_SOURCES}
        file_write.cpp
        test_runner.cpp
        test_insertion.cpp
        test_operation_smooth.cpp
        test_operations.cpp
        )
endif()
if(WMTK_APP_HARMTET)
    set(APP_UNIT_TEST_LIBS ${APP_UNIT_TEST_LIBS} wmtk::harmonic_tet)
    set(APP_UNIT_TEST_SOURCES ${APP_UNIT_TEST_SOURCES}
        test_harmonic_triangulations.cpp
        )
endif()

add_executable(app_unit_tests ${APP_UNIT_TEST_SOURCES})

include(catch2)
target_link_libraries(app_unit_tests PUBLIC Catch2::Catch2)

target_link_libraries(app_unit_tests PUBLIC ${APP_UNIT_TEST_LIBS})
wmtk_copy_dll(app_unit_tests)


# data()
# set(DATA_DIR "${THIRD_PARTY_DIR}/data/")
# target_compile_definitions(app_unit_tests PUBLIC -DWILDMESHING_TOOLKIT_DATA_DIR=\"${DATA_DIR}\")
target_compile_definitions(app_unit_tests PUBLIC CATCH_CONFIG_ENABLE_BENCHMARKING)

################################################################################
# Register tests
################################################################################

foreach(source IN ITEMS ${APP_UNIT_TEST_SOURCES})
    source_group("tests" FILES "${source}")
endforeach()

# Include Catch2 and provide function `catch_discover_tests` to register tests.
FetchContent_GetProperties(catch2)
include("${catch2_SOURCE_DIR}/contrib/Catch.cmake")

catch_discover_tests(app_unit_tests)
