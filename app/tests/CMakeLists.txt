################################################################################
# Tests
################################################################################

set(test_sources
	main.cpp

	test_2d_operations.cpp
	test_operations.cpp
	file_write.cpp
	test_operation_smooth.cpp
	test_runner.cpp
)

add_executable(app_unit_tests ${test_sources})

target_link_libraries(app_unit_tests PUBLIC WildMeshingToolkit TetWild EdgeCollapse)
target_link_libraries(app_unit_tests PUBLIC warnings::all)

include(catch2)
target_link_libraries(app_unit_tests PUBLIC Catch2::Catch2)


# data()
# set(DATA_DIR "${THIRD_PARTY_DIR}/data/")
# target_compile_definitions(app_unit_tests PUBLIC -DWILDMESHING_TOOLKIT_DATA_DIR=\"${DATA_DIR}\")
target_compile_definitions(app_unit_tests PUBLIC CATCH_CONFIG_ENABLE_BENCHMARKING)

################################################################################
# Register tests
################################################################################

foreach(source IN ITEMS ${test_sources})
    source_group("tests" FILES "${source}")
endforeach()

# Include Catch2 and provide function `catch_discover_tests` to register tests.
FetchContent_GetProperties(catch2)
include("${catch2_SOURCE_DIR}/contrib/Catch.cmake")

catch_discover_tests(app_unit_tests)