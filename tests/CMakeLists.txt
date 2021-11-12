################################################################################
# Tests
################################################################################

set(test_sources
	main.cpp

	test_operations.cpp
)

add_executable(unit_tests ${test_sources})

################################################################################
# Required Libraries
################################################################################

target_link_libraries(unit_tests PUBLIC WildMeshingToolkit)

target_link_libraries(unit_tests PUBLIC warnings::all)

include(catch2)
target_link_libraries(unit_tests PUBLIC Catch2::Catch2)

################################################################################
# Compiler options
################################################################################

# wildmeshing_toolkit_polyfem_data()
# set(DATA_DIR "${THIRD_PARTY_DIR}/data/")
# target_compile_definitions(unit_tests PUBLIC -DWILDMESHING_TOOLKIT_DATA_DIR=\"${DATA_DIR}\")

target_compile_definitions(unit_tests PUBLIC CATCH_CONFIG_ENABLE_BENCHMARKING)

################################################################################
# Sanitizers
################################################################################

if(WILDMESHING_TOOLKIT_WITH_SANITIZERS)
	add_sanitizers(unit_tests)
endif()

################################################################################
# Register tests
################################################################################

foreach(source IN ITEMS ${test_sources})
    source_group("tests" FILES "${source}")
endforeach()

set(PARSE_CATCH_TESTS_ADD_TO_CONFIGURE_DEPENDS ON)

# Include Catch2 and provide function `catch_discover_tests` to register tests.
FetchContent_GetProperties(catch2)
include("${catch2_SOURCE_DIR}/contrib/Catch.cmake")

catch_discover_tests(unit_tests)
