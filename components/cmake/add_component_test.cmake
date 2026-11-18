# function(add_component_test COMPONENT_TARGET_NAME ...)

#     if(NOT WILDMESHING_TOOLKIT_TOPLEVEL_PROJECT)
#         return()
#     endif()
#     list(REMOVE_AT ARGV 0)

#     message(STATUS "Add test files for component ${COMPONENT_TARGET_NAME}: ${ARGV}")

#     target_sources(${WMTK_COMPONENT_TEST_TARGET} PUBLIC ${ARGV})
#     target_link_libraries(${WMTK_COMPONENT_TEST_TARGET} PRIVATE ${COMPONENT_TARGET_NAME} wmtk_test_tools)

# endfunction()

function(add_component_test COMPONENT_NAME)
    # PER_CASE: one ctest entry per test case instead of one for the whole executable (see the
    # registration below).
    cmake_parse_arguments(ARG "PER_CASE" "" "" ${ARGN})

    # Unit tests
    if(NOT BUILD_TESTING)
        return()
    endif()

    set(COMPONENT_TEST_NAME wmtk_test_${COMPONENT_NAME})

    option(WMTK_ENABLE_COMPONENT_TEST_${COMPONENT_NAME} "Enable wmtk component test for ${COMPONENT_NAME}" ON)
    
    IF(NOT ${WMTK_ENABLE_COMPONENT_TEST_${COMPONENT_NAME}})
        return()
    ENDIF()

    message(STATUS "Add component test ${COMPONENT_TEST_NAME}.")

    add_executable(${COMPONENT_TEST_NAME})
    target_link_libraries(${COMPONENT_TEST_NAME} PRIVATE
        wmtk::${COMPONENT_NAME}
        Catch2::Catch2WithMain
        wmtk::warnings 
        wmtk::toolkit
        wmtk::data
    )

    # Group source files for IDEs
    file(GLOB_RECURSE COMPONENTS_FILES_FOR_SOURCE_GROUP "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp" "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
    source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${COMPONENTS_FILES_FOR_SOURCE_GROUP})

    set_target_properties(${COMPONENT_TEST_NAME} PROPERTIES FOLDER wmtk_components_tests)

    # Register with ctest. Without this the executable is built and never run, by CI or by
    # anyone typing `ctest` locally -- which is how two suites came to sit broken on main
    # (tetwild's surface-swap case and three shortest_edge_collapse cases), each failing
    # against behaviour that had changed underneath it while nothing was watching.
    #
    # By default one entry per component rather than catch_discover_tests: most components'
    # whole sets run in seconds or a few minutes, so per-case granularity is not worth it there.
    # PER_CASE below is for one that does not.
    # --allow-running-no-tests because c1_simplification's two cases are currently commented
    # out, so its executable holds nothing and Catch2 exits non-zero on an empty run. Without
    # the flag that component alone would fail the moment these are registered.
    # --durations yes prints each case's time, which one entry per component would otherwise
    # hide: it is how a case too slow for the Debug jobs (Windows Debug above all) is found, and
    # tagged release_only in its file.
    #
    # PER_CASE registers every case as its own entry (catch_discover_tests, as tests/ registers
    # wmtk_tests), named "<component>/<case>", so a parallel ctest runs a component's cases side
    # by side and each has ctest's default timeout to itself. For a component whose cases add up
    # to more than that timeout: topological_offset's ran 742 s on Linux Release and 1077 s on
    # Windows Release on main (CI, 2026-11-17), and past the 1500 s timeout on both Windows jobs
    # with PR #1180's cases. The module path is set here from Catch2's own source directory.
    if(ARG_PER_CASE)
        list(APPEND CMAKE_MODULE_PATH ${Catch2_SOURCE_DIR}/extras)
        include(Catch)
        catch_discover_tests(${COMPONENT_TEST_NAME} TEST_PREFIX "${COMPONENT_NAME}/")
    else()
        add_test(NAME ${COMPONENT_TEST_NAME} COMMAND ${COMPONENT_TEST_NAME} --allow-running-no-tests --durations yes)
        # COST: a whole component's cases in one entry, the longest a few minutes (tetwild),
        # so a parallel ctest starts these before the single-case entries.
        set_tests_properties(${COMPONENT_TEST_NAME} PROPERTIES COST 100)
    endif()
    wmtk_copy_dll(${COMPONENT_TEST_NAME})

endfunction()
